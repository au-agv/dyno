#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( D ) ( Y ) ( N ) ( O )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#              DYNO: Ground Vehicle Dynamics Validation Toolkit             #
#############################################################################

#  MIT License
#
#  DYNO: Ground Vehicle Dynamics Validation Toolkit
#  Copyright (c) 2024 Dario Sirangelo (dsi@aarhusrobotics.com).
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to
#  deal in the Software without restriction, including without limitation the
#  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
#  sell copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in
#  all copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
#  IN THE SOFTWARE.

from __future__ import annotations

import argparse
import json
import logging
import shlex
import subprocess

from os import getcwd
from pathlib import Path
from typing import List

from pydyno.postprocess import PostprocessorJSON, PostprocessorHDF5

logger = logging.getLogger(__name__)


class Entrypoint:

    def __init__(self: Entrypoint) -> None:

        self._parser: argparse.ArgumentParser = self._register_parser()
        self._args: argparse.Namespace = self._parse_arguments()

    def _register_parser(self: Entrypoint) -> argparse.ArgumentParser:

        parser = argparse.ArgumentParser(
            prog='DYNO launcher',
            description=
            'Launch preprocessing, simulation and postprocessing for a DYNO simulation',
            epilog='Copyright Dario Sirangelo 2024')

        self._add_arguments(parser)

        return parser

    def _add_arguments(self, parser) -> None:

        parser.add_argument("--input-filter",
                            required=False,
                            action='store_true')

        parser.add_argument("--parameters-file", required=False)

        parser.add_argument("--launch-file", required=False)

        parser.add_argument("--output-files",
                            nargs='+',
                            default=(),
                            required=False)

        parser.add_argument("--python-includes",
                            nargs='+',
                            default=(),
                            required=False)

        parser.add_argument("--output-filter", required=False)

        parser.add_argument("--mode", default="standalone", required=False)

        parser.add_argument("--driver", required=False)

        parser.add_argument("--asynchronous", required=False)

        parser.add_argument("--scenario", required=False)

        parser.add_argument("--config", required=False)

        parser.add_argument("--log", default="warn", required=False)

    def _parse_arguments(self: Entrypoint) -> argparse.Namespace:

        return self._parser.parse_args()

    def _preprocess(self) -> None:
        """Launch the preprocessor."""

        PreprocessorWrapper(self._args.parameters_file,
                            self._args.python_includes,
                            self._args.output_files)._preprocess()

        if self._args.mode == "ros":
            logger.warning(
                "The ROS preprocessor may be missing Rosbag setup functionality!"
            )

    def _execute(self) -> None:

        if self._args.mode == "ros":
            #current_environment = os.environ.copy()

            # Compute the ROS domain ID for the current thread.
            #with open(self._args.parameters_file, "r") as file:
            #    file.read

            #if(self._args.asynchronous):
            #    id = self._args.asynchronous

            #current_environment["ROS_DOMAIN_ID"] = f"/usr/sbin:/sbin:{my_env['PATH']}"

            subprocess.run(
                shlex.split(
                    f"ros2 launch dyno_simulation {self._args.launch_file}.launch.py parameters_overrides:={(Path.cwd() / Path(self._args.parameters_file)).as_posix()} default_path:={getcwd()}"
                ))
        elif self._args.mode == "standalone":
            simulator = SimulatorWrapper(self._args.driver, self._args.scenario,
                                         self._args.config)
            simulator.run()
        else:
            raise Exception

    def _postprocess(self) -> None:

        if self._args.mode == "ros":
            postprocessor = RosPostprocessorWrapper(self._args.output_filter)
        elif self._args.mode == "standalone":
            postprocessor = PostprocessorHDF5Wrapper("case.h5", "none.h5")
            postprocessor.to_dakota_results(self._args.output_filter,
                                            self._find_descriptors())
        else:
            raise Exception

    def _find_descriptors(self) -> List[str]:

        responses: List[str] = []

        with open(self._args.parameters_file, 'r') as file:
            parameters = json.load(file)
        for response in parameters["responses"]:
            responses.append(response["label"])

        return responses

    def launch(self) -> None:

        if self._args.input_filter:
            self._preprocess()
        elif self._args.driver:
            self._execute()
        elif self._args.output_filter:
            self._postprocess()
        else:
            raise Exception


class PreprocessorWrapper:

    def __init__(
        self: PreprocessorWrapper,
        parameters_file: str,
        python_includes=(),
        output_files=()) -> None:

        self._parameters_file = Path(parameters_file)

        self._python_includes = [Path(file) for file in python_includes]

        assert len(
            output_files
        ) > 0, "At least one output file is required for preprocessing."
        assert [Path(file).exists() for file in output_files]
        self._output_files = [Path(file) for file in output_files]

    def run(self: PreprocessorWrapper) -> None:

        self._preprocess()

    def _preprocess(self: PreprocessorWrapper) -> None:

        logger.info("Running the DYNO preprocessor ...")

        with open(self._parameters_file.as_posix(), "r") as file:
            parameters = json.load(file)
        with open("variables.json", "w") as file:
            active_variables = {}
            for variable in parameters["variables"]:
                active_variables[variable["label"]] = variable["value"]
            json.dump(active_variables, file)

        if len(self._python_includes) > 0:
            python_includes = f"--include {''.join([include.as_posix() for include in self._python_includes])}"
        else:
            python_includes = ""

        for output_file in self._output_files:

            template_file = output_file.with_suffix(".tpl" + output_file.suffix)

            subprocess.run(
                shlex.split(
                    f"pyprepro {python_includes} --json-include variables.json {template_file.as_posix()} {output_file.as_posix()}"
                ))


class PostprocessorWrapper:

    def __init__(self: PostprocessorWrapper, input_path: Path,
                 output_path: Path) -> None:

        # Path of the results file to be postprocessed.
        self._input_path = Path(input_path)
        self._output_path = Path(output_path)

    def run(self: PostprocessorWrapper) -> None:

        logger.debug("Running postprocessor ...")
        #self._postprocess()


class PostprocessorHDF5Wrapper(PostprocessorWrapper):

    def __init__(self: DynoJSONPostprocessorWrapper, input_path: Path,
                 output_path: Path) -> None:

        super().__init__(input_path, output_path)
        self._postprocessor = PostprocessorHDF5(self._input_path)

    def postprocess(self: PostprocessorHDF5Wrapper,
                    responses: List[str]) -> None:

        pass

    def to_dakota_results(self, path, responses) -> None:

        self._postprocessor.compute_statistics(responses)
        self._postprocessor.save_statistics(path, responses)


class DynoJSONPostprocessorWrapper(PostprocessorWrapper):
    """Postprocessor wrapper for DYNO simulations.
    """

    def __init__(self: DynoJSONPostprocessorWrapper, input_path: Path,
                 output_path: Path) -> None:
        """Create a DYNO postprocessor wrapper.

        Args:
            input_path (Path): Path to the DYNO simulation results file.
            output_path (Path): Path to the postprocessed results output file to
            be written.
        """

        super().__init__(self, input_path, output_path)

    def _postprocess(self: DynoJSONPostprocessorWrapper) -> None:
        """Postprocess the loaded. DYNO results file.

        At the current time, this method can only postprocess results from DYNO vehicle simulations, as it relies on the downstream VehiclePostprocess class.
        """

        postprocessor = PostprocessorJSON.from_file(self._input_path)

        # Save the desired statistics.
        postprocessor.save_statistics(
            self._output_path,
            ["average_speed", "max_speed", "average_throttle"])


class RosPostprocessorWrapper(PostprocessorWrapper):
    """Wrapper for the ROTO Rosbag postprocessor.
    """

    def __init__(self, output_path: Path) -> None:
        """Create a ROTO Rosbag postprocessor.

        Args:
            input_path (Path): Path to the Rosbag.
            bag_semantics (Path): Path to the Rosbag semantics definition file.
            output_path (Path): Path to the postprocessed results file.
        """

        mydict = {"functions": {"response_fn_1": 0.1}}
        with open(output_path, 'w') as file:
            json.dump(mydict, file)
        #super().__init__(input_path, output_path)
        #self._bag_semantics: Path = Path(bag_semantics)

    def run(self: RosPostprocessorWrapper) -> None:

        self._postprocess()

    def _postprocess(self: RosPostprocessorWrapper) -> None:

        logger.info("Running ROS postprocessor wrapper ...")

        #postprocessor = RotoPostprocessor(self._input_path,
        #                                  self._bag_semantics)

        #postprocessor.save_statistics(
        #    self._output_path,
        #    ["average_speed", "max_speed", "average_throttle"])


class SimulatorWrapper():

    def __init__(self: SimulatorWrapper, executable_name: Path, scenario: str,
                 config_file: Path) -> None:

        self._executable_name: Path = Path(executable_name)
        self._scenario = scenario
        self._config_file: Path = Path(config_file)

    def run(self: SimulatorWrapper) -> None:

        logger.info("Running executable \"%s\" ...", self._executable_name)

        subprocess.run(
            shlex.split(
                f"{self._executable_name.as_posix()} --scenario {self._scenario} --options {self._config_file.as_posix()}"
            ))
