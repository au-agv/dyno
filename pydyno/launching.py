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
#  Copyright (c) 2024 Dario Sirangelo (dev@dariosirangelo.me).
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

import argparse
import json
import logging
import os
import shlex
import subprocess
import sys

from pathlib import Path
from typing import Dict, List, Optional

import pydyno.tooling

from pydyno.processing.databases import UniHierarchicalDataProcessor


class Launcher:

    def __init__(self):

        self._create_parser()


class LauncherDyno(Launcher):

    def __init__(self):

        super().__init__()

    def _create_parser(self):

        self._parser = argparse.ArgumentParser(
            description="DYNO process launcher for standalone usage.",
            epilog="Copyright Aarhus University Autonomous Ground Vehicles (c) 2024",
        )

        self._parser.add_argument(
            "scenario",
            type=str,
            help="Scenario name.",
            metavar="scenario",
        )

        self._parser.add_argument(
            "configuration",
            type=Path,
            help="Path to the scenario configuration file.",
            metavar="configuration",
        )

        self._parser.add_argument(
            "--debug",
            "-d",
            action="store_true",
            help="Whether to use the debugger.",
        )

    def _parse_parser(self):

        self._parser_args = self._parser.parse_args()

    def run(self):

        self._parse_parser()
        self._run_simulator(
            scenario=self._parser_args.scenario,
            configuration=self._parser_args.configuration,
            debug=self._parser_args.debug,
        )

    def _run_simulator(
        self, scenario: str, configuration: Path, debug: bool = False
    ):

        subprocess.run(
            shlex.split(
                f"{'gdb -ex run --args ' if debug else ''} dyno --scenario"
                f" {scenario} --configuration {configuration.as_posix()}"
            )
        )


class LauncherDakota(Launcher):

    def __init__(self):

        super().__init__()

    def _create_parser(self):

        self._parser = argparse.ArgumentParser(
            description=(
                "DYNO process launcher and supervisor for case studies using"
                " Sandia National Laboratories Dakota."
            ),
            epilog="Copyright Aarhus University Autonomous Ground Vehicles (c) 2024",
        )
        self._create_subparsers()
        self._create_subparser_analysis_driver()
        self._create_subparser_input_filter()
        self._create_subparser_output_filter()

    def _create_subparsers(self):

        self._parser_subparsers = self._parser.add_subparsers(
            dest="verb",
            required=True,
        )

    def _create_subparser_input_filter(self):

        subparser = self._parser_subparsers.add_parser(
            "input_filter",
            help="Parser for the Dakota input filter.",
        )

        subparser.add_argument(
            "--template-files",
            required=True,
            type=Path,
            nargs="+",
            help=(
                "List of paths to the configuration templates on which to"
                " perform parameter substitution using the Dakota preprocessor."
            ),
            metavar="template-files",
        )

        subparser.add_argument(
            "parameters",
            type=Path,
            help=(
                "Path to the parameters file generated by the Dakota"
                " executable."
            ),
            metavar="parameters-file",
        )

        subparser.add_argument(
            "--python-includes",
            "-P",
            nargs="+",
            type=Path,
            help=(
                "Path to the optional Python scripts to be executed before the"
                " Dakota preprocessor."
            ),
            default=[],
            metavar="pythonIncludes",
        )

        subparser.add_argument(
            "--json-includes",
            "-J",
            nargs="+",
            type=Path,
            help=(
                "Path to the optional JSON dictionaries to be parsed before the"
                " Dakota preprocessor."
            ),
            default=[],
            metavar="jsonIncludes",
        )

    def _run_input_filter(
        self,
        template_files: List[Path],
        parameters_file: Path,
        json_includes: Optional[List[Path]] = None,
        python_includes: Optional[List[Path]] = None,
    ):

        self._process_templates(
            template_files, parameters_file, json_includes, python_includes
        )

    def _process_templates(
        self,
        template_files: List[Path],
        parameters_file: Path,
        json_includes: Optional[List[Path]] = None,
        python_includes: Optional[List[Path]] = None,
        substitutions_file: Optional[Path] = Path(
            f"substitutions_{pydyno.tooling.generate_short_uid()}.json"
        ),
    ):
        """
        Processes a list of template files using the pyprepro preprocessor.

        Args:
            template_files: A list of template files to be processed.

            parameters_file: The path to the parameters file containing
                             substitution variables.

            json_includes: Optional list of JSON files to include as
                           substitutions.

            python_includes: Optional list of Python files to include as
                             substitutions.

            substitutions_file: Optional path to a temporary JSON file for
                                substitutions.

        Raises:
            RuntimeError: If the pyprepro command fails.
        """
        for template_file in template_files:
            logging.info(
                'Processing template file "%s" ...', template_file.stem
            )

            # Create a temporary substitutions JSON dictionary on disk with the
            # Dakota variables in a format compatible with pyprepro.
            self.__class__._convert_to_json_include(
                parameters_file, substitutions_file
            )

            # Run the `pyprepro` preprocessor.
            python_includes: str = (
                f"{self.__class__._unwrap_multiple_arguments('python-include', python_includes)}"
                if python_includes is not None
                else ""
            )
            json_includes: str = (
                f"--json-include {substitutions_file}"
                if json_includes is not None
                else ""
            )
            command: str = (
                f"pyprepro {python_includes} "
                f"{json_includes} {template_file.as_posix()} "
                f"{pydyno.tooling.remove_intermediate_extension(template_file)}"
            )
            print(command)
            subprocess.run(shlex.split(command))

            # Delete the temporary substitutions JSON dictionary file and the
            # user-provided template (we assume this is just a symlink to the
            # original template in the Dakota case root directory).
            substitutions_file.unlink()
            template_file.unlink()

    def run(self, args):

        self._parse_args(args)
        self._process_args()

    def _parse_args(self, args):

        self._parser_args = self._parser.parse_args(args)

    def _process_args(self):

        if "--help" in self._parser_args or "-h" in self._parser_args:
            self._parser.print_help()
            sys.exit(0)
        if self._parser_args.verb == "analysis_driver":
            self._parse_subparser_analysis_driver()
        elif self._parser_args.verb == "input_filter":
            self._parse_subparser_input_filter()
        elif self._parser_args.verb == "output_filter":
            self._parse_subparser_output_filter()

    def _parse_subparser_input_filter(self):
        logging.info(
            'Launching input filter with templates "%s", JSON includes'
            ' "%s", Python include "%s" and parameters file "%s" ...',
            ", ".join(
                [
                    template.as_posix()
                    for template in self._parser_args.template_files
                ]
            ),
            ", ".join(
                [
                    include.as_posix()
                    for include in self._parser_args.json_includes
                ]
            ),
            ", ".join(
                [
                    include.as_posix()
                    for include in self._parser_args.python_includes
                ]
            ),
            self._parser_args.parameters,
        )
        self._run_input_filter(
            self._parser_args.template_files,
            self._parser_args.parameters,
            json_includes=self._parser_args.json_includes,
            python_includes=self._parser_args.python_includes,
        )

    @staticmethod
    def _convert_to_json_include(
        parameters_file: Path, substitutions_output: Path
    ):

        substitutions = {}

        with open(parameters_file, "r") as file:
            parameters = json.load(file)

            for entry in parameters["variables"]:
                substitutions[entry["label"]] = entry["value"]
            substitutions["eval_id"] = parameters["eval_id"]
            substitutions["workspace"] = Path.cwd().as_posix()

        with open(substitutions_output, "w") as file:
            json.dump(substitutions, file)

    @staticmethod
    def _unwrap_multiple_arguments(keyword: str, args):

        if args is not None:
            return " ".join(
                [f"--{keyword} {include.as_posix()}" for include in args]
            )
        else:
            return ""


class LauncherDakotaStandalone(LauncherDakota):

    def __init__(self):

        super().__init__()

    def _create_subparser_analysis_driver(self):

        subparser = self._parser_subparsers.add_parser(
            "analysis_driver",
            help=(
                "Parser for the Dakota analysis driver command line arguments."
            ),
        )

        subparser.add_argument(
            "scenario",
            type=str,
            choices=[
                "autonomousNavigation",
                "doubleLaneChange",
                "straightLineAcceleration",
            ],
            help="Scenario name to run.",
            metavar="scenarioName",
        )

        subparser.add_argument(
            "configuration",
            type=Path,
            help="Path to the scenario configuration file.",
            metavar="configurationFilePath",
        )

    def _create_subparser_output_filter(self):

        subparser = self._parser_subparsers.add_parser(
            "output_filter",
            help="Parser for the Dakota output filter command line arguments.",
        )

        subparser.add_argument(
            "parameters",
            type=Path,
            help=(
                "Path to the parameters template file as provided by the Dakota"
                " executable."
            ),
            metavar="parametersTemplate",
        )

        subparser.add_argument(
            "--database",
            "-d",
            default=None,
            type=Path,
            help=(
                "Path to the parameters processed file as expected by the"
                " Dakota executable."
            ),
            metavar="database",
        )

        subparser.add_argument(
            "results",
            type=Path,
            help=(
                "Path to the results processed file as expected by the Dakota"
                " executable."
            ),
            metavar="results",
        )

    def _parse_subparser_analysis_driver(self):

        logging.info(
            'Launching analysis driver for scenario "%s" with configuration'
            ' file "%s" ...',
            self._parser_args.scenario,
            self._parser_args.configuration.as_posix(),
        )
        self._run_analysis_driver(
            self._parser_args.scenario, self._parser_args.configuration
        )

    def _parse_subparser_output_filter(self):

        logging.info(
            'Launching output filter with parameters includes "%s",'
            ' simulation database "%s" writing to results "%s" ...',
            self._parser_args.parameters,
            self._parser_args.database,
            self._parser_args.results,
        )
        self._run_output_filter(
            self._parser_args.parameters,
            self._parser_args.results,
            database=self._parser_args.database,
        )

    def _run_analysis_driver(self, scenario: str, parameters: Path):

        subprocess.run(
            shlex.split(
                f"dyno --scenario {scenario} --configuration"
                f" {parameters.as_posix()}"
            )
        )

    def _run_output_filter(
        self,
        parameters: Path,
        results: Path,
        database: Optional[Path],
    ):

        if database is None:
            database = max(
                parameters.parent.glob("*.h5"),
                key=lambda properties: properties.stat().st_ctime,
                default=None,
            )

        with open(parameters, "r") as file:
            labels: List[str] = [
                response["label"] for response in json.load(file)["responses"]
            ]

        postprocessor = UniHierarchicalDataProcessor(database)

        with open(results, "w") as file:
            json.dump(
                {
                    "functions": dict(
                        [
                            (
                                label,
                                pydyno.tooling.sanitize(
                                    postprocessor.get_timeseries_filtered(label)
                                ),
                            )
                            for label in labels
                        ]
                    ),
                    "fail": not postprocessor.get_metadata("success"),
                },
                file,
            )

        postprocessor.close()


class LauncherDakotaROS(LauncherDakota):

    def __init__(self):

        super().__init__()

    def _create_subparser_analysis_driver(self):

        subparser = self._parser_subparsers.add_parser(
            "analysis_driver",
            help=(
                "Parser for the Dakota analysis driver command line arguments"
                " using Docker Compose."
            ),
        )

        subparser.add_argument(
            "launch_file",
            type=Path,
            help="Path to the ROS launch Python file.",
            metavar="launch_file",
        )

        subparser.add_argument(
            "configuration_file",
            type=Path,
            help="Path to the DYNO configuration JSON file.",
            metavar="configuration_file",
        )

        subparser.add_argument(
            "parameters",
            type=Path,
            help="Path to the ROS parameters YAML file.",
            metavar="parameters_file",
        )

        subparser.add_argument(
            "--launch-arguments",
            "-l",
            type=Path,
            help="Additional ROS launch file arguments.",
            metavar="launch_arguments",
        )

    def _parse_subparser_analysis_driver(self):

        logging.info(
            "Launching DYNO ROS analysis driver with launch file '%s',"
            " configuration file '%s' and parameters file '%s' ...",
            self._parser_args.launch_file,
            self._parser_args.configuration_file,
            self._parser_args.parameters,
        )
        self._run_analysis_driver(
            launch_file=self._parser_args.launch_file,
            configuration_file=self._parser_args.configuration_file,
            parameters_file=self._parser_args.parameters,
            launch_arguments=self._parser_args.launch_arguments,
        )

    def _run_analysis_driver(
        self,
        launch_file: Path,
        configuration_file: Path,
        parameters_file: Path,
        launch_arguments: Optional[List[str]],
        environment: Optional[Dict] = None,
    ):

        logging.info("Running case ...")

        # Override the ROS bag name using the environment variables exposed in
        # the Docker Compose file.
        process_environment = os.environ.copy()
        if environment is not None:
            for key, value in environment:
                process_environment[key] = value

        configuration_arg: str = (
            f"configuration:={configuration_file.as_posix()}"
        )
        parameters_arg: str = f"parameters_file:={parameters_file.as_posix()}"
        subprocess.run(
            shlex.split(
                "ros2 launch"
                f" {launch_file.as_posix()} {configuration_arg} {parameters_arg} {launch_arguments}"
            ),
            env=process_environment,
        )

    def _run_output_filter(
        self, parameters: Path, database: Path, results: Path
    ):

        pass

    def _create_subparser_output_filter(self):

        pass

    def _parse_subparser_output_filter(self):
        pass


class LauncherSelectorDakota:

    def __init__(self):

        self._create_parser()

    def _create_parser(self):

        self._parser = argparse.ArgumentParser(add_help=False)

        self._parser.add_argument(
            "type",
            choices=["standalone", "ros"],
            nargs="?",
            default="standalone",
        )

    def _parse_parser(self):

        known_args, forward_args = self._parser.parse_known_args()

        if known_args.type == "standalone":
            LauncherDakotaStandalone().run(forward_args)
        elif known_args.type == "ros":
            LauncherDakotaROS().run(forward_args)
        else:
            raise ValueError("Unknown launcher type.")

    def run(self):

        self._parse_parser()
