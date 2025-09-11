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

import importlib.resources
import json
import logging
import os
import shutil
import subprocess

import coloredlogs
import numpy as np

from pathlib import Path

from pydyno.postprocess import PostprocessorJSON

logger = logging.getLogger(__name__)
coloredlogs.install(level=logger.level, logger=logger)


class Generator:

    def __init__(self, uid: int):

        self._uid: int = uid

    def run(self, executable: Path, template_name: str):

        template: Path = importlib.resources.path(
            "pydyno", "data") / f"scenarios/{template_name}.json"

        logger.info("Running case %s", template_name.upper())
        subprocess.run([executable, "--options", template],
                       cwd=Path.home(),
                       check=True)

        output_directory: Path = self.get_output_directory(template)
        output_directory = self.rename_directory(output_directory)

        self.extract_metadata(output_directory /
                              Path(str(output_directory) + ".json"))

        # Postprocessor
        postprocessor = PostprocessorJSON.from_metadata_file(
            output_directory / Path(str(output_directory) + ".json"))

        df = postprocessor.get_dataframe()
        df["pose.position.x"] = df["pose.position.x"] + np.random.normal(
            0.0, 0.75, df["pose.position.x"].shape)
        df["pose.position.x"] = df["pose.position.x"].apply(
            lambda n: self.mutate(n, 0.0, 15.0))
        df["pose.position.x"].plot()
        import matplotlib.pyplot as plt
        plt.show()
        #postprocessor.save_csv(output_directory.parent / Path("here.csv"))

        #logger.info("Compressing output.")
        #compress(output_directory)

    def mutate(n, d_min, d_max, mutate_chance=0.05, round=True):
        r = np.random.random()
        d = d_min + np.random.random() * (d_max - d_min)
        if r < mutate_chance / 2:
            # mutate high
            return int(n + d) if round else n + d
        elif r < mutate_chance:
            # mutate low
            return int(n - d) if round else n - d

        return n

    def rename_directory(self, path):

        old_results_file: Path = path / f"{path.stem}.json"
        new_results_file: Path = path / f"DLC194-{self.uid}.json"
        os.rename(old_results_file, new_results_file)
        logger.info("Renamed simulation results file from '%s' to '%s'",
                    old_results_file.stem, new_results_file.stem)

        old_results_folder: Path = path
        new_results_folder: Path = path.parent / f"DLC194-{self.uid}"
        os.rename(old_results_folder, new_results_folder)
        logger.info("Renamed simulation results folder from '%s' to '%s'",
                    old_results_folder.stem, new_results_folder.stem)

        return new_results_folder / Path(f"DLC194-{self.uid}")

    def get_output_directory(template_path):
        with open(Path(template_path), "r", encoding="UTF-8") as file:
            output_path: Path = json.load(file)["output"]["path"]
        path = Path.home() / Path(output_path)
        latest = max(Path(path).glob('*/'), key=os.path.getmtime)
        return latest

    def extract_metadata(results_path):
        with open(Path(results_path), "r", encoding="UTF-8") as file:
            metadata: Path = json.load(file)["metadata"]
        with open(results_path.parent /
                  Path(f"{results_path.stem}_metadata.json"),
                  'w',
                  encoding="UTF-8") as file:
            json.dump(metadata, file)

    def compress(path: Path):

        logger.info("Compressing file %s to .zip under '%s'", path.stem, path)
        shutil.make_archive(path, 'zip', path)
        #os.removedirs(path)

class VehicleDatasetGenerator(DatasetGenerator):

    def __init__(self, data, name):

        super().__init__(data, name)

class DatasetGenerator(Postprocessor):

    def __init__(self: DatasetGenerator, data: Dict, name: str = "Unnamed"):

        super().__init__(data, name)

    def save_dataset_info(self: DatasetGenerator, path: Path) -> None:

        with open(path / Path(f"{self._name}.json"), 'w',
                  encoding="UTF-8") as file:
            json.dump(self._metadata, file)

