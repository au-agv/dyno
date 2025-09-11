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

import logging

from pathlib import Path
from typing import List, Optional

import coloredlogs
import h5py
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
#import seaborn as sns

logger = logging.getLogger(__name__)
coloredlogs.install(level='WARN', logger=logger)

#sns.set_theme()
#sns.set_style("darkgrid", {"axes.facecolor": ".9"})
#sns.set_context("paper")


class Plotter:
    """Plotter for DAKOTA runs simulation results.

    This plotter loads the results of a DAKOTA case from an HDF5 file and allows
    to plot response funcionts and various statistics.
    """

    def __init__(self: Plotter,
                 path: Path = "./results.h5",
                 model_name: str = "default",
                 interface_name: str = "default",
                 method_name: str = "default") -> None:
        """Create a new plotter.

        Args:
            path (Path, optional): Path to the DAKOTA HDF5 results file. Defaults to "./results.h5".
            model_name (str, optional): Key for the DAKOTA model name.
            interface_name (str, optional): Key for the DAKOTA interface name.
            method (str, optional): Key for the DAKOTA method name.
        """

        # Store the keys to access the results in the hierarchical structure.
        self._model_name = model_name
        self._interface_name = interface_name
        self._method_name = method_name

        # Store the path to and open the HDF5 results file.
        self._results_file_path: Path = path
        self._open(path)

    def _open(self: Plotter, path: Path) -> None:
        """Open and parse a DAKOTA HDF5 results file from disk.

        Args:
            path (Path): Path to the DAKOTA HDF5 results file.
        """

        logger.info("Opening results file ...")
        self._results_file = h5py.File(path, "r")

        # Store the variables descriptors in a list.
        self._variables_descriptors: List = [
            str(descriptor, 'utf-8') for descriptor in
            self._results_file["_scales"]["models"]["simulation"][
                self._model_name]["variables"]["continuous_descriptors"]
        ]

        # Store the responses descriptors in a list.
        self._responses_descriptors: List = [
            str(descriptor, 'utf-8') for descriptor in
            self._results_file["_scales"]["models"]["simulation"][
                self._model_name]["responses"]["function_descriptors"]
        ]

    def list_variables(self):

        variables = self._results_file["_scales"]["models"]["simulation"][
            self._model_name]["variables"]
        for index, _ in enumerate(list(variables["continuous_ids"])):
            logger.debug(
                "Variables %s %s %s", int(variables["continuous_ids"][index]),
                str(variables["continuous_descriptors"][index], 'utf-8'),
                str(variables["continuous_types"][index], 'utf-8'))

    def plot_variable_histogram(self, variable_index: int):

        parameters = self._results_file["methods"][
            self._method_name]["sources"]["default"]

        figure, axes = plt.subplots(1, 1)

        sns.histplot(pd.DataFrame(
            parameters["variables"]["continuous"][:, variable_index]),
                     ax=axes)

        axes.set_title(
            f"{self._variables_descriptors[variable_index].replace('_', ' ').capitalize()} variable histogram"
        )

        axes.set_xlabel(self._variables_descriptors[variable_index].replace(
            "_", " ").capitalize())

    def plot_response_histogram(self, response_index: int):

        parameters = self._results_file["methods"][
            self._method_name]["sources"]["default"]

        figure, axes = plt.subplots(1, 1)

        sns.histplot(pd.DataFrame(
            parameters["responses"]["functions"][:, response_index]),
                     ax=axes)

        axes.set_title(
            f"{self._responses_descriptors[response_index].replace('_', ' ').capitalize()} response histogram"
        )

        axes.set_xlabel(self._responses_descriptors[response_index].replace(
            "_", " ").capitalize())

    def plot_responses_errorbars(self):

        _, axes = plt.subplots(1, 1)

        legend_entries = []
        for descriptor in self._responses_descriptors:
            legend_entries.append(descriptor.replace("_", " ").capitalize())

        sns.pointplot(
            pd.DataFrame(
                np.array(self._results_file["methods"][self._method_name]
                         ["sources"]["default"]["responses"]["functions"])),
            errorbar=("sd", 1),
            capsize=0.3,
            linestyles="none",
            ax=axes,
        )

    def plot_respose_errorbar(self, response_index: int):

        parameters = self._results_file["methods"][
            self._method_name]["sources"]["default"]

        frame = pd.DataFrame(
            parameters["responses"]["functions"][:, response_index])

        _, axes = plt.subplots(1, 1)

        sns.pointplot(frame, errorbar="sd", capsize=0.3, ax=axes)

    def plot_variable(self,
                      variable_index: int,
                      response_index: int,
                      title: Optional[str] = None,
                      xlabel: Optional[str] = None,
                      ylabel: Optional[str] = None):
        # Plot response 1 for variable 1
        figure, axes = plt.subplots(1, 1)

        parameters = self._results_file["methods"][
            self._method_name]["sources"]["default"]

        axes.scatter(parameters["variables"]["continuous"][:, variable_index],
                     parameters["responses"]["functions"][:, response_index])

        # Set the X axis label automatically unless otherwise specified.
        if xlabel is None:
            xlabel = self._variables_descriptors[variable_index].replace(
                "_", " ").capitalize()
        axes.set_xlabel(xlabel)

        # Set the Y axis label automatically unless otherwise specified.
        if ylabel is None:
            ylabel = self._responses_descriptors[response_index].replace(
                "_", " ").capitalize()
        axes.set_ylabel(ylabel)
        if title is None: title = f"\"{xlabel}\" vs \"{ylabel}\""
        axes.set_title(title)

    def show(self: Plotter) -> None:
        """Wrapper method to show all plotted figures.
        """

        plt.show()
