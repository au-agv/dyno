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

import contextlib
import csv
import importlib.resources
import json
import logging
import os
import typing

from abc import abstractmethod
from pathlib import Path
from typing import Dict, Optional, List, Tuple

#import dtale
import matplotlib
import matplotlib.axes
import matplotlib.figure
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import tables

from numpy.typing import NDArray
from pypdf import PdfMerger
from scipy.signal import savgol_filter

logger = logging.getLogger()
logger.setLevel(logging.INFO)


class Label:

    def __init__(self: Label):

        pass


class Quantity:

    def __init__(self: Quantity, data: NDArray, label: Label):

        self._data: NDArray = data
        self._label: Label = label


class Figure:

    def __init__(self: Figure,
                 title: str,
                 figure: matplotlib.Figure,
                 filename: Optional[str] = None):

        self.title: str = title
        self.filename: str = title.replace(
            " ", "_").lower() if filename is None else filename
        self.figure: matplotlib.Figure = figure


class VectorEntry:

    def __init__(self, entry: str):

        self._entry = entry


class Postprocessor:

    def __init__(self: Postprocessor, name: str = "Untitled") -> None:

        self._name: str = name
        self._time: NDArray = np.empty((0, ))

        # Configure dataframe parsing capabilities
        self._dataframe: pd.DataFrame = pd.DataFrame()

        # Configure plotting capabilities
        self._configure_matplotlib()
        self._figures: List[Figure] = []

    @classmethod
    def from_file(cls: Postprocessor, path: Path, **kwargs):

        with open(path, "r", encoding="UTF-8") as file:
            data = json.load(file)
            return cls(data, name=Path(path).stem, **kwargs)

    def _load_data(self: Postprocessor, path: Path):

        logger.info("Loading results file ...")

        with open(path, "r", encoding="UTF-8") as file:
            try:
                self._data = json.load(file)
                logger.info("Successfully loaded results file at \"%s\"",
                            path.as_posix())
            except json.JSONDecodeError:
                logger.error("Unsupported JSON syntax")
                raise

    def _configure_matplotlib(self) -> None:
        """Configure matplotlib with convenient default options and a custom style.
        """

        logger.info("Configuring matplotlib ...")

        # Disable the warning for the maximum number of open figures.
        plt.rcParams["figure.max_open_warning"] = False

        # Attempt to find the custom matplotlib style in the package resources.
        logger.info("Loading the DYNO matplotlib style file ...")
        try:
            plt.style.use(
                importlib.resources.path("pydyno", "data") /
                "matplotlib/dyno.mplstyle")
        except (ModuleNotFoundError, OSError, TypeError) as exception:
            logger.warning("DYNO matplotlib style not found: \"%s\"", exception)

    def _initialize_time_array(self):

        self._time = np.linspace(
            self._settings["start_time"],
            self._settings["end_time"],
            int(self._settings["end_time"] / self._settings["time_step"]) + 1,
            endpoint=True,
        )

    @abstractmethod
    def plot(self: Postprocessor) -> None:
        logger.error("Not implemented!")

    @abstractmethod
    def _convert_to_degrees(self: Postprocessor) -> None:
        logger.error(
            "The method to convert angular quantities to degrees was not implemented for this postprocessor."
        )

    @abstractmethod
    def _create_dataframe(self: Postprocessor) -> None:

        logger.error(
            "The method to create a Pandas Dataframes was not implemented for this postprocessor."
        )

    def get_dataframe(self: Postprocessor) -> pd.DataFrame:

        return self._dataframe

    def show(self):

        plt.show()

    def save_csv(self: PostprocessorJSON, path: Path) -> None:
        """Export the Pandas DataFrame to a comma-separated-values (CSV) file at
        the specified path."""

        self._dataframe.to_csv(Path(path) / "results.csv")

    def save_figures(self, path: str):

        if not Path(path).exists():
            logger.warning(
                'Output folder did not previously exist. Creating folder "%s"',
                Path(path).as_posix(),
            )
            Path(path).mkdir(parents=True)

        for entry in self._figures:
            filename = entry.filename
            figure = entry.figure

            logger.info('Saving figure "%s" to file %s.%s', entry.title,
                        filename, "pdf")
            figure.savefig(f"{Path(path)}/{filename}.pdf", format="pdf")

    def save_report(self, path: Path):
        # Initialise a PDF merger object.
        merger = PdfMerger()
        merger.set_page_layout("/TwoColumnLeft")
        merger.set_page_mode("/UseOutlines")

        # Initialise a page index for bookmarking.
        page_index = 0

        for figure in self._figures:
            # Add page to the output PDF file.
            merger.append(Path(f'{path}/{figure.filename}.pdf'))

            # Add bookmark entry for the figure and increase counter.
            merger.add_outline_item(figure.title, page_index)
            page_index += 1

        merger.write(f"{path}/{self._name} Report.pdf")
        merger.close()

    def _delete_files(self, path):

        path = Path(Path.cwd() / path)

        for entry in self._figures:
            filename = entry["filename"]

            logger.info("Removing file %s.pdf", filename)
            os.unlink(f"{path}/{filename}.pdf")

    def visualize(self: Postprocessor) -> None:

        pass
        #with contextlib.redirect_stdout(None):
        #    self._dtale = dtale.show(self._dataframe,
        #                             name=self._name,
        #                             subprocess=True)
        #    logging.getLogger().handlers.clear()
        #self._dtale.open_browser()
        #logger.info(f"D-Tale is available at {self._dtale._main_url}")
        #input("Press [ENTER] to continue ...")
        #self._dtale.kill()


class PostprocessorJSON(Postprocessor):
    """Postprocessor for Project Chrono vehicle simulations."""

    def __init__(self: PostprocessorJSON,
                 data: Dict,
                 name: str = "Untitled") -> None:

        super().__init__(data, name)

        self.use_degrees = False

        self.coeffs_butter = []

        self.wheels_indices = [
            "Front Left", "Front Right", "Rear Left", "Rear Right"
        ]

        self.limits = {}

        super().__post_init__()

    # pylint: disable-next=arguments-differ
    def _create_dataframe(self) -> None:
        """Create a Pandas DataFrame from the JSON simulation output file."""

        # Populate the time series
        self._dataframe["time"] = self._data["time"]

        # Populate the pose time series
        for component in ("x", "y", "z"):
            self._dataframe[f"pose.position.{component}"] = self._data[
                f"pose_position_{component}"]
            self._dataframe[f"pose.orientation.{component}"] = self._data[
                f"pose_orientation_{component}"]

            # Populate the velocity time series.
            self._dataframe[f"velocity.linear.{component}"] = self._data[
                f"velocity_linear_{component}"]
            self._dataframe[f"velocity.angular.{component}"] = self._data[
                f"velocity_angular_{component}"]

            # Populate the acceleration time series.
            self._dataframe[f"acceleration.linear.{component}"] = self._data[
                f"acceleration_linear_{component}"]
            self._dataframe[f"acceleration.angular.{component}"] = self._data[
                f"acceleration_angular_{component}"]

            # Populate the wheel time series.
            for index, label in zip(
                (0, 1, 2, 3),
                ("front_left", "front_right", "rear_left", "rear_right")):
                self._dataframe[
                    f"wheels.{label}.position.{component}"] = self._data[
                        "wheels"][index]["position"][component]
                self._dataframe[
                    f"wheels.{label}.rotation.{component}"] = self._data[
                        "wheels"][index]["rotation"][component]
                self._dataframe[
                    f"wheels.{label}.force.{component}"] = self._data["wheels"][
                        index]["force"][component]
                self._dataframe[
                    f"wheels.{label}.moment.{component}"] = self._data[
                        "wheels"][index]["moment"][component]

    def set_filter(self,
                   n=16,
                   omega_n: float = 5.0e-6,
                   f_s: typing.Optional[float] = None) -> None:
        # Deduce the sampling frequency by examining the time stamps of the
        # first to messages under the assumption of consistent sampling.
        if f_s is None:
            f_s = self._data["time"][1] - self._data["time"][0]

        # Update the Butterworth filter coefficients.
        self.coeffs_butter = sps.butter(n, omega_n, fs=f_s, output="sos")

    def plot_trajectory(self):
        logger.info("Plotting vehicle trajectory.")

        figure, axes = plt.subplots()

        # Set plot title, axes labels.
        axes.set_xlabel("X Position (m)")
        axes.set_ylabel("Y Position (m)")
        axes.set_title("Trajectory")

        axes.set_xlim(
            min(self._data["pose_position_x"]),
            max(self._data["pose_position_x"]),
        )
        axes.set_ylim(
            min(self._data["pose_position_y"]),
            max(self._data["pose_position_y"]),
        )

        axes.plot(self._data["pose_position_x"], self._data["pose_position_y"])

        self._figures.append(Figure("Trajectory", "trajectory", figure))

    def _set_x_axis_limits(self, axes):
        axes.set_xlim([self._data["time"][0], self._data["time"][-1]])

    def filter_tire_forces(self):
        """Filter high frequency components from the forces and moments
        generated by the tire model."""

        logger.info("Filtering tire forces...")

        for index in range(4):
            for entry in ("force", "moment"):
                for component in ("x", "y", "z"):
                    self._data["wheels"][index][entry][
                        component] = self._filter(
                            self._data["wheels"][index][entry][component])

    def filter_accelerations(self):
        """Filter high frequency components from the acceleration components."""

        logger.info("Filtering accelerations...")

        for entry in ("linear", "angular"):
            for component in ("x", "y", "z"):
                self._data["wheels"][entry][component] = self._filter(
                    self._data["wheels"][entry][component])

    def get_average_speed(self):

        return np.mean(self._data["velocity"]["linear"]["x"])


class PostprocessorHDF5(Postprocessor):
    """A postprocessor class for DYNO HDF5 simulation output."""
    _ENTRIES = ("time", VectorEntry("pose/position"),
                VectorEntry("pose/rotation"), VectorEntry("velocity/linear"),
                VectorEntry("velocity/angular"),
                VectorEntry("acceleration/linear"),
                VectorEntry("acceleration/angular"), "commands/throttle",
                "commands/brake", "commands/steering")

    _FUNCTION_MAP = {
        "max": np.max,
        "min": np.min,
        "mean": np.min,
        "median": np.median,
        "start": lambda x: np.array(x)[0],
        "end": lambda x: np.array(x)[-1]
    }

    _PLOTTING_DATA = {
        "pose/position/x": {
            "title": "Position (X)",
            "xlabel": "Time (s)",
            "ylabel": "Position ($m$)"
        },
        "pose/position/y": {
            "title": "Position (Y)",
            "xlabel": "Time (s)",
            "ylabel": "Position ($m$)"
        },
        "pose/position/z": {
            "title": "Position (Z)",
            "xlabel": "Time (s)",
            "ylabel": "Position ($m$)"
        },
        "pose/rotation/x": {
            "title": "Roll angle (X)",
            "xlabel": "Time (s)",
            "ylabel": "Angle ($deg$)"
        },
        "pose/rotation/y": {
            "title": "Pitch angle (Y)",
            "xlabel": "Time (s)",
            "ylabel": "Angle ($deg$)"
        },
        "pose/rotation/z": {
            "title": "Yaw angle (Z)",
            "xlabel": "Time (s)",
            "ylabel": "Angle ($deg$)"
        },
        "velocity/linear/x": {
            "title": "Longitudinal velocity",
            "xlabel": "Time (s)",
            "ylabel": "Velocity ($m/s$)"
        },
        "velocity/linear/y": {
            "title": "Lateral velocity",
            "xlabel": "Time (s)",
            "ylabel": "Velocity ($m/s$)"
        },
        "velocity/linear/z": {
            "title": "Vertical velocity",
            "xlabel": "Time (s)",
            "ylabel": "Velocity ($m/s$)"
        },
        "velocity/angular/x": {
            "title": "Roll rate",
            "xlabel": "Time (s)",
            "ylabel": "Angular velocity ($deg/s$)"
        },
        "velocity/angular/y": {
            "title": "Pitch rate",
            "xlabel": "Time (s)",
            "ylabel": "Angular velocity ($deg/s$)"
        },
        "velocity/angular/z": {
            "title": "Yaw rate",
            "xlabel": "Time (s)",
            "ylabel": "Angular velocity ($deg/s$)"
        },
        "acceleration/linear/x": {
            "title": "Longitudinal acceleration",
            "xlabel": "Time (s)",
            "ylabel": "Acceleration ($m/s^2$)"
        },
        "acceleration/linear/y": {
            "title": "Lateral acceleration",
            "xlabel": "Time (s)",
            "ylabel": "Acceleration ($m/s^2$)"
        },
        "acceleration/linear/z": {
            "title": "Vertical acceleration",
            "xlabel": "Time (s)",
            "ylabel": "Acceleration ($m/s^2$)"
        },
        "acceleration/angular/x": {
            "title": "Roll acceleration",
            "xlabel": "Time (s)",
            "ylabel": "Angular acceleration ($deg/s^2$)"
        },
        "acceleration/angular/y": {
            "title": "Pitch acceleration",
            "xlabel": "Time (s)",
            "ylabel": "Angular acceleration ($deg/s^2$)"
        },
        "acceleration/angular/z": {
            "title": "Yaw acceleration",
            "xlabel": "Time (s)",
            "ylabel": "Angular acceleration ($deg/s^2$)"
        },
    }

    def __init__(self, path: Path, name: str = "Untitled") -> None:
        """Create a new DYNO HDF5 simulation output postprocessor.

        Args:
            path (Path): Path to the DYNO HDF5 simulation output file, including
            the ".h5" extension.
        """

        super().__init__(name)

        self._path: Path = path
        """Path to the DYNO HDF5 simulation output file."""

        # Filter settings
        self._filter_order: int = 3
        self._filter_window: int = 200

        # Statistics
        self._statistics = {}

        self._dataframe = pd.DataFrame()

        # Load and parse the DYNO HDF5 simulation output file.
        self._file = None
        self._open_file()
        self._parse()

    def __del__(self) -> None:
        """Destroy the DYNO HDF5 simulation output postprocessor."""

        if (self._file is not None and self._file.isopen):
            logger.info("Closing HDF5 file at \"%s\" ...", self._path)
            self._file.close()

    def _open_file(self) -> None:
        """Load the DYNO HDF5 simulation output file."""

        try:
            logger.info("Opening HDF5 file at \"%s\" ...", self._path)
            self._file = tables.open_file(self._path, mode="r")
        except:
            logger.error("Could not open file!")
            exit(os.EX_IOERR)

    def _parse(self) -> None:

        for entry in self._ENTRIES:

            if type(entry) == VectorEntry:
                logging.info('Parsing vector entry \"%s/{x,y,z}\" ...', entry._entry)
                self._dataframe[entry._entry + "/x"] = self._file.get_node(
                    "/data", entry._entry)[:, 0]
                self._dataframe[entry._entry + "/y"] = self._file.get_node(
                    "/data", entry._entry)[:, 1]
                self._dataframe[entry._entry + "/z"] = self._file.get_node(
                    "/data", entry._entry)[:, 2]
            else:
                logging.info('Parsing scalar entry \"%s\" ...', entry)
                self._dataframe[entry] = self._file.get_node("/data", entry)

        self._filter_accelerations()

    def _get_component(self, component: str) -> Tuple[str, int]:

        valid_components: Tuple[str] = ("x", "y", "z")

        if component not in valid_components:
            raise ValueError("\"%s\" is not a valid component identifier!")

        return component, ("x", "y", "z").index(component)

    def _get_direction_friendly_name(self, component: str) -> str:
        return ("yaw", "pitch", "roll")[self._get_component(component)[1]]

    def _filter_accelerations(self: PostprocessorHDF5) -> None:

        for column in self._dataframe:
            if column.startswith("acceleration"):
                self._dataframe[column] = savgol_filter(self._dataframe[column],
                                                        self._filter_window,
                                                        self._filter_order)

    def _get_or_create_figure_and_axes(
        self,
        figure: matplotlib.Figure = None,
        axes: matplotlib.Axes = None
    ) -> Tuple[matplotlib.Figure, matplotlib.Axes]:

        if figure is None and axes is None:
            return plt.subplots()
        elif figure is None and axes is not None:
            figure = plt.figure()
            figure.add_axes(axes)
            return figure, axes
        elif figure is not None and axes is None:
            return figure, figure.axes()
        else:
            return figure, axes

    def _plot(
        self,
        x: np.ndarray,
        y: np.ndarray,
        axes: matplotlib.Axes = None,
        title: str = "",
        xlabel: str = "",
        ylabel: str = ""
    ) -> Tuple[matplotlib.figure.Figure, matplotlib.axes.Axes]:

        figure, axes = self._get_or_create_figure_and_axes(axes)
        axes.plot(x, y)
        axes.set_xlim(min(x), max(x))
        #axes.set_ylim()
        figure.canvas.manager.set_window_title(title)
        axes.set_title(title)
        axes.set_xlabel(xlabel)
        axes.set_ylabel(ylabel)

        return figure, axes

    def _plot_quantity(self, quantity, axes: matplotlib.Axes = None):

        entry = self._PLOTTING_DATA[quantity]

        self._figures.append(
            Figure(title=entry["title"],
                   figure=self._plot(x=self._dataframe["time"],
                                     y=self._dataframe[quantity],
                                     axes=axes,
                                     title=entry["title"],
                                     xlabel=entry["xlabel"],
                                     ylabel=entry["ylabel"])[0]))

    def plot_all(self: PostprocessorHDF5) -> None:

        self.plot_position()
        self.plot_velocity()
        self.plot_acceleration()
        self.plot_commands()

    def plot_pose(self: PostprocessorHDF5):

        self.plot_positions()
        self.plot_rotations()

    def plot_positions(self, axes: matplotlib.Axes = None):

        self.plot_position("x")
        self.plot_position("y")
        self.plot_position("z")

    def plot_position(self, component: str = "x", axes: matplotlib.Axes = None):

        self._plot_quantity("pose/position/" + component)

    def plot_rotations(self):

        self.plot_rotation("x")
        self.plot_rotation("y")
        self.plot_rotation("z")

    def plot_rotation(self,
                      component: str = "x",
                      axes: matplotlib.Axes = None,
                      degrees=True):

        self._plot_quantity("pose/rotation/" + component)

    def plot_acceleration(self: PostprocessorHDF5):

        self.plot_linear_acceleration("x")
        self.plot_linear_acceleration("y")
        self.plot_linear_acceleration("z")
        self.plot_angular_acceleration("x")
        self.plot_angular_acceleration("y")
        self.plot_angular_acceleration("z")

    def plot_linear_acceleration(self,
                                 component: str = "x",
                                 axes: matplotlib.Axes = None):

        self._plot_quantity("acceleration/linear/" + component)

    def plot_angular_acceleration(self,
                                  component: str = "x",
                                  axes: matplotlib.Axes = None,
                                  degrees=True):

        self._plot_quantity("acceleration/angular/" + component)

    def plot_commands(self, axes: matplotlib.Axes = None):

        self.plot_command("throttle")
        self.plot_command("brake")
        self.plot_command("steering")

    def plot_command(self, control: str, axes: matplotlib.Axes = None):

        figure, axes = self._plot(x=self._dataframe["time"],
                                  y=self._dataframe[f"commands/{control}"],
                                  axes=axes,
                                  title=f"{control.capitalize()}",
                                  xlabel="Time (s)",
                                  ylabel=f"{control.capitalize()} (-)")

        # Set the Y axis limit to valid control efforts
        axes.set_ylim([-1.0, 1.0]) if control == "steering" else axes.set_ylim(
            [0.0, 1.0])

        self._figures.append(
            Figure(f"{control.capitalize()} effort", figure=figure))

    def plot_velocity(self: PostprocessorHDF5) -> None:

        self.plot_linear_velocity("x")
        self.plot_linear_velocity("y")
        self.plot_linear_velocity("z")
        self.plot_angular_velocity("x")
        self.plot_angular_velocity("y")
        self.plot_angular_velocity("z")

    def plot_linear_velocity(self,
                             component: str = "x",
                             axes: matplotlib.Axes = None):

        self._plot_quantity("velocity/linear/" + component)

    def plot_angular_velocity(
        self,
        component: str = "x",
        degrees=True,
        axes: matplotlib.Axes = None,
    ) -> None:

        self._plot_quantity("velocity/angular/" + component)

    def plot_angular_velocity(self,
                              component: str = "x",
                              axes: matplotlib.Axes = None):

        self._plot_quantity("velocity/angular/" + component)

    def compute_statistics(self, quantities: List[str]) -> None:

        for quantity in quantities:
            key_found = False
            for key, function in self._FUNCTION_MAP.items():
                if quantity.startswith(key):
                    key_found = True
                    self._statistics[quantity] = function(
                        self._dataframe[quantity[len(key) + 1:].replace(
                            "_", "/")])
            if not key_found:
                self._file.close()
                raise Exception("Quantity identifier not recognized.")

    def save_statistics(self, file_path: Path, quantities: List[str]) -> None:

        with open(file_path, "w", encoding="UTF-8") as file:
            json.dump({"functions": self._statistics}, file)

    """
    def plot_wheel_speeds(self):
        # Instantiate a new figure-axes tuple.
        figure, axes = plt.subplots()

        # Plot the lateral tire forces for all tires.
        for index in range(4):
            axes.plot(self._data["time"], self._data["wheels"][index]["speed"])

        # Set axes limits
        axes.set_xlim([self._data["time"][0], self._data["time"][-1]])

        # Set plot title, axes labels and legend.
        axes.set_xlabel("Time (s)")
        axes.set_ylabel("Rotational speed (rad/s)")
        axes.set_title("Wheel speeds")
        axes.legend(["Front Left", "Front Right", "Rear Left", "Rear Right"],
                    loc="best")

        # Append figure handle to the list of open figures.
        self._figures.append(Figure("Wheel speeds", "wheel_speeds", figure))

    def plot_slip(self: PostprocessorJSON):

        figure, axes = plt.subplots()

        axes.plot(self._data["time"], self._data["alpha_fl"])
        axes.plot(self._data["time"], self._data["alpha_fr"])
        axes.plot(self._data["time"], self._data["alpha_rl"])
        axes.plot(self._data["time"], self._data["alpha_rr"])

        """


class MultipostprocessorHDF5():

    def __init__(self: MultipostprocessorHDF5,
                 case_workspace_path: Path,
                 run_name: str = "run") -> None:

        self._case_workspace_path: Path = Path(
            case_workspace_path
        ) if case_workspace_path is not Path else case_workspace_path

        assert self._case_workspace_path.is_dir(
        ), "Case workspace path must be a directory!"

        self._postprocessors: List[PostprocessorHDF5] = []

        from natsort import natsorted
        run_folders: List = natsorted(
            self._case_workspace_path.rglob("**/run*"))

        for run_index, run_path in enumerate(run_folders):
            logger.info("Postprocessing case #%i", run_index)
            self._postprocessors.append(
                PostprocessorHDF5(run_path / "results.h5"))

    def plot(self: MultipostprocessorHDF5, labels: Tuple[str]) -> None:

        time = self._postprocessors[0].get_dataframe()["time"]

        for postprocessor in self._postprocessors:
            plt.plot(time, postprocessor.get_dataframe()["velocity/linear/x"])
        plt.legend(labels)
        plt.show()
