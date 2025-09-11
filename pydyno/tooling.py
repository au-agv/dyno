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


import functools
import importlib
import json
import logging
import math
import re
import secrets
import warnings

from itertools import count
from pathlib import Path

import coloredlogs
import matplotlib.pyplot as plt
import numpy as np
import tables

from matplotlib.axes import Axes


class JSONWithCommentsDecoder(json.JSONDecoder):
    """
    A custom JSON decoder that can handle comments within the JSON string,
    specifically those starting with "//".

    This class inherits from Python's built-in `json.JSONDecoder` and overrides
    its default behavior to ignore lines starting with "//" during decoding.
    """

    def __init__(self, **kw):
        """
        Initializes the JSON comment-aware decoder instance.

        Args:
            \\*kw: Arbitrary keyword arguments to be passed to the parent class
            initializer.
        """
        super().__init__(**kw)

    def decode(self, s: str):
        """
        Decodes a string containing potentially commented-out JSON data and
        returns the parsed Python object.

        Args:
            s (str): The input string which may include lines starting with "//"
            as comments.

        Returns:
            dict or list: The decoded Python object representing the JSON data
            in `s`, excluding any lines that start with "//".
        """

        s = "\n".join(
            l for l in s.split("\n") if not l.lstrip(" ").startswith("//")
        )
        return super().decode(s)


def deep_getattr(obj, attr):
    """
    Recursively gets an attribute from an object using dot notation.

    This static method uses `functools.reduce` to apply `getattr` repeatedly
    on the string representation of the attribute path. It splits the
    attribute string by dots and applies getattr to each part recursively.

    Args:
        obj: The base object from which to get the attribute. attr (str): A
        dot-separated string representing the sequence of attributes to
        access.

    Returns:
        The value of the accessed attribute or `None` if any attribute in
        the chain is not found.
    """

    return functools.reduce(getattr, attr.split("."), obj)


@staticmethod
def configure_logger(level="info") -> None:
    """
    Configure the logger to use coloredlogs for improved readability.

    This method sets up the logger to use the 'coloredlogs' library, which
    enhances the readability of log messages by adding colors based on their
    severity levels. It installs the logger at INFO level.

    Args:
        self: The instance of the class calling this method.

    Returns:
        None
    """

    def decorator(function):
        @functools.wraps(function)
        def wrapper(*args, **kwargs):
            coloredlogs.install(
                level=logging.getLevelName(level),
                logger=logging.getLogger(),
                fmt="%(asctime)s [%(levelname)s] %(message)s",
                datefmt="%H:%M:%S",
            )
            return function(*args, **kwargs)

        return wrapper

    return decorator


def configure_plotter() -> None:
    """
    Configure matplotlib with convenient default options and a custom style.

    This method sets up the matplotlib configuration to disable warnings
    about the maximum number of open figures and attempts to load a custom
    DYNO style from the package resources. If the style file is not found,
    it logs a warning message.

    Args:
        self: The instance of the class calling this method.

    Returns:
        None
    """
    logging.info("Configuring matplotlib ...")

    # Disable the warning for the maximum number of open figures.
    plt.rcParams["figure.max_open_warning"] = False

    # Attempt to find the custom matplotlib style in the package resources.
    logging.info("Loading the DYNO matplotlib style file ...")
    try:
        plt.style.use(
            importlib.resources.files("pydyno").joinpath(
                "data/plotter/dyno.mplstyle"
            )
        )
    except (ModuleNotFoundError, OSError, TypeError) as exception:
        logging.warning('DYNO matplotlib style not found: "%s"', exception)


def trim_x_axis(func):
    """
    Decorator that trims the x-axis to the actual plotted data after the function runs.
    Ignores NaNs in the x-data.
    Assumes the wrapped function has an 'axes' keyword argument.
    """

    @functools.wraps(func)
    def wrapper(*args, axes: Axes = None, **kwargs):
        # Run the plotting function
        result = func(*args, axes=axes, **kwargs)

        if axes is not None:
            # Collect all x-data from lines in the axes
            xs = [line.get_xdata() for line in axes.get_lines()]
            if xs:
                x_all = np.hstack(xs)
                x_valid = x_all[~np.isnan(x_all)]
                if len(x_valid) > 0:
                    axes.set_xlim(x_valid.min(), x_valid.max())
                    axes.margins(x=0)  # remove default matplotlib padding

        return result

    return wrapper


def autoscale_axes(func):
    """
    Decorator that recomputes data limits and updates view limits
    on an Axes object after the wrapped function executes.
    Assumes the wrapped function has an `axes` keyword argument.
    """

    @functools.wraps(func)
    def wrapper(*args, axes: Axes = None, **kwargs):
        result = func(*args, axes=axes, **kwargs)

        if axes is not None:
            axes.relim()  # recompute the data limits
            axes.autoscale_view()  # update the view limits

        return result

    return wrapper


def with_axes(func):
    """
    Decorator that ensures a function has an Axes object and optionally sets:
    - title
    - xlabel
    - ylabel
    - legend
    Returns the Axes object.
    """

    @functools.wraps(func)
    def wrapper(
        *args,
        axes: Axes | None = None,
        title: str | None = None,
        xlabel: str | None = None,
        ylabel: str | None = None,
        legend: bool = False,
        **kwargs,
    ):
        # Create new axes if not provided
        if axes is None:
            fig, axes = plt.subplots()
        else:
            fig = axes.figure

        # Call the wrapped function
        result = func(*args, axes=axes, **kwargs)

        # Set axes properties
        if title is not None:
            axes.set_title(title)
        if xlabel is not None:
            axes.set_xlabel(xlabel)
        if ylabel is not None:
            axes.set_ylabel(ylabel)
        if legend:
            axes.legend()

        # Always return axes
        return axes

    return wrapper


def show_figure(func):
    """
    Decorator to optionally call plt.show() if `show=True` is passed.
    Assumes the wrapped function receives an Axes object (axes=...).
    """

    @functools.wraps(func)
    def wrapper(*args, axes: Axes | None = None, show: bool = False, **kwargs):
        # If axes is not passed, create one
        created_ax = False
        if axes is None:
            fig, axes = plt.subplots()
            created_ax = True
        else:
            fig = axes.figure

        # Call the plotting function
        result = func(*args, axes=axes, **kwargs)

        # Show the figure if requested
        if show:
            plt.show()
        else:
            plt.close(axes.figure)

        # Return the function result or axes
        return result if result is not None else axes

    return wrapper


def save_figure(func):
    """Decorator to optionally save a figure associated with an axes."""

    @functools.wraps(func)
    def wrapper(*args, axes=None, save=False, **kwargs):
        # The plotting function must always receive axes
        if axes is None:

            fig, axes = plt.subplots()
        else:
            fig = axes.figure

        result = func(*args, axes=axes, **kwargs)

        if save:
            # Determine path
            if save is True:
                # If axes has a title, use it; otherwise fallback to function name
                title = axes.get_title()
                if title:
                    filename = to_snake_case(title) + ".pdf"
                else:
                    filename = func.__name__ + ".pdf"
                path = Path(filename)
            else:
                path = Path(save)

            fig.savefig(path)
            print(f"Figure saved to {path.resolve()}")

        # Return the axes or the function result
        return result if result is not None else axes

    return wrapper


def to_snake_case(s: str) -> str:
    """
    Converts a string to snake_case.
    Example: "My Plot Title" -> "my_plot_title"
    """
    s = s.strip()
    s = re.sub(r"[^\w\s]", "", s)  # remove punctuation
    s = re.sub(r"\s+", "_", s)  # replace spaces with underscores
    return s.lower()


def show():

    def decorator(function):

        @functools.wraps(function)
        def wrapper(*args, **kwargs):
            if kwargs.pop("show", False):
                plt.show()
            return function(*args, **kwargs)

        return wrapper

    return decorator


def suppress_warnings():

    warnings.simplefilter("ignore", category=tables.UnclosedFileWarning)


def get_script_directory(file):

    return Path(file).resolve().parent


def generate_short_uid():

    return secrets.token_hex(4)


def remove_intermediate_extension(path):

    return f"{path.stem.split('.')[0]}{path.suffix}"


def sanitize(obj):

    def dakota_safe(x):
        if x is None or (isinstance(x, float) and math.isnan(x)):
            return -1.0e30
        return x

    if isinstance(obj, dict):
        return {k: sanitize(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [sanitize(v) for v in obj]
    return dakota_safe(obj)


def convert_camelcase_to_title(string):
    # Insert space before each capital letter (except the first)
    return re.sub(r"(?<!^)(?=[A-Z])", " ", string).lower().capitalize()
