from __future__ import annotations

import argparse
import logging

from pathlib import Path

import filedialpy

from pydyno.postprocess import PostprocessorHDF5

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--path", help="Path to a file or folder to postprocess.")
    args = parser.parse_args()

    path = args.path if args.path else Path.cwd().as_posix()
    path: Path = Path(filedialpy.openFile())

    output = PostprocessorHDF5(path)

    output.visualize()

    # output.filter_accelerations()

    # output.plot_position()
    # output.plot_rotation()

    # output.plot_linear_velocity("x")
    # output.plot_linear_velocity("y")
    # output.plot_linear_velocity("z")

    # output.plot_angular_velocity("x")
    # output.plot_angular_velocity("y")
    # output.plot_angular_velocity("z")

    # output.plot_angular_acceleration("x")
    # output.plot_angular_acceleration("y")
    # output.plot_angular_acceleration("z")

    # output.plot_controls()

    # plt.show()


if __name__ == "__main__":
    main()
