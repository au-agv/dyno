import argparse
import logging

from pathlib import Path

import coloredlogs
import filedialpy

from pydyno.postprocess import PostprocessorHDF5

logger = logging.getLogger("pydyno")
coloredlogs.install(level='DEBUG', logger=logger)


def main():

    logger.info("Running postprocessor ...")
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--load", nargs="?", default=Path.cwd())
    parser.add_argument("--report", nargs="?", default=None)
    parser.add_argument("-v", "--visualize", action="store_true")

    args = parser.parse_args()

    path: Path = Path(args.load)
    if path.is_dir():
        path = Path(filedialpy.openFile(path))

    postprocessor = PostprocessorHDF5(path)

    if args.visualize:
        postprocessor.visualize()

    if args.report:
        postprocessor.plot()

    if args.report is not None:
        report_path: Path = Path(args.report)
        postprocessor.save_to_disk(report_path)


if __name__ == "__main__":
    main()
