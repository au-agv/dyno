#!/usr/bin/env python


import json
import logging
import subprocess

from multiprocessing import Pool
from shlex import split

from numpy import arange

from pydyno.utilities import JSONWithCommentsDecoder


class Case:

    def __init__(self):

        self._load_configuration()

    def _load_configuration(self):

        with open("include/config.json", "r") as file:
            self._configuration = json.load(file, cls=JSONWithCommentsDecoder)

    def _run_case(self, slope):
        logging.info("Launching case for slope {slope / 100.0}% ...", slope)

        case_number = str(slope)

        case_configuration = self._configuration

        case_configuration["scenario"]["gradePercentage"] = slope
        case_configuration["output"][
            "filename"
        ] = f"straightLineAcceleration_rigid_{slope:0.0f}pct"

        with open(f"include/config_{case_number}.json", "w") as file:
            json.dump(case_configuration, file)

        subprocess.run(
            split(
                f"dyno --scenario straightLineAcceleration --options include/config_{case_number}.json"
            )
        )


def main():

    friction_coefficients = arange(0.1, 0.8, 0.1)
    slopes = arange(0.0, 30.0, 5.0)

    case = Case()
    with Pool(processes=4) as pool:
        pool1 = pool.map_async(
            case._run_case,
            (slopes),
        )
        pool1.wait()


if __name__ == "__main__":
    main()
