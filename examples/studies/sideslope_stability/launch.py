#!/usr/bin/env python

import json
import logging
import subprocess

from shlex import split

import numpy as np

from pydyno.utilities import JSONWithCommentsDecoder


def main():

    slopes = np.arange(25, 35, 5, dtype=np.float64)
    speeds = np.arange(5, 15, 2.5, dtype=np.float64)

    print(slopes)
    print(speeds)

    with open("include/config.json", "r") as file:
        configuration = json.load(file, cls=JSONWithCommentsDecoder)

    for speed in speeds:
        for slope in slopes:
            logging.info("Launching case for slope {slope / 100.0}% ...", slope)

            configuration["scenario"]["targetSpeed"] = speed
            configuration["scenario"]["gradePercentage"] = slope
            configuration["output"][
                "filename"
            ] = f"sideslopeStability_{speed:0.0f}ms_{slope:0.0f}pct"

            with open("include/configCurrent.json", "w") as file:
                json.dump(configuration, file)

            subprocess.run(
                split(
                    "dyno --scenario sideslopeStability --options include/configCurrent.json"
                )
            )


if __name__ == "__main__":
    main()
