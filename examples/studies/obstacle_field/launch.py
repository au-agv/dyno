#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( D ) ( Y ) ( N ) ( O )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#              DYNO: Ground Vehicle Dynamics Validation Toolkit             #
#############################################################################

import subprocess
import json


def main():

    with open("config.json", "r") as file:
        config = json.load(file)

    speeds = (3.0, 5.0)

    for speed in speeds:
        config["scenario"]["targetSpeed"] = speed

        with open("currentConfig.json", "w") as file:
            json.dump(config, file)

        result = subprocess.run(
            [
                "dyno",
                "--scenario",
                "autonomousNavigation",
                "--options",
                "currentConfig.json",
            ],
            text=True,
        )
        print(result.stdout)
        print(result.stderr)


if __name__ == "__main__":
    main()
