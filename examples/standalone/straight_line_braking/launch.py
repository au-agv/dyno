#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( D ) ( Y ) ( N ) ( O )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#              DYNO: Ground Vehicle Dynamics Validation Toolkit             #
#############################################################################

import subprocess
import shlex


def main():

    subprocess.run(
        shlex.split("dyno --scenario straightLineBraking --options config.json")
    )


if __name__ == "__main__":
    main()
