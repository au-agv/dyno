#!/usr/bin/env python3

#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( D ) ( Y ) ( N ) ( O )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#              DYNO: Ground Vehicle Dynamics Validation Toolkit             #
#############################################################################

import subprocess


def main():

    result = subprocess.run(
        ["dyno", "--scenario", "doubleLaneChange", "--configuration", "config.json"],
        capture_output=True,
        text=True)
    print(result.stdout)


if __name__ == '__main__':
    main()
