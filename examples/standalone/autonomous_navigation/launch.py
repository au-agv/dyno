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
        ["dyno", "--scenario", "autonomousNavigation", "--configuration", "config.json"],
        text=True)
    print(result.stdout)
    print(result.stderr)


if __name__ == '__main__':
    main()
