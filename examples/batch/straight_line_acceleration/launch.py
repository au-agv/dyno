#!/usr/bin/env python

from pydyno.launchers_new import ScenarioLauncher


def main():

    case = ScenarioLauncher(scenario="straightLineAcceleration")
    case._run_asynchronous(processes=8)


if __name__ == "__main__":
    main()
