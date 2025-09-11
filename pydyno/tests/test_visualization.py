from pathlib import Path

import filedialpy

from pydyno.visualize import Plotter


def main():

    path: Path = Path(
        filedialpy.openFile(
            "/home/dsirangelo/Repositories/github.com/aarhus-robotics/dyno/pydyno/cases/split_mu/parameter_study/"
        ))

    plotter = Plotter(path, "default")

    # Plot the input variables
    plotter.plot_variable(0, 0)
    plotter.plot_variable(0, 1)
    plotter.plot_variable(0, 2)

    # Plot the distribution of the response for each variable
    plotter.plot_variable_histogram(0)

    # Plot the histogram of the responses for each variable
    plotter.plot_response_histogram(0)
    plotter.plot_response_histogram(1)

    # Plot the error bars for the responses
    plotter.plot_responses_errorbars()
    
    plotter.show()


if __name__ == "__main__":
    main()
