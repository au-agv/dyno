#!/usr/bin/env python3

#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( D ) ( Y ) ( N ) ( O )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#              DYNO: Ground Vehicle Dynamics Validation Toolkit             #
#############################################################################

#  MIT License
#
#  DYNO: Ground Vehicle Dynamics Validation Toolkit
#  Copyright (c) 2024 Dario Sirangelo (dsi@aarhusrobotics.com).
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to
#  deal in the Software without restriction, including without limitation the
#  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
#  sell copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in
#  all copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
#  IN THE SOFTWARE.


import pydyno.tooling

from pydyno.processing.databases import (
    MultiHierarchicalDataProcessor,
    TimeSeriesProcessor,
)


@pydyno.tooling.configure_logger(level="info")
def main():

    processor = MultiHierarchicalDataProcessor(
        pydyno.tooling.get_script_directory(__file__).parents[0]
        # / "dakota/grade_climbing/runs"
        / "dakota/double_lane_change/runs"
    )

    # Plot a response based on a metadata entry
    # processor.plot_response("frictionCoefficient", "velocity.linear.x")

    # processor.plot_response(
    #    "frictionCoefficientLeft",
    #    "velocity.linear.y",
    #    additional_metadata={"success": 1.0},
    #    range=(3, 100.0),
    # )
    # processor.plot_responses_offset("targetSlope", "velocity.linear.x")

    # Plot a response surface based on a metadata pair
    # processor.plot_response_surface(
    #    metadata_field_x="frictionCoefficient",
    #    metadata_field_y="targetSlope",
    #    data_entry="velocity.linear.x",
    # )

    # figure = processor.plot_mean(
    #    data_entry="pose.position.y", range=(10.0, None)
    # )

    # figure = processor.plot_mean(
    #    data_entry="vehicle.slip_angle", range=(10.0, None)
    # )

    # figure = processor.plot_mean(
    #    data_entry="velocity.angular.z", range=(10.0, None)
    # )

    # ------------------------------------------------------------------------ #
    # Plot a variables correlation heatmap.
    # ------------------------------------------------------------------------ #
    processor.plot_correlation_heatmap(
        variables=[
            "velocity.linear.x",
            "velocity.angular.z",
            "pose.rotation.x",
            "velocity.angular.x",
        ],
        functions={
            "velocity.linear.x": TimeSeriesProcessor._max,
            "velocity.angular.z": TimeSeriesProcessor._max,
            "pose.rotation.x": TimeSeriesProcessor._max,
            "velocity.angular.x": TimeSeriesProcessor._max,
        },
        labels=(
            r"Velocity X (Max)",
            r"Yaw rate $\psi$ (Max)",
            r"Roll angle $\phi$ (Max)",
            r"Roll rate $\dot{\phi}$ (Max)",
        ),
        save=False,
        show=False,
    )

    # ------------------------------------------------------------------------ #
    # Plot a specific time series for one of the runs.
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    # Plot a response surface.
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    # Plot a time series heatmap.
    # ------------------------------------------------------------------------ #

    # There is a bug on the metadata selection!! It is not applying the fcn
    # coorectly.
    processor.plot_timeseries_heatmap(
        "velocity.angular.z",
        {
            # "targetSpeed": lambda x: x > 5.0 and x <5.8,
            "frictionCoefficientLeft": lambda x: x < 0.85 and x > 0.25,
            # "sideslope": lambda x: x > 1.0 and x < 9.51,
        },
        sort_order=["frictionCoefficientLeft"],
        title="Yaw rate for ($\mu > 0.8$, $v > 5 m s^{-1}$)",
        save=False,
        show=False,
    )

    # ------------------------------------------------------------------------ #
    # Plot a scatter plot.
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    # Plot a generic (X-Y) series.
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    # Plot a time series with mean value and errorbars.
    # ------------------------------------------------------------------------ #
    processor.plot_mean(
        data_entry="pose.position.y",
        range=(10.0, None),
        title="Lateral displacement Y",
        xlabel="Time [s]",
        ylabel="Displacement [$m$]",
        legend=False,
        save=False,
        show=False,
    )

    processor.plot_mean_nontime(
        data_entry="pose.position.x",
        data_entry2="wheels.0.force.z",
        range=(0.0, None),
        title="Velocity X",
        xlabel="Time [s]",
        ylabel="Velocity [$m s^{-1}$]",
        legend=False,
        save=False,
        show=True,
    )

    processor.plot_mean(
        data_entry="velocity.angular.z",
        range=(10.0, None),
        title="Yaw rate",
        xlabel="Time [s]",
        ylabel="Yaw rate [$rad s^{-1}$]",
        legend=False,
        save=False,
        show=False,
    )

    processor.plot_mean(
        data_entry="velocity.angular.x",
        range=(10.0, None),
        title="Roll rate",
        xlabel="Time [s]",
        ylabel="Angle [$rad s^{-1}$]",
        legend=False,
        save=False,
        show=False,
    )

    # Plot response

    # Plot pair


if __name__ == "__main__":
    main()
