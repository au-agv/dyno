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

from argparse import ArgumentParser
from pydyno.postprocess import PostprocessorHDF5, MultipostprocessorHDF5


class PostprocessorCLI:

    def __init__(self):
        self._parser = ArgumentParser()
        self._subparsers = self._parser.add_subparsers(dest="subparser")

        self._parser.add_argument("--input", required=True)

        self._subparsers_handles = {}
        self._subparsers_handles["plot"] = self._subparsers.add_parser("plot")
        self._subparsers_handles["plot"].add_argument("quantities", nargs="+")

        self._subparsers_handles["multiplot"] = self._subparsers.add_parser(
            "multiplot")
        self._subparsers_handles["multiplot"].add_argument("--labels",
                                                           nargs="+")
        self._subparsers_handles["multiplot"].add_argument("plot")

        self._parser.add_argument("--show", default=False, action="store_true")
        self._parser.add_argument("--save")

        self._args = self._parser.parse_args()

    def _define_mappings(self, postprocessor):

        self._mappings = {
            "pose": postprocessor.plot_pose,
            "position": postprocessor.plot_positions,
            "rotation": postprocessor.plot_rotations,
            "velocity": postprocessor.plot_velocity,
            "acceleration": postprocessor.plot_acceleration,
            "commands": postprocessor.plot_commands,
        }

    def spin(self):

        if self._args.subparser == "plot":
            postprocessor = PostprocessorHDF5(self._args.input)
            self._define_mappings(postprocessor)

            for quantity in self._args.quantities:
                if quantity in self._mappings.keys():
                    self._mappings[quantity]()

            if self._args.show:
                postprocessor.show()

            if self._args.save:
                postprocessor.save_figures(self._args.save)
                postprocessor.save_report(self._args.save)
                postprocessor.save_csv(self._args.save)

        if self._args.subparser == "multiplot":
            multipostprocessor = MultipostprocessorHDF5(self._args.input)

            multipostprocessor.plot(self._args.labels if self._args.labels else
                                    ())


def main():
    import logging
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)

    cli = PostprocessorCLI()
    cli.spin()


if __name__ == "__main__":
    main()
