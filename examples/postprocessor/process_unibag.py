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

from pydyno.processing.bags import UniBagDataProcessor

from pathlib import Path

@pydyno.tooling.configure_logger(level="info")
def main():

    processor = UniBagDataProcessor(
        pydyno.tooling.get_script_directory(__file__).parents[0]
        / "postprocessor/",
        Path.cwd() / "myh5database.h5"
    )

    #processor._list_topics(
    #    processor._get_connections(processor._get_bag("1768519073-bag-11_0_"))
    #)
    #processor.get_specific_timeseries_set(
    #    ["/clock"], {"clock.sec": lambda x: x < 1.0}
    #)


if __name__ == "__main__":
    main()
