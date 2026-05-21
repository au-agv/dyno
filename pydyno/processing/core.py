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
#  Copyright (c) 2024 Dario Sirangelo (dev@dariosirangelo.me).
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

from pathlib import Path

class Processor:

    def __init__(self):

        pass

    @staticmethod
    def _get_data_file_key(path: Path, key_method: str = "from_indexed_folder"):
        """
        Generates a key for a data file based on the specified method.

        Args:
            path: The file path for which to generate the key.

            key_method: The method to use for generating the key. Options are:
                - "from_indexed_folder": Uses the file name and folder suffix.
                - "from_filename": Uses only the file name.

        Returns:
            A string representing the generated key.

        Raises:
            ValueError: If an unexpected key method is provided.
        """
        if key_method == "from_indexed_folder":
            return f"{path.stem}_{path.parent.suffix[1:]}"
        elif key_method == "from_filename":
            return path.stem
        else:
            raise ValueError(
                f"Unexpected database key assignment method: {key_method}"
            )

    @staticmethod
    def _get_function_name(string):
        """
        Returns the part of the string before the first colon.

        Args:
            string (str): The input string to process.

        Returns:
            (str) The substring before the first colon.
        """
        return string.split(":", 1)[0]

    @staticmethod
    def _get_filter_name(string: str) -> str:
        """
        Returns everything after the first colon in a string.

        If no colon is present, returns "same".

        Args:
            string (str): The input string to process.

        Returns:
            (str) The substring after the first colon, or "same" if no colon is found.
        """
        parts = string.split(":", 1)  # split at the first colon only
        if len(parts) == 2:
            return parts[1].split(":")  # split the rest by colon
        else:
            return ["same"]