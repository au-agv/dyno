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

import functools

import numpy as np
import scipy.signal


class TimeSeriesProcessor:

    @staticmethod
    def get_caret_args(s: str) -> list:
        parts = s.split("^", 1)
        if len(parts) == 1:
            return []
        raw_args = parts[1].split("^")

        # Try to convert each arg to int or float if possible
        converted_args = []
        for arg in raw_args:
            try:
                if "." in arg:
                    converted_args.append(float(arg))
                else:
                    converted_args.append(int(arg))
            except ValueError:
                converted_args.append(arg)  # leave as string if not numeric
        return converted_args

    @classmethod
    def apply(cls, x: np.typing.NDArray, function: str | list[str]):
        """
        Apply one or more class methods sequentially to x.

        Each function string can have optional arguments after '^'.

        Args:
            x: The input data to process.

            function: A single function name as a string or a list of function
                      names.

        Returns:
            The result of applying the functions sequentially to x.

        Raises:
            AttributeError: If a specified method does not exist in the class.

            TypeError: If the function arguments are not compatible with the
            method.
        """

        def apply_single(value: float, method_signature: str):
            """
            Applies a single function to the input value.

            Args:
                value: The current value to process.

                method_signature: The function string specifying the method and
                                   its arguments.

            Returns:
                The result of applying the function to val.

            Raises:
                AttributeError: If the specified method does not exist in the
                                class.

                TypeError: If the arguments are not compatible with the method.
            """
            method_name = method_signature.split("^", 1)[0]
            args = cls.get_caret_args(method_signature)

            method = getattr(cls, f"_{method_name}")

            return method(value, *args)

        functions = [function] if isinstance(function, str) else function

        return functools.reduce(apply_single, functions, x)

    @staticmethod
    def _same(x: np.typing.NDArray) -> np.typing.NDArray:
        """
        Returns the input array unchanged.

        Args:
            x: The input array.

        Returns:
            The same array passed as input.
        """
        return x

    @staticmethod
    def _butter(
        x: np.typing.NDArray,
        cutoff_frequency: float = 0.1,
        samping_frequency: float = 1.0,
        order: int = 4,
        type: str = "low",
    ) -> np.typing.NDArray:
        """
        Apply a Butterworth filter to a 1D array.

        Args:
            x: Input array (1D).

            cutoff: Cutoff frequency in Hz (or normalized to [0 – 0.5] if fs=1).

            fs: Sampling frequency.

            order: Filter order.

            btype: Filter type ('low', 'high', 'bandpass', 'bandstop').

        Returns:
            Filtered array of the same shape as x.

        Raises:
            ValueError: If the input array is not 1D.

            ValueError: If the filter type is not valid.
        """

        # Normalize the cutoff frequency to Nyquist frequency.
        if isinstance(cutoff_frequency, (int, float)):
            cutoff_frequency_normalized = cutoff_frequency / (
                0.5 * samping_frequency
            )
        else:
            cutoff_frequency_normalized = np.array(cutoff_frequency) / (
                0.5 * samping_frequency
            )

        # Compute the Butterworth filter coefficients
        butterworth_coeff_b, butterworth_coeff_a = scipy.signal.butter(
            order, cutoff_frequency_normalized, btype=type, analog=False
        )

        # Apply the zero-phase filter to the array.
        return scipy.signal.filtfilt(
            butterworth_coeff_b, butterworth_coeff_a, x
        )

    @staticmethod
    def _start(x: np.typing.NDArray) -> float:
        """
        Returns the first element of the array.

        Args:
            x: The input array from which to retrieve the first element.

        Returns:
            The first element of the array.
        """
        return x[0]

    @staticmethod
    def _end(x: np.typing.NDArray) -> float:
        """
        Returns the last element of the array.

        Args:
            x: The input array from which to retrieve the last element.

        Returns:
            The last element of the array.
        """
        return x[-1]

    @staticmethod
    def _min(x: np.typing.NDArray) -> float:
        """
        Computes the minimum value in an array.

        Args:
            x: The input array for which to compute the minimum value.

        Returns:
            The minimum value in the array.
        """
        return np.min(x)

    @staticmethod
    def _absmin(x: np.typing.NDArray) -> float:
        """
        Computes the minimum absolute value in an array.

        Args:
            x: The input array for which to compute the minimum absolute value.

        Returns:
            The minimum absolute value in the array.
        """
        return np.min(np.abs(x))

    @staticmethod
    def _max(x: np.typing.NDArray) -> float:
        """
        Computes the maximum value in an array.

        Args:
            x: The input array for which to compute the maximum value.

        Returns:
            The maximum value in the array.
        """
        return np.max(x)

    @staticmethod
    def _absmax(x: np.typing.NDArray) -> float:
        """
        Computes the maximum absolute value in an array.

        Args:
            x: The input array for which to compute the maximum absolute value.

        Returns:
            The maximum absolute value in the array.
        """
        return np.max(np.abs(x))

    @staticmethod
    def _mean(x: np.typing.NDArray) -> float:
        """
        Computes the mean of an array.

        Args:
            x: The input array for which to compute the mean.

        Returns:
            The mean value of the array.
        """
        return np.mean(x)

    @staticmethod
    def _median(x):
        """
        Computes the median of an array.

        Args:
            x: The input array for which to compute the median.

        Returns:
            The median value of the array.
        """
        return np.median(x)

    @staticmethod
    def _std(x: np.typing.NDArray) -> float:
        """
        Computes the standard deviation of an array.

        Args:
            x (float): The input array for which to compute the standard
                       deviation.

        Returns:
            (float): The standard deviation of the array.
        """
        return np.std(x)
