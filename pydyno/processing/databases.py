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

import logging

from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple, Union

import matplotlib.pyplot as plt
import matplotlib.tri as mtri
import numpy as np
import pandas as pd
import seaborn as sns
import tables

from collections.abc import Iterable

import pydyno.tooling

from pydyno.processing.core import Processor
from pydyno.processing.filters import TimeSeriesProcessor


class HierarchicalDataProcessor(Processor):

    _METADATA_ROOT_PATH: str = "/metadata/"
    _METADATA_FIELD_SUCCESS: str = "success"

    def __init__(self):

        pass


class UniHierarchicalDataProcessor(HierarchicalDataProcessor):

    def __init__(self, path: Path):

        self._database = self.__class__._load_database(path)

    @staticmethod
    def _load_database(path: Path):

        return tables.open_file(
            path, mode="r", driver="H5FD_CORE", driver_core_backing_store=False
        )

    def get_timeseries_filtered(self, label: str):

        return TimeSeriesProcessor.apply(
            self._get_timeseries(label), self.__class__._get_filter_name(label)
        )

    def _get_timeseries(self, label: str):

        return pydyno.tooling.deep_getattr(
            self._database.root.data, self.__class__._get_function_name(label)
        )

    def get_metadata(self, label: str):

        return list(self._database.get_node("/metadata/" + label))[0]

    def close(self):

        self._database.close()


class MultiHierarchicalDataProcessor(HierarchicalDataProcessor):

    _HUE_PALETTE: dict = {True: (0, 0.3, 0, 0.05), False: (1, 0, 0, 0.01)}

    def __init__(self, path: Union[str, Path]):

        pydyno.tooling.suppress_warnings()
        pydyno.tooling.configure_plotter()

        self._databases = self.__class__.load_databases(path)

    @staticmethod
    def load_databases(path: Path):
        databases = {}

        logging.info('Loading databases from "%s" ...', path.as_posix())

        hdf5_files = list(path.glob("**/*.h5"))
        if len(hdf5_files) < 1:
            logging.error("No databases found under '%s'", path.as_posix())
            raise FileExistsError("")
        for file in hdf5_files:

            logging.info(
                "Found database '%s!'",
                file.parent.as_posix().rsplit("/")[-1] + "/" + file.name,
            )
            hdf = tables.open_file(file)
            databases[Processor._get_data_file_key(file)] = hdf
        return databases

    def is_valid(self, database):

        success = self.get_metadata(database, self._METADATA_FIELD_SUCCESS)
        if not success:
            logging.info("Skipping failed run: %s", bool(success))
            return False
        return True

    def _ensure_list_arg(x):
        if isinstance(x, Iterable) and not isinstance(x, (str, bytes)):
            return list(x)
        return [x]

    def apply_funcs_by_qty_and_series(
        dataframe: pd.DataFrame,
        qty_func_map: Dict[str, Callable],
        value_col: str = "value",
        database_label: str = "series",
        quantity_label: str = "quantity",
    ) -> pd.DataFrame:
        """
        Apply one function per qty, independently for each series.

        qty_func_map: dict mapping qty -> function
        """
        rows = []

        for series_name, df_series in dataframe.groupby(database_label):

            row = {database_label: series_name}

            for qty, func in qty_func_map.items():
                subset = df_series[df_series[quantity_label] == qty][value_col]

                if subset.empty:
                    raise ValueError
                    result = None
                else:
                    result = func(subset.to_numpy())

                col_name = f"{qty}_{func.__name__}"
                row[col_name] = result

            rows.append(row)

        return pd.DataFrame(rows)

    def get_specific_timeseries_set(
        self, quantities: List[str], metadata: Dict[str, Callable]
    ):
        quantities = self.__class__._ensure_list_arg(quantities)

        dataframes = []

        for quantity in quantities:

            for key, database in self._databases.items():

                add_to_dataframes: bool = True
                for mkey, function in metadata.items():
                    if not function(self.get_metadata(database, mkey)):
                        add_to_dataframes = False
                if not add_to_dataframes:
                    continue

                dataframes.append(
                    pd.DataFrame(
                        {
                            "time": pydyno.tooling.deep_getattr(
                                database.root.data, "time"
                            ),
                            "value": pydyno.tooling.deep_getattr(
                                database.root.data, quantity
                            ),
                            "series": key,
                            "quantity": quantity,
                            **self.get_all_metadata(database),
                        }
                    )
                )
        return pd.concat(dataframes, ignore_index=True)

    def get_metadata_timeseries_as_dataframe(
        self, metadata_entry: str, data_entry, additional_metadata: dict
    ) -> pd.DataFrame:

        dataframes = []

        for key, database in self._databases.items():

            # if not self.check_additional_metadata(
            #    database, additional_metadata
            # ):
            #    continue

            dataframes.append(
                pd.DataFrame(
                    {
                        "time": pydyno.tooling.deep_getattr(
                            database.root.data, "time"
                        ),
                        "value": pydyno.tooling.deep_getattr(
                            database.root.data, data_entry
                        ),
                        "series": key,
                        "quantity": data_entry,
                        **self.get_all_metadata(database),
                    }
                )
            )

            # Check whether the run is valid.
            # if not self.is_valid(database):
            #    data_entries.append(0.0)
            # else:
            #    data_entries.append(
            #        pydyno.tooling.deep_getattr(database.root.data, data_entry)[-1]
            #    )

        return pd.concat(dataframes, ignore_index=True)

    def get_timeseries_set_as_dataframe_mod(
        self, data_entries: List[str]
    ) -> pd.DataFrame:

        dataframes = []

        for key, database in self._databases.items():

            dataframes.append(
                pd.DataFrame(
                    {
                        "time": pydyno.tooling.deep_getattr(
                            database.root.data, "time"
                        ),
                        **{
                            data_entry: pydyno.tooling.deep_getattr(
                                database.root.data, data_entry
                            )
                            for data_entry in data_entries
                        },
                        "series": key,
                        "success": self.is_valid(database),
                    }
                )
            )

        return pd.concat(
            dataframes,
            ignore_index=True,
        )

    def get_timeseries_set_as_dataframe(
        self, data_entries: List[str]
    ) -> pd.DataFrame:

        dataframes = []

        for entry in data_entries:
            dataframes.append(self.get_timeseries_as_dataframe(entry))

        return pd.concat(
            dataframes,
            ignore_index=True,
        )

    def get_timeseries_as_dataframe(self, data_entry: str) -> pd.DataFrame:

        dataframes = []

        for key, database in self._databases.items():

            dataframes.append(
                pd.DataFrame(
                    {
                        "time": pydyno.tooling.deep_getattr(
                            database.root.data, "time"
                        ),
                        "value": pydyno.tooling.deep_getattr(
                            database.root.data, data_entry
                        ),
                        "series": key,
                        "quantity": data_entry,
                        "success": self.is_valid(database),
                    }
                )
            )

        return pd.concat(dataframes, ignore_index=True)

    def filter_by_condition(self, df, main_field, other_conditions):
        filtered = df
        for col, val in other_conditions.items():

            filtered = filtered[np.isclose(filtered[col], val, rtol=0.05)]
        return filtered

    def trim_dataframe(self, df, column, value_range):
        """
        Trim DataFrame `df` based on a column and a range (low, high).

        Parameters
        ----------
        df : pd.DataFrame
            The DataFrame to trim.
        column : str
            The column name to apply the filter.
        value_range : tuple(float|None, float|None)
            (low, high). Use None to skip that side of the trim.

        Returns
        -------
        pd.DataFrame
            Trimmed DataFrame.
        """
        low, high = value_range

        if low is not None and high is not None:
            return df[df[column].between(low, high)]
        elif low is not None:
            return df[df[column] >= low]
        elif high is not None:
            return df[df[column] <= high]
        else:
            return df.copy()  # no trimming

    @pydyno.tooling.save_figure
    @pydyno.tooling.show_figure
    @pydyno.tooling.trim_x_axis
    @pydyno.tooling.with_axes
    def plot_response(
        self,
        metadata_field,
        data_entry,
        additional_metadata,
        range=(None, None),
    ):

        dataframe = self.get_metadata_timeseries_as_dataframe(
            metadata_field, data_entry, additional_metadata
        )

        subset = self.filter_by_condition(
            dataframe, metadata_field, additional_metadata
        )

        for metadata in subset[metadata_field].unique():
            subset2 = subset[subset[metadata_field] == metadata]
            subset2 = self.trim_dataframe(subset2, "time", range)
            plt.plot(
                subset2["time"], subset2["value"], label=f"{metadata:0.2f}"
            )

        plt.legend(
            ncol=3,
            fontsize=12,
            title_fontsize=12,
            handlelength=1,
            handletextpad=0.5,
            loc="upper right",
            title=pydyno.tooling.convert_camelcase_to_title(metadata_field),
        )

        plt.show()

    def plot_pair(self, df):

        sns.pairplot(
            df,
            x_vars=["input1", "input2", "input3"],
            y_vars=["qoi1", "qoi2"],
            kind="reg",  # regression line
            height=3,
            aspect=1,
        )
        plt.suptitle("Input vs QoI with Regression", y=1.02)
        plt.show()

    def plot_responses_offset(self, metadata_field, data_entry):

        dataframe = self.get_metadata_timeseries_as_dataframe(
            metadata_field, data_entry
        )

        fig = plt.figure(figsize=(8, 6))
        axes = fig.add_subplot(111, projection="3d")

        # Plot lines for each category
        for metadata in dataframe["meta"].unique():
            subset = dataframe[dataframe["meta"] == metadata]
            axes.plot(
                subset["time"],
                subset["value"],
                zs=subset["meta"],
                zdir="y",
                marker="o",
                label=f"{metadata}",
                lw=0.5,
            )
        plt.show()

    @pydyno.tooling.save_figure
    @pydyno.tooling.show_figure
    @pydyno.tooling.trim_x_axis
    @pydyno.tooling.with_axes
    def plot_timeseries_heatmap(
        self,
        variable: str,
        metadata_entries: Dict[str, float],
        axes=None,
        sort_order: List[str] = None,
        **kwargs,
    ):

        dataframe = self.get_specific_timeseries_set(variable, metadata_entries)

        dataframe["time"] = pd.to_numeric(dataframe["time"], errors="coerce")

        dataframe["label"] = dataframe[metadata_entries.keys()].apply(
            lambda row: ", ".join(
                f"{col}={x:.1f}" if isinstance(x, float) else f"{col}={x}"
                for col, x in zip(metadata_entries.keys(), row)
            ),
            axis=1,
        )

        if sort_order is not None:
            # Sort by location, then sensor_type
            dataframe = dataframe.sort_values(sort_order, ascending=True)

        df_agg = dataframe.groupby(
            ["series", "time", "label"], as_index=False, sort=False
        )["value"].mean()

        # Create new row labels

        dataframe_pivot = df_agg.pivot_table(
            index="label",
            columns="time",
            values="value",
            aggfunc="mean",
        )

        if sort_order is not None:
            sorted_labels = dataframe.sort_values(sort_order)["label"].unique()
            dataframe_pivot = dataframe_pivot.reindex(sorted_labels)

        mask = dataframe_pivot.isna()

        dataframe_pivot = dataframe_pivot.fillna(
            np.nanmin(dataframe_pivot.values)
        )

        sns.heatmap(
            dataframe_pivot,
            cmap="viridis",
            annot=False,
            cbar=True,
            ax=axes,
            # mask=mask,
        )
        # sns.clustermap(dataframe_pivot,cmap="viridis",row_cluster=True,col_cluster=False,ax=axes, cbar=True)

        tick_step = 5.0
        columns = dataframe_pivot.columns.values
        tick_values = np.arange(
            columns.min(), columns.max() + tick_step, tick_step
        )

        # Find nearest column indices for each tick value
        tick_positions = [np.argmin(np.abs(columns - t)) for t in tick_values]
        tick_labels = [f"{columns[pos]:.1f}" for pos in tick_positions]

        plt.grid(False, which="both")
        axes.set_xticks(tick_positions)
        axes.set_xticklabels(tick_labels, rotation=45, ha="right")
        plt.tight_layout()

    @pydyno.tooling.save_figure
    @pydyno.tooling.show_figure
    @pydyno.tooling.trim_x_axis
    @pydyno.tooling.with_axes
    def plot_mean(
        self,
        data_entry: str,
        range: Tuple[Optional[float], Optional[float]] = (None, None),
        errorbar="sd",
        axes: Optional[plt.Axes] = None,
    ):

        dataframe = self.get_timeseries_as_dataframe(data_entry)
        dataframe = self.trim_dataframe(dataframe, "time", range)

        sns.lineplot(
            data=dataframe,
            x="time",
            y="value",
            units="series",
            estimator=None,
            hue="success",
            palette=self._HUE_PALETTE,
            ax=axes,
        )

        sns.lineplot(
            data=dataframe[dataframe["success"] == True],
            x="time",
            y="value",
            estimator="mean",
            errorbar="sd",
            ax=axes,
        )

    def interpolate_along_position_with_time(
        df: pd.DataFrame,
        series_col: str = "series",
        position_col: str = "position",  # independent variable for interpolation
        time_col: str = "time",  # metadata to interpolate alongside
        value_cols: list[str] = None,  # dependent variables (e.g., speed)
        n_points: int = 50,
        fill_edges: bool = False,
    ) -> pd.DataFrame:
        """
        Interpolates dependent variables along position, preserving time metadata.

        Parameters
        ----------
        df : pd.DataFrame
            Must contain [series_col, position_col, time_col, value_cols].
        series_col : str
            Column identifying each series.
        position_col : str
            Column to interpolate along (independent variable).
        time_col : str
            Metadata column to interpolate alongside.
        value_cols : list[str] | None
            Columns to interpolate. If None, all numeric columns except series_col, position_col, time_col will be used.
        n_points : int
            Number of points for the uniform position grid per series.
        fill_edges : bool
            If True, forward/backward fill edges to remove NaNs at boundaries.

        Returns
        -------
        pd.DataFrame
            Interpolated DataFrame with columns [series_col, position_col, time_col, value_cols, ...].
        """
        df = df.copy()

        # Ensure numeric columns
        df[position_col] = pd.to_numeric(df[position_col], errors="coerce")
        df[time_col] = pd.to_numeric(df[time_col], errors="coerce")

        if value_cols is None:
            value_cols = df.select_dtypes(include=[np.number]).columns.tolist()
            value_cols = [
                c
                for c in value_cols
                if c not in [series_col, position_col, time_col]
            ]

        df[value_cols] = df[value_cols].apply(pd.to_numeric, errors="coerce")

        dfs_interp = []

        for series_name, group in df.groupby(series_col):
            group = group.sort_values(position_col)
            pos_min, pos_max = (
                group[position_col].min(),
                group[position_col].max(),
            )
            if pos_min == pos_max:
                dfs_interp.append(group)
                continue

            pos_grid = np.linspace(pos_min, pos_max, n_points)

            df_interp = pd.DataFrame(
                {position_col: pos_grid, series_col: series_name}
            )
            from scipy.interpolate import interp1d

            # Interpolate dependent variables
            for col in value_cols + [time_col]:
                f = interp1d(
                    group[position_col],
                    group[col],
                    kind="linear",
                    bounds_error=False,
                    fill_value="extrapolate" if fill_edges else np.nan,
                )
                df_interp[col] = f(pos_grid)

            dfs_interp.append(df_interp)

        return pd.concat(dfs_interp, ignore_index=True)

    def plot_mean_std(
        df: pd.DataFrame,
        x: str,
        y: str,
        window: float = 0.01,
        n_points: int = 200,
        color: str = "black",
        shade_color: str = "gray",
        shade_alpha: float = 0.3,
        label_mean: str = "Mean",
        label_std: str = "±1 std",
        ax=None,
    ):
        """
        Plots mean and ±1 std of a column y across multiple series in a DataFrame,
        aligned on a common x grid.

        Parameters:
        -----------
        df : pd.DataFrame
            Data containing columns for x, y, and multiple series.
        x : str
            Name of the x-axis column (e.g., position or time).
        y : str
            Name of the y-axis column (e.g., velocity).
        window : float
            Tolerance to select points near each x grid value.
        n_points : int
            Number of points in the interpolated x grid.
        color : str
            Color of the mean line.
        shade_color : str
            Color of the ±1 std shading.
        shade_alpha : float
            Transparency of the shading.
        label_mean : str
            Label for mean line.
        label_std : str
            Label for shaded std.
        ax : matplotlib.axes.Axes, optional
            Axes to plot on. Creates a new one if None.
        """

        if ax is None:
            fig, ax = plt.subplots()

        # Create uniform x grid
        x_grid = np.linspace(df[x].min(), df[x].max(), n_points)

        mean_list = []
        std_list = []

        for xi in x_grid:
            vals = df.loc[(df[x] >= xi - window) & (df[x] <= xi + window), y]
            mean_list.append(vals.mean())
            std_list.append(vals.std())

        stats_df = pd.DataFrame({x: x_grid, "mean": mean_list, "std": std_list})

        # Plot ±1 std shading
        ax.fill_between(
            stats_df[x],
            stats_df["mean"] - stats_df["std"],
            stats_df["mean"] + stats_df["std"],
            color=shade_color,
            alpha=shade_alpha,
            label=label_std,
        )

        # Plot mean line
        sns.lineplot(
            data=stats_df,
            x=x,
            y="mean",
            color=color,
            linewidth=2,
            label=label_mean,
            ax=ax,
        )

        return ax

    @pydyno.tooling.save_figure
    @pydyno.tooling.show_figure
    @pydyno.tooling.trim_x_axis
    @pydyno.tooling.with_axes
    def plot_mean_nontime(
        self,
        data_entry: str,
        data_entry2: str,
        range: Tuple[Optional[float], Optional[float]] = (None, None),
        errorbar="sd",
        axes: Optional[plt.Axes] = None,
    ):

        dataframe = self.get_timeseries_set_as_dataframe_mod(
            [data_entry, data_entry2]
        )
        dataframe = self.trim_dataframe(dataframe, data_entry, range)
        print(dataframe)

        df_interp = self.__class__.interpolate_along_position_with_time(
            dataframe,
            position_col=data_entry,
            value_cols=[data_entry2],
            fill_edges=False,
            n_points=5000,
        )
        sns.lineplot(
            data=df_interp,
            x=data_entry,
            y=data_entry2,
            units="series",
            estimator=None,
            hue="series",
            # palette=self._HUE_PALETTE,
            alpha=0.1,
            ax=axes,
        )

        # mean_df = df_interp.groupby(data_entry)[data_entry2].mean().reset_index()

        self.__class__.plot_mean_std(df_interp, data_entry, data_entry2)

    @staticmethod
    def rename_dataframe_columns(
        dataframe: pd.DataFrame, labels: List[str]
    ) -> pd.DataFrame:

        return dataframe.rename(columns=dict(zip(dataframe.columns, labels)))

    @staticmethod
    def rename_dataframe_rows(
        dataframe: pd.DataFrame, labels: List[str]
    ) -> pd.DataFrame:

        dataframe.index = labels
        return dataframe

    # Default variable name sanitizer
    # def _sanitize_filtered_quantity_name(quantity_to_function_map:Dict):
    #
    #    return f"{key.replace('_', ' ').replace('.', ' ').capitalize()} {value.__name__.replace('_', '').capitalize()}"

    @pydyno.tooling.save_figure
    @pydyno.tooling.show_figure
    @pydyno.tooling.trim_x_axis
    @pydyno.tooling.with_axes
    def plot_correlation_heatmap(
        self,
        variables: List[str],
        functions: Dict[str, Callable],
        labels: Optional[List[str]] = None,
        lower_triangular_only: bool = True,
        axes: Optional[plt.Axes] = False,
        hide_diagonal: bool = False,
        **kwargs,
    ) -> plt.Axes:

        dataframe = self.get_timeseries_set_as_dataframe(variables)

        # Apply the specified preprocessing functions.
        processed_dataframe = self.__class__.apply_funcs_by_qty_and_series(
            dataframe, functions
        )

        correlation_dataframe = processed_dataframe[
            [f"{key}_{value.__name__}" for key, value in functions.items()]
        ].corr()

        # Compute a mask to cover the upper triangular region of the correlation
        # matrix.
        mask = np.zeros_like(correlation_dataframe, dtype=bool)
        if lower_triangular_only:
            mask = np.triu(np.invert(mask), k=1)
        if hide_diagonal:
            np.fill_diagonal(mask, 1)

        # Rename the DataFrame labels before plotting.
        if labels is not None:
            correlation_dataframe = self.__class__.rename_dataframe_rows(
                correlation_dataframe, labels
            )
            correlation_dataframe = self.__class__.rename_dataframe_columns(
                correlation_dataframe, labels
            )

        # Plot the correlation heatmap.
        sns.heatmap(
            correlation_dataframe,
            mask=mask,
            annot=True,
            cmap="viridis",
            center=0,
            vmin=-1,
            vmax=1,
            ax=axes,
            robust=True,
        )

        # Disable the major and minor grids to avoid clutter in the correlation
        # plot.
        axes.grid(False, which="both")
        axes.xaxis.label.set_visible(False)
        axes.set_xlabel("")

    def plot_response_surface(self, metadata_field):

        pass

    def plot_surface(
        self, metadata_field_x: str, metadata_field_y: str, data_entry: str
    ):

        figure, axes = plt.subplots(subplot_kw={"projection": "3d"})

        metadata_entries_x: List = []
        metadata_entries_y: List = []
        data_entries: List = []

        for _, database in self._databases.items():
            metadata_entries_x.append(
                self.get_metadata(database, metadata_field_x)
            )
            metadata_entries_y.append(
                self.get_metadata(database, metadata_field_y)
            )
            # Check whether the run is valid.
            if not self.is_valid(database):
                data_entries.append(0.0)
            else:
                data_entries.append(
                    pydyno.tooling.deep_getattr(database.root.data, data_entry)[
                        -1
                    ]
                )

        Xx = np.array(metadata_entries_x)
        Yy = np.array(metadata_entries_y)
        Zz = np.array(data_entries)

        surf = axes.plot_trisurf(
            Xx,
            Yy,
            self.__class__._smoothen_response_surface(Xx, Yy, Zz, iterations=2),
            antialiased=True,
            cmap="viridis",
        )

    @staticmethod
    def _smoothen_response_surface(x, y, z, iterations=5):
        triangulation = mtri.Triangulation(x, y)
        z_new = z.copy()
        for _ in range(iterations):
            z_temp = z_new.copy()
            for i in range(len(z)):
                mask = np.any(triangulation.triangles == i, axis=1)
                neighbors = np.unique(triangulation.triangles[mask])
                z_new[i] = z_temp[neighbors].mean()
        return z_new

    def get_metadata(self, database: tables.File, field: str):

        return list(getattr(database.root.metadata, field))[0]

    def get_all_metadata(self, database):

        data_dict = {
            node._v_name: self.__class__.safe_read_node(
                node
            )  # read() for Table or Array
            for node in database.root.metadata._v_children.values()
            if node._v_name != "gates"
        }

        return data_dict

    def show(self):
        plt.show()

    @staticmethod
    def safe_read_node(node):
        if isinstance(node, tables.Table):
            # Structured array → convert each row to dict
            arr = node.read()
            return [
                {
                    name: (
                        row[name].item()
                        if np.isscalar(row[name])
                        else row[name]
                    )
                    for name in arr.dtype.names
                }
                for row in arr
            ]
        else:
            # For Array / CArray / VLArray
            arr = node.read()
            # Convert scalars to native Python types if array has length 1
            if arr.shape == ():  # scalar
                return arr.item()
            elif arr.size == 1:
                return arr.item()
            else:
                # Convert elements to Python types if they are scalars
                if np.issubdtype(arr.dtype, np.number) or np.issubdtype(
                    arr.dtype, np.bool_
                ):
                    return arr.astype(arr.dtype.type)
                return arr
