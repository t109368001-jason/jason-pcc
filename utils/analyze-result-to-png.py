import pathlib
import re

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

fig_size = (10, 10)
title_fontsize = 24
axis_fontsize = 20
legend_fontsize = 16

x_index_beg = 1
entropy_x_multiplier = 0.1

skip_other = False
skip_exists = True

savefig_kwargs = dict(bbox_inches='tight')

folder = pathlib.Path("../../result/jason-pcc/")

file_info_pattern = re.compile(
    r'(?P<filepath>.*analyzer([^/^\\]*)[/\\]*\[(?P<frequency>[\d.]*)]\[(?P<resolution>[\d.]*)][/\\]*'
    r'(?P<title>(?P<x_label>[a-zA-Z]*)To(?P<y_label>[a-zA-Z]*))(?:\[(?P<other_parameters>[\d.[\]]*)])*\.csv)')

plot_config_dict = {
    "VoxelPointCountToVoxelCount": {
        "yscale": 'log',
        "xscale": 'log',
    },
    "VoxelOccupancyCountToVoxelCount": {
        "yscale": 'log',
    },
    "VoxelPointNormalAngleSTDToVoxelCount": {
        "override": lambda file_info: plot_voxel_point_normal_angle_std_to_voxel_count(file_info),
    },
    "VoxelOccupancyIntervalSTDToVoxelCount": {
        "yscale": 'log',
    },
    "VoxelOccludedPercentageToVoxelCount": {
        "x_max": 100,
    },
    "VoxelOccupancyChangeCountToVoxelCount": {
        "yscale": 'log',
    },
    "VoxelIntensitySTDToVoxelCount": {
        "yscale": 'log',
        "x_max": 127,
    },
    "VoxelPointNormalAngleEntropyToVoxelCount": {
        "override": lambda file_info: plot_voxel_point_normal_angle_entropy_to_voxel_count(file_info),
    },
    "VoxelOccupancyIntervalEntropyToVoxelCount": {
        "x_multiplier": entropy_x_multiplier
    },
    "VoxelIntensityEntropyToVoxelCount": {
        "x_multiplier": entropy_x_multiplier
    }
}


def to_spaced_camelcase(text):
    return re.sub(r'((?<=[a-z])[A-Z]|(?<!\A)[A-Z](?=[a-z]))', r' \1', text)


def format_title(title):
    return to_spaced_camelcase(title).replace("To", "-")


def format_label(label):
    return to_spaced_camelcase(label)


def set_default_axes(axes: plt.Axes, file_info):
    axes.set_title(format_title(file_info["title"]), fontsize=title_fontsize)
    axes.set_xlabel(format_label(file_info["x_label"]), fontsize=axis_fontsize)
    axes.set_ylabel(format_label(file_info["y_label"]), fontsize=axis_fontsize)


def build_figure(file_info, rows=1, cols=1):
    fig, axes = plt.subplots(rows, cols, figsize=(fig_size[0] * cols, fig_size[1] * rows))
    fig: plt.Figure
    fig.suptitle(f'frequency={float(file_info["frequency"]):g},resolution={float(file_info["resolution"]):g}')
    if isinstance(axes, np.ndarray):
        for a in axes:
            set_default_axes(a, file_info)
    else:
        set_default_axes(axes, file_info)
    return fig, axes


def plot_lines(axes, x, background, dynamic, other):
    if not skip_other:
        axes.plot(x, other, 'g', label="Other", alpha=0.8)
    axes.plot(x, background, 'b', label="Background", alpha=0.8)
    axes.plot(x, dynamic, 'r', label="Dynamic", alpha=0.8)
    axes.legend(loc="upper right", fontsize=legend_fontsize)


def plot(file_info, plot_config):
    if "override" in plot_config:
        plot_config["override"](file_info)
        return
    png_path = pathlib.Path(file_info["filepath"]).with_suffix(".png")
    if png_path.exists() and skip_exists:
        return

    csv = pd.read_csv(file_info["filepath"])
    csv.replace(0, np.nan, inplace=True)

    fig, axes = build_figure(file_info)

    if "x_multiplier" in plot_config:
        csv.iloc[:, 0] = csv.iloc[:, 0] * plot_config["x_multiplier"]

    plot_lines(axes=axes,
               x=csv.iloc[x_index_beg:, 0],
               background=csv.iloc[x_index_beg:, 1],
               dynamic=csv.iloc[x_index_beg:, 2],
               other=csv.iloc[x_index_beg:, 3])

    if "xscale" in plot_config:
        axes.set_xscale(plot_config["xscale"])
    if "yscale" in plot_config:
        axes.set_yscale(plot_config["yscale"])
    axes.set_xlim(xmin=plot_config.get("x_min"), xmax=plot_config.get("x_max"))

    print(f"write {png_path}")
    fig.savefig(png_path, **savefig_kwargs)


def plot_voxel_point_normal_angle_std_to_voxel_count(file_info):
    png_path = pathlib.Path(file_info["filepath"]).with_suffix(".png")
    if png_path.exists() and skip_exists:
        return

    csv = pd.read_csv(file_info["filepath"], skiprows=1)
    csv.replace(0, np.nan, inplace=True)

    fig, (axes_azimuth, axes_zenith) = build_figure(file_info, 2, 1)

    axes_azimuth.set_title(format_title(file_info["title"]).replace("Angle", "Azimuth"), fontsize=title_fontsize)
    axes_azimuth.set_xlabel(format_label(file_info["x_label"]).replace("Angle", "Azimuth"), fontsize=axis_fontsize)
    axes_azimuth.set_xlim(0, 180)

    axes_zenith.set_title(format_title(file_info["title"]).replace("Angle", "Zenith"), fontsize=title_fontsize)
    axes_zenith.set_xlabel(format_label(file_info["x_label"]).replace("Angle", "Zenith"), fontsize=axis_fontsize)
    axes_zenith.set_xlim(0, 90)

    plot_lines(axes=axes_azimuth,
               x=csv.iloc[x_index_beg:, 0],
               background=csv.iloc[x_index_beg:, 1],
               dynamic=csv.iloc[x_index_beg:, 2],
               other=csv.iloc[x_index_beg:, 3])

    plot_lines(axes=axes_zenith,
               x=csv.iloc[x_index_beg:, 0],
               background=csv.iloc[x_index_beg:, 4],
               dynamic=csv.iloc[x_index_beg:, 5],
               other=csv.iloc[x_index_beg:, 6])

    print(f"write {png_path}")
    fig.savefig(png_path, **savefig_kwargs)


def plot_voxel_point_normal_angle_entropy_to_voxel_count(file_info):
    png_path = pathlib.Path(file_info["filepath"]).with_suffix(".png")
    if png_path.exists() and skip_exists:
        return

    csv = pd.read_csv(file_info["filepath"], skiprows=1)
    csv.replace(0, np.nan, inplace=True)

    fig, (axes_azimuth, axes_zenith) = build_figure(file_info, 2, 1)

    axes_azimuth.set_title(format_title(file_info["title"]).replace("Angle", "Azimuth"), fontsize=title_fontsize)
    axes_azimuth.set_xlabel(format_label(file_info["x_label"]).replace("Angle", "Azimuth"), fontsize=axis_fontsize)

    axes_zenith.set_title(format_title(file_info["title"]).replace("Angle", "Zenith"), fontsize=title_fontsize)
    axes_zenith.set_xlabel(format_label(file_info["x_label"]).replace("Angle", "Zenith"), fontsize=axis_fontsize)

    csv.iloc[:, 0] = csv.iloc[:, 0] * entropy_x_multiplier

    plot_lines(axes=axes_azimuth,
               x=csv.iloc[x_index_beg:, 0],
               background=csv.iloc[x_index_beg:, 1],
               dynamic=csv.iloc[x_index_beg:, 2],
               other=csv.iloc[x_index_beg:, 3])

    plot_lines(axes=axes_zenith,
               x=csv.iloc[x_index_beg:, 0],
               background=csv.iloc[x_index_beg:, 4],
               dynamic=csv.iloc[x_index_beg:, 5],
               other=csv.iloc[x_index_beg:, 6])

    print(f"write {png_path}")
    fig.savefig(png_path, **savefig_kwargs)


def files_to_file_infos(files):
    file_infos = [file_info_pattern.match(str(file)) for file in files]
    file_infos = [file_info for file_info in file_infos if file_info is not None]
    file_infos = [file_info.groupdict() for file_info in file_infos]
    for file_info in file_infos:
        if "other_parameters" not in file_info:
            continue
        if file_info["other_parameters"] is None:
            file_info["other_parameters"] = []
            continue
        file_info["other_parameters"] = file_info["other_parameters"].split("][")
    return file_infos


def main():
    files = list(folder.rglob("*/*.csv"))

    file_infos = files_to_file_infos(files)

    for file_info in file_infos:
        plot(file_info, plot_config_dict[file_info["title"]])


if __name__ == '__main__':
    main()
