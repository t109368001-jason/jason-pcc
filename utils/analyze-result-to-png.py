import pathlib
import re

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

fig_size = (10, 10)
title_fontsize = 24
axis_fontsize = 20
legend_fontsize = 16

skip_exists = True

savefig_kwargs = dict(bbox_inches='tight')

folder = pathlib.Path("../../result/jason-pcc/")

file_info_pattern = re.compile(
    r'(?P<filepath>.*analyzer([^/^\\]*)[/\\]*\[(?P<frequency>[\d.]*)]\[(?P<resolution>[\d.]*)][/\\]*'
    r'(?P<title>(?P<x_label>[a-zA-Z]*)To(?P<y_label>[a-zA-Z]*))(?:\[(?P<other_parameters>[\d.[\]]*)])*\.csv)')

plot_config_dict = {
    "VoxelPointCountToVoxelCount": {},
    "VoxelOccupancyCountToVoxelCount": {},
    "VoxelPointNormalAngleSTDToVoxelCount": {
        "override": lambda file_info: plot_voxel_point_normal_angle_std_to_voxel_count(file_info),
    },
    "VoxelOccupancyIntervalSTDToVoxelCount": {
    },
    "VoxelOccludedPercentageToVoxelCount": {
        "xscale": 'linear',
        "xlim": (0, 100),
    },
    "VoxelOccupancyChangeCountToVoxelCount": {},
    "VoxelIntensitySTDToVoxelCount": {
        "xscale": 'linear',
        "xlim": (0, 127),
    },
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
    axes.set_xscale('log')
    axes.set_yscale('log')


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
    axes.plot(x, background, label="Background")
    axes.plot(x, dynamic, label="Dynamic")
    axes.plot(x, other, label="Other")
    axes.legend(loc="upper right", fontsize=legend_fontsize)


def plot(file_info, plot_config):
    if "override" in plot_config:
        plot_config["override"](file_info)
    png_path = pathlib.Path(file_info["filepath"]).with_suffix(".png")
    if png_path.exists() and skip_exists:
        return

    csv = pd.read_csv(file_info["filepath"])

    fig, axes = build_figure(file_info)

    if "xscale" in plot_config:
        axes.set_xscale(plot_config["xscale"])
    if "xlim" in plot_config:
        axes.set_xlim(plot_config["xlim"])

    plot_lines(axes=axes,
               x=csv.iloc[:, 0],
               background=csv.iloc[:, 1],
               dynamic=csv.iloc[:, 2],
               other=csv.iloc[:, 3])

    print(f"write {png_path}")
    fig.savefig(png_path, **savefig_kwargs)


def plot_voxel_point_normal_angle_std_to_voxel_count(file_info):
    png_path = pathlib.Path(file_info["filepath"]).with_suffix(".png")
    if png_path.exists() and skip_exists:
        return

    csv = pd.read_csv(file_info["filepath"], skiprows=2, header=None)

    fig, (axes_azimuth, axes_zenith) = build_figure(file_info, 2, 1)

    axes_azimuth.set_title(format_title(file_info["title"]).replace("Angle", "Azimuth"), fontsize=title_fontsize)
    axes_azimuth.set_xlabel(format_label(file_info["x_label"]).replace("Angle", "Azimuth"), fontsize=axis_fontsize)
    axes_azimuth.set_xscale('linear')
    axes_azimuth.set_xlim(0, 180)

    axes_zenith.set_title(format_title(file_info["title"]).replace("Angle", "Zenith"), fontsize=title_fontsize)
    axes_zenith.set_xlabel(format_label(file_info["x_label"]).replace("Angle", "Zenith"), fontsize=axis_fontsize)
    axes_zenith.set_xscale('linear')
    axes_zenith.set_xlim(0, 90)

    plot_lines(axes=axes_azimuth,
               x=csv.iloc[:, 0],
               background=csv.iloc[:, 1],
               dynamic=csv.iloc[:, 2],
               other=csv.iloc[:, 3])

    plot_lines(axes=axes_zenith,
               x=csv.iloc[:, 0],
               background=csv.iloc[:, 4],
               dynamic=csv.iloc[:, 5],
               other=csv.iloc[:, 6])

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
