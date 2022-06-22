import pathlib
import re

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

fig_size = (10, 10)
title_fontsize = 24
axis_fontsize = 20
legend_fontsize = 16

folder = pathlib.Path("../../result/jason-pcc/")

file_info_pattern = re.compile(
    r'(?P<filepath>.*analyzer([^/^\\].*)[/\\]*\[(?P<frequency>[\d.]*)]\[(?P<resolution>[\d.]*)][/\\]*'
    r'(?P<title>(?P<x_label>[a-zA-Z]*)To(?P<y_label>[a-zA-Z]*))(?:\[(?P<other_parameters>[\d.[\]]*)])*\.csv)')


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


def plot_voxel_point_count_to_voxel_count(file_info):
    csv = pd.read_csv(file_info["filepath"])

    fig, axes = build_figure(file_info)

    plot_lines(axes=axes,
               x=csv["Voxel Point Count"],
               background=csv["Voxel Count (Background)"],
               dynamic=csv["Voxel Count (Dynamic)"],
               other=csv["Voxel Count (Other)"])

    fig.savefig(pathlib.Path(file_info["filepath"]).with_suffix(".png"))


def plot_voxel_occupancy_count_to_voxel_count(file_info):
    csv = pd.read_csv(file_info["filepath"])

    fig, axes = build_figure(file_info)

    plot_lines(axes=axes,
               x=csv["Voxel Occupancy Count"],
               background=csv["Voxel Count (Background)"],
               dynamic=csv["Voxel Count (Dynamic)"],
               other=csv["Voxel Count (Other)"])

    fig.savefig(pathlib.Path(file_info["filepath"]).with_suffix(".png"))


def plot_voxel_point_normal_angle_std_to_voxel_count(file_info):
    csv = pd.read_csv(file_info["filepath"], skiprows=2, header=None)

    fig, (axes_azimuth, axes_zenith) = build_figure(file_info, 2, 1)

    axes_azimuth.set_title(format_title(file_info["title"]).replace("Angle", "Azimuth"), fontsize=title_fontsize)
    axes_azimuth.set_xlabel(format_label(file_info["x_label"]).replace("Angle", "Azimuth"), fontsize=axis_fontsize)
    axes_azimuth.set_xscale('linear')
    axes_azimuth.set_xlim(0, 360)

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

    fig.savefig(pathlib.Path(file_info["filepath"]).with_suffix(".png"))


def plot_voxel_occupancy_interval_std_to_voxel_count(file_info):
    csv = pd.read_csv(file_info["filepath"])

    fig, axes = build_figure(file_info)

    plot_lines(axes=axes,
               x=csv["Voxel Occupancy Interval STD"],
               background=csv["Voxel Count (Background)"],
               dynamic=csv["Voxel Count (Dynamic)"],
               other=csv["Voxel Count (Other)"])

    fig.savefig(pathlib.Path(file_info["filepath"]).with_suffix(".png"))


def plot_voxel_occluded_percentage_to_voxel_count(file_info):
    csv = pd.read_csv(file_info["filepath"])

    fig, axes = build_figure(file_info)
    axes.set_xscale('linear')
    axes.set_xlim(0, 100)

    plot_lines(axes=axes,
               x=csv["Voxel Occluded Percentage"],
               background=csv["Voxel Count (Background)"],
               dynamic=csv["Voxel Count (Dynamic)"],
               other=csv["Voxel Count (Other)"])

    fig.savefig(pathlib.Path(file_info["filepath"]).with_suffix(".png"))


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
        if "VoxelPointCountToVoxelCount" == file_info["title"]:
            plot_voxel_point_count_to_voxel_count(file_info)
        elif "VoxelOccupancyCountToVoxelCount" == file_info["title"]:
            plot_voxel_occupancy_count_to_voxel_count(file_info)
        elif "VoxelPointNormalAngleSTDToVoxelCount" == file_info["title"]:
            plot_voxel_point_normal_angle_std_to_voxel_count(file_info)
        elif "VoxelOccupancyIntervalSTDToVoxelCount" == file_info["title"]:
            plot_voxel_occupancy_interval_std_to_voxel_count(file_info)
        elif "VoxelOccludedPercentageToVoxelCount" == file_info["title"]:
            plot_voxel_occluded_percentage_to_voxel_count(file_info)


if __name__ == '__main__':
    main()
