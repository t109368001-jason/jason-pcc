import re
from pathlib import Path

import matplotlib.pylab as pylab
import numpy as np
import pandas as pd
from PIL import Image
from matplotlib import pyplot as plt, cycler

import bd

marker_list = [
    'o',  # 'point'
    'D',  # 'diamond'
    '^',  # 'triangle_up'
    'v'  # 'triangle_down'
]

params = {
    'font.size':
        16,
    'figure.figsize': (10, 10),  # (10, 10) = 1000 x 1000 pixels
    'lines.markersize':
        10,
    "axes.prop_cycle":
        cycler(
            color=["r", "g", "b", "k"],
            linestyle=[
                '-',  # 'solid',
                ':',  # 'dotted',
                '--',  # 'dashed',
                '-.'  # 'dashdot'
            ],
            marker=marker_list)
}
pylab.rcParams.update(params)

savefig_kwargs = dict(bbox_inches='tight')
annotate_kwargs = dict(xycoords='data',
                       textcoords="offset points",
                       xytext=(0, 16),
                       horizontalalignment='center',
                       verticalalignment='center')

RESOLUTION_LIST = ["100", "200", "300", "400", "500"]

ROOT_FOLDER = Path("../../result/test-parameters/")

GPCC_BYTES = [582787403, 362976909, 252035824, 188440045, 149546411]
GPCC_PSNR = [70.4182, 65.0899, 61.5701, 59.1515, 57.1218]

file_info_pattern = re.compile(r'(?P<file_path>.*'
                               r'[/\\]test-(?P<parameter_name>[^-]+)-\d+-\d+'
                               r'[/\\]ZX-XS-20220707\[seg-split-(?P<resolution>\d+)]'
                               r'\[tmc3-(?P<qp>\d+)-0]'
                               r'\[[^-]+-(?P<value>[+-]?([0-9]*[.])?[0-9]+)-?(?P<value_2>[+-]?([0-9]*[.])?[0-9]+)?]'
                               r'.*\.csv)')

LABEL_COLUMN_NAME = "Parameter Value"
RESOLUTION_COLUMN_NAME = "Resolution (mm)"

BD_RATE_LABEL = "BD-Rate"
BD_PSNR_LABEL = "BD-PSNR"
DYNAMIC_STREAM_LABEL = "Dynamic Stream (%)"
DYNAMIC_LABEL = "Dynamic (bytes)"
STATIC_ADDED_LABEL = "StaticAdded (bytes)"
STATIC_REMOVED_LABEL = "StaticRemoved (bytes)"
TOTAL_LABEL = "Total (bytes)"
PSNR_LABEL = "A2B (db)"
WALL_LABEL = "Wall (s)"
MEM_LABEL = "Peek memory (KB)"

LABEL_COLUMN = [BD_RATE_LABEL, BD_PSNR_LABEL, DYNAMIC_STREAM_LABEL, ""]
LABEL_COLUMN.extend([DYNAMIC_LABEL] * len(RESOLUTION_LIST))
LABEL_COLUMN.extend([""])
LABEL_COLUMN.extend([STATIC_ADDED_LABEL] * len(RESOLUTION_LIST))
LABEL_COLUMN.extend([""])
LABEL_COLUMN.extend([STATIC_REMOVED_LABEL] * len(RESOLUTION_LIST))
LABEL_COLUMN.extend([""])
LABEL_COLUMN.extend([TOTAL_LABEL] * len(RESOLUTION_LIST))
LABEL_COLUMN.extend([""])
LABEL_COLUMN.extend([PSNR_LABEL] * len(RESOLUTION_LIST))
LABEL_COLUMN.extend([""])
LABEL_COLUMN.extend([WALL_LABEL] * len(RESOLUTION_LIST))
LABEL_COLUMN.extend([""])
LABEL_COLUMN.extend([MEM_LABEL] * len(RESOLUTION_LIST))
LABEL_COLUMN.extend([""])

RESOLUTION_COLUMN = [""] * 4
RESOLUTION_COLUMN.extend(RESOLUTION_LIST)
RESOLUTION_COLUMN.extend([""])
RESOLUTION_COLUMN.extend(RESOLUTION_LIST)
RESOLUTION_COLUMN.extend([""])
RESOLUTION_COLUMN.extend(RESOLUTION_LIST)
RESOLUTION_COLUMN.extend([""])
RESOLUTION_COLUMN.extend(RESOLUTION_LIST)
RESOLUTION_COLUMN.extend([""])
RESOLUTION_COLUMN.extend(RESOLUTION_LIST)
RESOLUTION_COLUMN.extend([""])
RESOLUTION_COLUMN.extend(RESOLUTION_LIST)
RESOLUTION_COLUMN.extend([""])
RESOLUTION_COLUMN.extend(RESOLUTION_LIST)
RESOLUTION_COLUMN.extend([""])


def gen_summary_csv(parameter_name, file_info_dict: dict):
    result = pd.DataFrame({LABEL_COLUMN_NAME: LABEL_COLUMN, RESOLUTION_COLUMN_NAME: RESOLUTION_COLUMN})

    for parameter_value, file_info_list in file_info_dict.items():
        result[parameter_value] = ""

        for file_info in file_info_list:
            copy_file_info_to_result_csv(file_info, result)

        cal_bd(parameter_value, result)

    result.to_csv(f"{ROOT_FOLDER}/test-{parameter_name}.csv", index=False)

    return result


def cal_bd(parameter_value, result):
    all_dynamic_bytes = result.loc[result[LABEL_COLUMN_NAME] == DYNAMIC_LABEL, parameter_value].astype(float).values
    all_bytes = result.loc[result[LABEL_COLUMN_NAME] == TOTAL_LABEL, parameter_value].astype(float).values
    all_psnr = result.loc[result[LABEL_COLUMN_NAME] == PSNR_LABEL, parameter_value].astype(float).values
    bd_rate = bd.bd_rate(GPCC_BYTES, GPCC_PSNR, all_bytes, all_psnr)
    bd_psnr = bd.bd_psnr(GPCC_BYTES, GPCC_PSNR, all_bytes, all_psnr)
    dynamic_stream_percentage = np.mean(all_dynamic_bytes / all_bytes) * 100.0
    result.loc[(result[LABEL_COLUMN_NAME] == BD_RATE_LABEL), parameter_value] = bd_rate
    result.loc[(result[LABEL_COLUMN_NAME] == BD_PSNR_LABEL), parameter_value] = bd_psnr
    result.loc[result[LABEL_COLUMN_NAME] == DYNAMIC_STREAM_LABEL, parameter_value] = dynamic_stream_percentage


def copy_file_info_to_result_csv(file_info, result):
    metric_csv = pd.read_csv(file_info["file_path"], index_col=0, names=["name", "value"], header=None)
    dynamic_bytes = metric_csv.loc[DYNAMIC_LABEL, "value"]
    static_added_bytes = metric_csv.loc[STATIC_ADDED_LABEL, "value"]
    static_removed_bytes = metric_csv.loc[STATIC_REMOVED_LABEL, "value"]
    total_bytes = int(dynamic_bytes) + int(static_added_bytes) + int(static_removed_bytes)
    a2b_psnr = metric_csv.loc[PSNR_LABEL, "value"]
    if not isinstance(a2b_psnr, str):
        a2b_psnr = a2b_psnr[0]
    resolution_filter = (result[RESOLUTION_COLUMN_NAME] == file_info["resolution"])
    result.loc[(result[LABEL_COLUMN_NAME] == DYNAMIC_LABEL) & resolution_filter, file_info["value"]] = dynamic_bytes
    result.loc[(result[LABEL_COLUMN_NAME] == STATIC_ADDED_LABEL) & resolution_filter,
               file_info["value"]] = static_added_bytes
    result.loc[(result[LABEL_COLUMN_NAME] == STATIC_REMOVED_LABEL) & resolution_filter,
               file_info["value"]] = static_removed_bytes
    result.loc[(result[LABEL_COLUMN_NAME] == TOTAL_LABEL) & resolution_filter, file_info["value"]] = total_bytes
    result.loc[(result[LABEL_COLUMN_NAME] == PSNR_LABEL) & resolution_filter, file_info["value"]] = a2b_psnr
    result.loc[(result[LABEL_COLUMN_NAME] == WALL_LABEL) & resolution_filter,
               file_info["value"]] = metric_csv.loc[WALL_LABEL, "value"]
    result.loc[(result[LABEL_COLUMN_NAME] == MEM_LABEL) & resolution_filter,
               file_info["value"]] = metric_csv.loc[MEM_LABEL, "value"]
    return result


def gen_summary_png(parameter_name, result_csv: pd.DataFrame, file_info_dict):
    fig, axes = plt.subplots()
    fig: plt.Figure
    axes: plt.Axes
    right_axes: plt.Axes = axes.twinx()
    # noinspection PyProtectedMember
    right_axes._get_lines.prop_cycler = axes._get_lines.prop_cycler
    filename = f"test-{parameter_name}.png"
    axes.set_title(f"{parameter_name} - BD-Rate/BD-PSNR")
    axes.set_xlabel(parameter_name)
    axes.set_ylabel("%")
    right_axes.set_ylabel("%")
    axes.set_ylim(-50, 0)
    right_axes.set_ylim(0, 50)

    x = np.array(list(file_info_dict.keys())).astype(float)
    x.sort()
    bd_rate = result_csv.loc[result_csv[LABEL_COLUMN_NAME] == BD_RATE_LABEL,
                             file_info_dict.keys()].values.flatten().astype(float)
    axes.plot(x, bd_rate, label="DB-Rate")
    for xx, yy in zip(x, bd_rate):
        axes.annotate(f"({xx:.4g}, {yy:.4g})", (xx, yy), **annotate_kwargs)

    dynamic_stream = result_csv.loc[result_csv[LABEL_COLUMN_NAME] == DYNAMIC_STREAM_LABEL,
                                    file_info_dict.keys()].values.flatten().astype(float)
    right_axes.plot(x, dynamic_stream, label="Dynamic Stream")
    for xx, yy in zip(x, dynamic_stream):
        right_axes.annotate(f"({xx:.4g}, {yy:.4g})", (xx, yy), **annotate_kwargs)

    axes.legend(loc="upper left")
    right_axes.legend(loc="upper right")
    axes.grid()
    filepath = ROOT_FOLDER / filename
    fig.savefig(filepath, **savefig_kwargs)
    img = Image.open(filepath)
    img.convert("L").save(filepath.with_stem(filepath.stem + "_gray"))


def to_file_info_list_group(files):
    file_info_list = [file_info_pattern.match(str(file)) for file in files]
    file_info_list = [file_info for file_info in file_info_list if file_info is not None]
    file_info_list = [file_info.groupdict() for file_info in file_info_list]
    file_info_dict_dict = {}
    for file_info in file_info_list:
        if file_info["parameter_name"] not in file_info_dict_dict:
            file_info_dict_dict[file_info["parameter_name"]] = {}
        if file_info["value"] not in file_info_dict_dict[file_info["parameter_name"]]:
            file_info_dict_dict[file_info["parameter_name"]][file_info["value"]] = []
        file_info_dict_dict[file_info["parameter_name"]][file_info["value"]].append(file_info)
    for parameter_name, file_info_dict in file_info_dict_dict.items():
        for value, file_info_list in file_info_dict.items():
            assert len(file_info_list) == len(RESOLUTION_LIST)
    return file_info_dict_dict


def main():
    files = list(ROOT_FOLDER.rglob("test-*/*/metric-summary.csv"))

    file_info_dict_dict = to_file_info_list_group(files)
    for parameter_name, file_info_dict in file_info_dict_dict.items():
        result_csv = gen_summary_csv(parameter_name, file_info_dict)
        gen_summary_png(parameter_name, result_csv, file_info_dict)


if __name__ == '__main__':
    main()
