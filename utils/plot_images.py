import pathlib
import sys

from PIL import Image
from matplotlib import pyplot as plt

import matplotlib.pylab as pylab

root_folder = "G:/My Drive/蕭賢傑/4研究所/4研究所/蕭賢傑/畢業光碟/01.論文主體/圖片/"


def plot_images(rows, cols, image_path_list):
    fig, axes = plt.subplots(rows, cols, figsize=((3.20 * cols), (2.40 * rows)))
    fig: plt.Figure
    for i, (ax, image_path) in enumerate(zip(axes.flatten(), image_path_list)):
        ax: plt.Axes
        ax.set_xlabel(
            fr'({chr(ord("a") + i)}) ${{TH}}_{{Static,Null,Upper}}={image_path[image_path.index("NST") + 4:image_path.index("NST") + 11]}$'
        )
        ax.imshow(Image.open(image_path))
        ax.set_xticks([])
        ax.set_yticks([])
    fig.tight_layout()
    filepath = pathlib.Path(str(pathlib.Path(image_path_list[0]).parent) + "/13640-[NST-all].png")
    fig.savefig(filepath)
    img = Image.open(filepath)
    img.convert("L").save(filepath.with_stem(filepath.stem + "_gray"))


name_dict = {
    'E': 'west to east',
    'S': 'north to south',
    'W': 'east to west',
    'N': 'south to north',
}


def plot_reg(rows, cols, image_path_list, params, fis_size_scale=2):
    pylab.rcParams.update(params)
    fig, axes = plt.subplots(rows, cols, figsize=((3.20 * cols * fis_size_scale), (2.40 * rows * fis_size_scale)))
    fig: plt.Figure
    for i, (ax, image_path) in enumerate(zip(axes.flatten(), image_path_list)):
        ax: plt.Axes
        ax.set_xlabel(f'({chr(ord("a") + i)}) view from {name_dict[image_path[image_path.index("V") + 1]]}',
                      fontsize=18)
        ax.imshow(Image.open(image_path))
        ax.set_xticks([])
        ax.set_yticks([])
    fig.tight_layout()
    filepath = pathlib.Path(str(pathlib.Path(image_path_list[0]).parent) + "/[113][REG].png")
    fig.savefig(filepath)
    img = Image.open(filepath)
    img.convert("L").save(filepath.with_stem(filepath.stem + "_gray"))


def plot_shaded(rows, cols, image_path_list, params, fis_size_scale=2):
    pylab.rcParams.update(params)
    fig, axes = plt.subplots(rows, cols, figsize=((3.20 * cols * fis_size_scale), (2.40 * rows * fis_size_scale)))
    fig: plt.Figure
    for i, (ax, image_path) in enumerate(zip(axes.flatten(), image_path_list)):
        ax: plt.Axes
        ax.set_xlabel(
            f'({chr(ord("a") + i)}) frame {image_path[image_path.index("[Shaded][") + 9:image_path.index("].png")]}',
            fontsize=18)
        ax.imshow(Image.open(image_path))
        ax.set_xticks([])
        ax.set_yticks([])
    fig.tight_layout()
    filepath = pathlib.Path(str(pathlib.Path(image_path_list[0]).parent) + "/[Shaded].png")
    fig.savefig(filepath)
    img = Image.open(filepath)
    img.convert("L").save(filepath.with_stem(filepath.stem + "_gray"))


if __name__ == '__main__':
    plot_images(2, 3, [
        root_folder + "13640[NST--0.038298432--0.039096316][crop-320-240].png",
        root_folder + "13640[NST--0.037500548--0.038298432][crop-320x240].png",
        root_folder + "13640[NST--0.036702664--0.037500548][crop-320-240].png",
        root_folder + "13640[NST--0.035106896--0.036702664][crop-320x240].png",
        root_folder + "13640[NST--0.031117476--0.032713244][crop-320x240].png",
        root_folder + "13640[NST--0.023138636--0.024734404][crop-320x240].png",
    ])

    plot_reg(2,
             2, [
                 root_folder + "[113][REG][VE].png", root_folder + "[113][REG][VS].png",
                 root_folder + "[113][REG][VW].png", root_folder + "[113][REG][VN].png"
             ], {
                 'font.size': 16,
             },
             fis_size_scale=2)

    plot_shaded(
        2,
        3, [
            root_folder + "[Shaded][445].png", root_folder + "[Shaded][455].png", root_folder + "[Shaded][465].png",
            root_folder + "[Shaded][475].png", root_folder + "[Shaded][485].png", root_folder + "[Shaded][495].png"
        ], {
            'font.size': 16,
        },
        fis_size_scale=2)
