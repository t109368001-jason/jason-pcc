import pathlib
import sys

from PIL import Image
from matplotlib import pyplot as plt


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


if __name__ == '__main__':
    plot_images(int(sys.argv[1]), int(sys.argv[2]), sys.argv[3:])
