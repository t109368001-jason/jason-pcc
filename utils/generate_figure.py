import pathlib

import matplotlib.pylab as pylab
import numpy as np
from PIL import Image
from matplotlib import pyplot as plt, cycler

params = {
    'font.size': 16,
    'figure.figsize': (10, 10),  # (10, 10) = 1000 x 1000 pixels
    'lines.markersize': 10,
    "axes.prop_cycle": cycler(
        color=["r", "g", "b", "k"],
        linestyle=[
            '-',  # 'solid',
            ':',  # 'dotted',
            '--',  # 'dashed',
            '-.'  # 'dashdot'
        ],
        marker=[
            'o',  # 'point'
            'D',  # 'diamond'
            '^',  # 'triangle_up'
            'v'  # 'triangle_down'
        ]
    )
}
pylab.rcParams.update(params)
savefig_kwargs = dict(bbox_inches='tight')
annotate_kwargs = dict(
    xycoords='data',
    textcoords="offset points",
    xytext=(0, 16),
    horizontalalignment='center',
    verticalalignment='center'
)

folder = pathlib.Path("../../result/jason-pcc/figure/")


def gaussian(x, u, s, w=None):
    w = w if w is not None else 1
    v = np.power(s, 2)
    return w * 1 / (np.sqrt(2 * np.pi * v)) * np.exp(-np.power(x - u, 2) / (2 * v))


def plot_learning_rate(alpha_dicts, n, filename="learning_rate.png", **kwargs):
    fig, axes = plt.subplots()
    fig: plt.Figure
    axes: plt.Axes
    axes.set_title(kwargs.get("title", f"Weight Curve N=700{n}"))
    axes.set_xlabel(r"$t$")
    axes.set_ylabel(r"$\omega$")
    x = np.arange(n + 1)
    for index, alpha_dict in enumerate(alpha_dicts):
        y = np.power(1 - alpha_dict["alpha"], x)
        annotate_x = alpha_dict["annotate_x"]
        annotate_y = np.power(1 - alpha_dict["alpha"], annotate_x)
        axes.plot(x, y, label=fr"$\alpha={alpha_dict['alpha']}$", markevery=[annotate_x])
        axes.annotate(f"({annotate_x:.3g}, {annotate_y:.3g})", (annotate_x, annotate_y), **annotate_kwargs)

    axes.set_yticks(np.arange(0.0, 1.1, 0.1))
    axes.legend(loc="upper right")
    axes.grid()
    filepath = folder / filename
    fig.savefig(filepath, **savefig_kwargs)
    img = Image.open(filepath)
    img.convert("L").save(filepath.with_stem(filepath.stem + "_gray"))


def plot_gmm(clusters, filename="gmm.png", **kwargs):
    fig, axes = plt.subplots()
    fig: plt.Figure
    axes: plt.Axes
    axes.set_title(kwargs.get("title", "Gaussian Mixture Model"))
    axes.set_xlabel("intensity")
    axes.set_ylabel("probability")
    if "ylim" in kwargs:
        ylim = kwargs["ylim"]
        axes.set_ylim(*ylim)
    xlim = kwargs.get("xlim", [0, 512])
    x = np.arange(xlim[0], xlim[1], 0.2)
    handles = []
    for index, cluster in enumerate(clusters):
        y = gaussian(x, cluster["u"], cluster["s"], cluster.get("w"))
        peek_index = np.argmax(y)
        peek_x = x[peek_index]
        peek_y = y[peek_index]
        label = f"cluster {index}"
        if "w" in cluster:
            label += fr", $\omega={cluster['w']:.3g}$"
        label += fr", $\mu={cluster['u']:.3g}, \sigma={cluster['s']:.3g}$"
        axes.fill_between(x, y, step="pre", alpha=0.5)
        line, = axes.plot(x, y, label=label, alpha=0.8, markevery=[peek_index])
        handles.append(line)
        axes.annotate(f"({peek_x:.4g}, {peek_y:.4g})", (peek_x, peek_y), **annotate_kwargs)

        for multiplier in cluster.get("sd_multiplier_list", []):
            annotate_x = cluster["u"] + cluster["s"] * multiplier
            annotate_y = gaussian(annotate_x, cluster["u"], cluster["s"], cluster.get("w"))
            scatter = axes.scatter(annotate_x, annotate_y, label=fr"$\mu + {multiplier} \sigma$",
                                   zorder=line.zorder + 1)
            handles.append(scatter)
            axes.annotate(f"({annotate_x:.4g}, {annotate_y:.4g})", (annotate_x, annotate_y), **annotate_kwargs)

    if len(clusters) > 1:
        y_total = np.zeros(shape=x.shape)
        for index, cluster in enumerate(clusters):
            y = gaussian(x, cluster["u"], cluster["s"], cluster.get("w"))
            y_total += y if cluster["u"] <= 255 else -y
        line, = axes.plot(x, y_total, 'k', marker='', linestyle=(0, (3, 5, 1, 5)), label="total probability", alpha=0.8)
        handles.append(line)
    axes.legend(handles=handles, loc="lower right")
    axes.grid()
    filepath = folder / filename
    fig.savefig(filepath, **savefig_kwargs)
    img = Image.open(filepath)
    img.convert("L").save(filepath.with_stem(filepath.stem + "_gray"))


if __name__ == '__main__':
    print("generate_figure.py")

    folder.mkdir(parents=True, exist_ok=True)

    plot_learning_rate([{"alpha": 0.00198, "annotate_x": 350}, {"alpha": 0.00656, "annotate_x": 700}],
                       700,
                       "weight_curve-n[700].png", title=r"Weight Curve $N=700$")

    plot_gmm([{"u": 75, "s": 25}, {"u": 150, "s": 5}, {"u": 200, "s": 50}],
             "gmm.png",
             title=r"Gaussian Mixture Model $K=3$", xlim=[0, 255], ylim=[-0.03, 0.09])
    plot_gmm([{"u": 384, "s": 10}, {"u": 0, "s": 10}, {"u": 128, "s": 10}],
             "gmm_without_weight.png",
             title=r"Gaussian Mixture Model $K=3$")
    plot_gmm([{"u": 384, "s": 10, "w": 0.9}, {"u": 128, "s": 10, "w": 0.1}],
             "gmm_weight[9_1].png",
             title=r"Gaussian Mixture Model $K=2$")
    plot_gmm([{"u": 384, "s": 10, "w": 0.9},
              {"u": 0, "s": 10, "w": 0.01},
              {"u": 128, "s": 10, "w": 0.09}],
             "gmm_weight[9_0.5_0.5].png",
             title=r"Gaussian Mixture Model $K=3$")

    plot_gmm([{"u": 128, "s": 10, "w": 0.1,
               "sd_multiplier_list": [0.5, 1, 2]
               }],
             "gmm_10w_sd_multiplier[0.5,1,2].png",
             title="Probability of Background GMM Cluster", xlim=(0, 255)
             # , ylim=[-0.4, 1.2]
             , ylim=[-0.0015, 0.0045]
             )
