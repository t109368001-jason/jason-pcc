import pathlib

import matplotlib.pylab as pylab
import numpy as np
from PIL import Image
from matplotlib import pyplot as plt, cycler

from bd import bd_rate, bd_psnr

from utils.bd import bd_rate, bd_psnr

marker_list = [
    'o',  # 'point'
    'D',  # 'diamond'
    '^',  # 'triangle_up'
    'v'  # 'triangle_down'
]

linestyle = [
    '-',  # 'solid',
    ':',  # 'dotted',
    '--',  # 'dashed',
    '-.'  # 'dashdot'
]
params = {
    'font.size': 16,
    'figure.figsize': (10, 10),  # (10, 10) = 1000 x 1000 pixels
    'lines.markersize': 10,
    "axes.prop_cycle": cycler(color=["r", "g", "b", "k"], linestyle=linestyle, marker=marker_list)
}
savefig_kwargs = dict(bbox_inches='tight')
annotate_kwargs = dict(xycoords='data',
                       textcoords="offset points",
                       xytext=(0, 16),
                       horizontalalignment='center',
                       verticalalignment='center')

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

        for index, multiplier in enumerate(cluster.get("sd_multiplier_list", [])):
            annotate_x = cluster["u"] + cluster["s"] * multiplier
            annotate_y = gaussian(annotate_x, cluster["u"], cluster["s"], cluster.get("w"))
            scatter = axes.scatter(annotate_x,
                                   annotate_y,
                                   label=fr"$\mu + {multiplier} \sigma$",
                                   zorder=line.zorder + 1,
                                   marker=marker_list[index % 3 + 1])
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


def quiver(x, y, axes: plt.Axes, **kwargs):
    u = np.diff(x)
    v = np.diff(y)
    pos_prob = x[:-1] + u / 2
    pos_y1 = y[:-1] + v / 2
    norm = np.sqrt(u**2 + v**2)
    axes.quiver(pos_prob, pos_y1, u / norm, v / norm, **kwargs)


def plot_schmitt(filename="schmitt_trigger.png"):
    x1 = [0, 0.2, 0.4, 0.6, 0.6, 0.6, 0.8, 1]
    x2 = [1, 0.6, 0.4, 0.4, 0.4, 0]
    t1 = 0.6
    t2 = 0.4
    y1 = [-0.5, -0.5, -0.5, -0.5, 0, 0.5, 0.5, 0.5]
    y2 = [0.5, 0.5, 0.5, 0, -0.5, -0.5]

    fig, axes = plt.subplots()
    fig: plt.Figure
    axes: plt.Axes

    axes.plot(x1, y1, color='r', marker='')
    axes.plot(x2, y2, color='g', marker='')

    quiver(x1, y1, axes, color='r', angles="xy", zorder=5, pivot="mid")
    quiver(x2, y2, axes, color='g', angles="xy", zorder=5, pivot="mid")

    # y = np.arange(-0.5, 0.5, 0.001)
    # axes.fill_betweenx(y, 0, 1, where=y > 0, color='r', alpha=0.5)
    # axes.fill_betweenx(y, 0, 1, where=y <= 0, color='g', alpha=0.5)

    axes.axis('off')
    axes.plot((1), (0), ls="", marker=">", ms=10, color="k", transform=axes.get_yaxis_transform(), clip_on=False)
    axes.axhline(0, color='k')
    axes.axvline(0, color='k')
    axes.annotate("Probability", (1, 0), **{**annotate_kwargs, "xytext": (0, -20)})
    axes.annotate("Static", (0.25, 0.25), **annotate_kwargs)
    axes.annotate("Dynamic", (0.25, -0.25), **annotate_kwargs)
    axes.annotate("TH1", (t1, 0), **annotate_kwargs)
    axes.annotate("TH2", (t2, 0), **annotate_kwargs)

    filepath = folder / filename
    fig.savefig(filepath, **savefig_kwargs)
    img = Image.open(filepath)
    img.convert("L").save(filepath.with_stem(filepath.stem + "_gray"))


def plot_comparison(proposed_br,
                    proposed_d1_psnr,
                    proposed_d2_psnr,
                    gpcc_br,
                    gpcc_d1_psnr,
                    gpcc_d2_psnr,
                    filename="comparison.png"):
    fig, (d1_axes, d2_axes) = plt.subplots(1, 2)
    fig: plt.Figure
    fig.set_size_inches(20, 10)
    d1_axes: plt.Axes
    d2_axes: plt.Axes
    d1_axes.set_xlabel("Bytes")
    d1_axes.set_ylabel("D1 PSNR (db)")
    d2_axes.set_xlabel("Bytes")
    d2_axes.set_ylabel("D2 PSNR (db)")
    y_min = np.min([np.min(proposed_d1_psnr), np.min(gpcc_d1_psnr), np.min(proposed_d2_psnr), np.min(gpcc_d2_psnr)])
    y_max = np.max([np.max(proposed_d1_psnr), np.max(gpcc_d1_psnr), np.max(proposed_d2_psnr), np.max(gpcc_d2_psnr)])
    offset = (y_max - y_min) * 0.1
    d1_axes.set_ylim(y_min - offset, y_max + offset)
    d2_axes.set_ylim(y_min - offset, y_max + offset)

    d1_axes.plot(proposed_br, proposed_d1_psnr, label="Proposed")
    d1_axes.plot(gpcc_br, gpcc_d1_psnr, label="G-PCC")
    d2_axes.plot(proposed_br, proposed_d2_psnr, label="Proposed")
    d2_axes.plot(gpcc_br, gpcc_d2_psnr, label="G-PCC")

    d1_axes.legend()
    d2_axes.legend()
    d1_axes.grid()
    d2_axes.grid()

    print('D1 BD-RATE: ', bd_rate(gpcc_br, gpcc_d1_psnr, proposed_br, proposed_d1_psnr))
    print('D1 BD-PSNR: ', bd_psnr(gpcc_br, gpcc_d1_psnr, proposed_br, proposed_d1_psnr))
    print('D2 BD-RATE: ', bd_rate(gpcc_br, gpcc_d2_psnr, proposed_br, proposed_d2_psnr))
    print('D2 BD-PSNR: ', bd_psnr(gpcc_br, gpcc_d2_psnr, proposed_br, proposed_d2_psnr))

    filepath = folder / filename
    fig.savefig(filepath, **savefig_kwargs)
    img = Image.open(filepath)
    img.convert("L").save(filepath.with_stem(filepath.stem + "_gray"))

    print('D1 BD-RATE: ', bd_rate(gpcc_br, gpcc_d1_psnr, proposed_br, proposed_d1_psnr))
    print('D1 BD-PSNR: ', bd_psnr(gpcc_br, gpcc_d1_psnr, proposed_br, proposed_d1_psnr))
    print('D2 BD-RATE: ', bd_rate(gpcc_br, gpcc_d2_psnr, proposed_br, proposed_d2_psnr))
    print('D2 BD-PSNR: ', bd_psnr(gpcc_br, gpcc_d2_psnr, proposed_br, proposed_d2_psnr))


if __name__ == '__main__':
    print("generate_figure.py")

    folder.mkdir(parents=True, exist_ok=True)

    plot_learning_rate([{
        "alpha": 0.01310,
        "annotate_x": 350,
        "name": r"$0.01\omega_{k,t_0}$ at $t_N=\frac{N_{GMM,Short}}{2}$"
    }, {
        "alpha": 0.00395,
        "annotate_x": 175,
        "name": r"$0.5\omega_{k,t_0}$ at $t_N=\frac{N_{GMM,Short}}{4}$"
    }, {
        "alpha": 0.00656,
        "annotate_x": 700,
        "name": r"$0.01\omega_{k,t_0}$ at $t_N=N_{GMM,Short}$"
    }, {
        "alpha": 0.00198,
        "annotate_x": 350,
        "name": r"$0.5\omega_{k,t_0}$ at $t_N=\frac{N_{GMM,Short}}{2}$"
    }, {
        "alpha": 0.00328,
        "annotate_x": 1400,
        "name": r"$0.01\omega_{k,t_0}$ at $t_N=2N_{GMM,Short}$"
    }, {
        "alpha": 0.00099,
        "annotate_x": 700,
        "name": r"$0.5\omega_{k,t_0}$ at $t_N=N_{GMM,Short}$"
    }],
                       700,
                       "weight_curve-n[700].png",
                       title=r"Weight Curve $N=700$")

    plot_gmm([{
        "u": 75,
        "s": 25
    }, {
        "u": 150,
        "s": 5
    }, {
        "u": 200,
        "s": 50
    }],
             "gmm.png",
             title=r"Gaussian Mixture Model $K=3$",
             xlim=[0, 255],
             ylim=[-0.03, 0.09])
    plot_gmm([{
        "u": 512,
        "s": 10
    }, {
        "u": 0,
        "s": 10
    }, {
        "u": 128,
        "s": 10
    }],
             "gmm_without_weight.png",
             title=r"Gaussian Mixture Model $K=3$")
    plot_gmm([{
        "u": 512,
        "s": 10,
        "w": 0.9
    }, {
        "u": 128,
        "s": 10,
        "w": 0.1
    }],
             "gmm_weight[9_1].png",
             title=r"Gaussian Mixture Model $K=2$")
    plot_gmm([{
        "u": 512,
        "s": 10,
        "w": 0.9
    }, {
        "u": 0,
        "s": 10,
        "w": 0.01
    }, {
        "u": 128,
        "s": 10,
        "w": 0.09
    }],
             "gmm_weight[9_0.5_0.5].png",
             title=r"Gaussian Mixture Model $K=3$")

    plot_gmm(
        [{
            "u": 128,
            "s": 10,
            "w": 0.1,
            "sd_multiplier_list": [0.5, 1, 2]
        }],
        "gmm_10w_sd_multiplier[0.5,1,2].png",
        title="Probability of Background GMM Cluster",
        xlim=(0, 255)
        # , ylim=[-0.4, 1.2]
        ,
        ylim=[-0.0015, 0.0045])

    plot_schmitt()

    plot_comparison([317897544, 124215175, 77629108, 53294010, 44614066], [70.7461, 64.8373, 61.2408, 58.9536, 56.5804],
                    [76.4772, 70.309, 66.4609, 64.3363, 61.1571],
                    [582787403, 362976909, 252035824, 188440045, 149546411],
                    [70.4182, 65.0899, 61.5701, 59.1515, 57.1218], [75.2006, 69.8344, 66.4166, 64.0504, 61.8438])
