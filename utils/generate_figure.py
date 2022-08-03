import pathlib

import numpy as np
from matplotlib import pyplot as plt

fig_size = (10, 10)
legend_fontsize = 16
scatter_annotate_xytext = (0, 10)

folder = pathlib.Path("../../result/jason-pcc/figure/")

colors = ["C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9"]


def gaussian(x, u, s, w=None):
    w = w if w is not None else 1
    v = np.power(s, 2)
    return w * 1 / (np.sqrt(2 * np.pi * v)) * np.exp(-np.power(x - u, 2) / (2 * v))


def plot_gmm(clusters, filename="gmm.png"):
    fig, axes = plt.subplots(figsize=fig_size)
    fig: plt.Figure
    axes: plt.Axes
    x = np.array([i / 255.0 for i in range(-255, 255, 1)])
    y_total = np.zeros(shape=x.shape)
    for index, cluster in enumerate(clusters):
        y = gaussian(x, cluster["u"], cluster["s"], cluster.get("w"))
        y_total += y if cluster["u"] >= 0 else -y
        label = f"cluster {index}"
        if "w" in cluster:
            label += f" w={cluster['w']:.3f}"
        label += f" u={cluster['u']:.3f}, s={cluster['s']:.3f}"
        axes.fill_between(x, y, step="pre", alpha=0.4)
        line, = axes.plot(x, y, label=label)

        xx = cluster["u"]
        yy = gaussian(cluster["u"], cluster["u"], cluster["s"], cluster.get("w"))
        axes.scatter(xx, yy, color=line.get_color())
        axes.annotate(f"({xx:.3f},{yy:.3f})",
                      (xx, yy),
                      xycoords='data',
                      textcoords="offset points",
                      xytext=scatter_annotate_xytext,
                      horizontalalignment='center',
                      verticalalignment='center')

    axes.plot(x, y_total, 'k', label="total probability", alpha=0.4)
    axes.legend(loc="lower right", fontsize=legend_fontsize)
    axes.grid()
    fig.savefig(folder / filename)


def plot_learning_rate(alpha_dicts, n, filename="learning_rate.png"):
    fig, axes = plt.subplots(figsize=fig_size)
    fig: plt.Figure
    axes: plt.Axes
    x = np.arange(n + 1)

    for alpha_dict in alpha_dicts:
        y = np.power(1 - alpha_dict["alpha"], x)
        line, = axes.plot(x, y, label=f"alpha={alpha_dict['alpha']}")

        xx = alpha_dict["annotate_x"]
        yy = np.power(1 - alpha_dict["alpha"], xx)
        axes.scatter(xx, yy, color=line.get_color())
        axes.annotate(f"({xx:.3f},{yy:.3f})",
                      (xx, yy),
                      xycoords='data',
                      textcoords="offset points",
                      xytext=scatter_annotate_xytext,
                      horizontalalignment='center',
                      verticalalignment='center')

    axes.set_yticks(np.arange(0.0, 1.1, 0.1))
    axes.legend(loc="upper right", fontsize=legend_fontsize)
    axes.grid()
    fig.savefig(folder / filename)


if __name__ == '__main__':
    print("generate_figure.py")

    folder.mkdir(parents=True, exist_ok=True)

    plot_gmm([{"u": -0.5, "s": 0.04}, {"u": 0, "s": 0.04}, {"u": 128 / 255, "s": 0.04}], "gmm_without_weight.png")
    plot_gmm([{"u": -0.5, "s": 0.04, "w": 0.9}, {"u": 128 / 255, "s": 0.04, "w": 0.1}], "gmm_weight[9_1].png")
    plot_gmm([{"u": -0.5, "s": 0.04, "w": 0.9},
              {"u": 0, "s": 0.04, "w": 0.01},
              {"u": 128 / 255, "s": 0.04, "w": 0.09}],
             "gmm_weight[9_0.5_0.5].png")
    plot_learning_rate([{"alpha": 0.00656, "annotate_x": 700}, {"alpha": 0.00198, "annotate_x": 350}],
                       700,
                       "learning_rate-n[700].png")
