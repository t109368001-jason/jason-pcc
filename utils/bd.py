import numpy as np
import scipy.interpolate


def bd_integral(piecewise, x1, x2, y1, y2):
    p1 = np.polyfit(x1, y1, 3)
    p2 = np.polyfit(x2, y2, 3)
    # integration interval
    min_x = max(min(x1), min(x2))
    max_x = min(max(x1), max(x2))
    # find integral
    if piecewise == 0:
        p_int1 = np.polyint(p1)
        p_int2 = np.polyint(p2)

        integral1 = np.polyval(p_int1, max_x) - np.polyval(p_int1, min_x)
        integral2 = np.polyval(p_int2, max_x) - np.polyval(p_int2, min_x)
    else:
        lin = np.linspace(min_x, max_x, num=100, retstep=True)
        interval = lin[1]
        samples = lin[0]
        v1 = scipy.interpolate.pchip_interpolate(np.sort(x1), y1[np.argsort(x1)], samples)
        v2 = scipy.interpolate.pchip_interpolate(np.sort(x2), y2[np.argsort(x2)], samples)
        # Calculate the integral using the trapezoid method on the samples.
        integral1 = np.trapz(v1, dx=interval)
        integral2 = np.trapz(v2, dx=interval)
    return integral1, integral2, max_x, min_x


def bd_psnr(r1, psnr1, r2, psnr2, piecewise=0):
    l_r1 = np.log(r1)
    l_r2 = np.log(r2)

    int_psnr1, int_psnr2, max_l_r, min_l_r = bd_integral(piecewise, l_r1, l_r2, psnr1, psnr2)

    # find avg diff
    avg_diff = (int_psnr2 - int_psnr1) / (max_l_r - min_l_r)

    return avg_diff


def bd_rate(r1, psnr1, r2, psnr2, piecewise=0):
    l_r1 = np.log(r1)
    l_r2 = np.log(r2)

    int_l_r1, int_l_r2, max_psnr, min_psnr = bd_integral(piecewise, psnr1, psnr2, l_r1, l_r2)

    # find avg diff
    avg_exp_diff = (int_l_r2 - int_l_r1) / (max_psnr - min_psnr)

    avg_diff = (np.exp(avg_exp_diff) - 1) * 100
    return avg_diff
