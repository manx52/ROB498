import numpy as np


def l2p(grid: np.ndarray) -> np.ndarray:
    """
    p(x) = 1 - \frac{1}{1 + e^l(x)}
    :param grid:
    :return:
    """
    return 1 - (1 / (1 + np.exp(grid)))

