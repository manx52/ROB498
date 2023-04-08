import numpy as np


def l2p(grid: np.ndarray) -> np.ndarray:
    """
    p(x) = 1 - \frac{1}{1 + e^l(x)}
    :param grid:
    :return:
    """
    return 1 - (1 / (1 + np.exp(grid)))


def p2l(p):
    """
    l(x) = log(\frac{p(x)}{1 - p(x)})
    :param p:
    :return:
    """
    return np.log(p / (1 - p))


def cost_to_come(trajectory_o):
    # The cost to get to a node from lavalle

    # print("trajectory_o: ", trajectory_o)
    end_pt = trajectory_o[-1, :]
    pos = trajectory_o[1, :]
    # print("end_pt: ", end_pt)
    # print("self.pos: ", self.pos)
    cost = np.linalg.norm(pos - end_pt)

    return cost
