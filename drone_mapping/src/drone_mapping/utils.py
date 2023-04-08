import numpy as np

"""
Utility Functions
"""


def l2p(grid: np.ndarray) -> np.ndarray:
    """
    The function calculates the probability p(x) for each element x in the input numpy array
    using the sigmoid function 1 / (1 + e^(-x)).
    Then it subtracts the result from 1 to get 1 - p(x), which is the final probability value.

    :param grid: a numpy array containing the input values for which the probability needs to be calculated
    :return: a numpy array containing the calculated probability values
    """
    return 1 - (1 / (1 + np.exp(grid)))


def p2l(p: np.ndarray) -> np.ndarray:
    """
    This function computes l(x) = log(p(x) / (1 - p(x))), where p(x) is the probability of an event occurring.

    :param p: A scalar or array of probabilities in the range [0,1] representing the probability of an event occurring
    :return: A scalar or array of values representing the natural logarithm of the odds ratio, l(x)
    """
    return np.log(p / (1 - p))
