import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def drawSphere(ax,pos,r,c):
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    x = pos[0] + r * np.outer(np.cos(u), np.sin(v))
    y = pos[1] + r * np.outer(np.sin(u), np.sin(v))
    z = pos[2] + r * np.outer(np.ones(np.size(u)), np.cos(v))

    ax.plot_surface(x, y, z,  rstride=4, cstride=4, color=c, linewidth=0, alpha=0.5)


def drawCylinder(ax, pos, r, h, c):
    u = np.linspace(0, 2 * np.pi, 100)
    z = np.linspace(0, h, 50)

    # Create arrays for the cylinder surface
    r, z = np.meshgrid(r, z)
    x = pos[0] + r * np.cos(u)
    y = pos[1] + r * np.sin(u)

    # Create arrays for the top and bottom faces of the cylinder
    xtop, ytop = np.meshgrid(x[:, 0], y[:, 0])
    xbottom, ybottom = np.meshgrid(x[:, -1], y[:, -1])

    # Plot the cylinder
    ax.plot_surface(x, y, z , rstride=4, cstride=4, color=c, linewidth=0, alpha=0.5)
    ax.plot_surface(xtop, ytop, pos[2] * np.ones_like(xtop), color=c, linewidth=0, alpha=0.5)
    ax.plot_surface(xbottom, ybottom, (-pos[2] + h) * np.ones_like(xbottom), color=c, linewidth=0, alpha=0.5)



def putText(ax, pos, text):
    ax.text(pos[0], pos[1], pos[2]+0.2, text, size=10, zorder=1, color='k')

