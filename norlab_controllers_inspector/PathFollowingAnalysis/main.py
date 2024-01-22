"""
Author: Cyril Goffin
Last modified: 31/05/2023
"""


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.spatial import KDTree


files_path = '/home/nicolas/Desktop/TeachAndRepeat_Husky/RunInterieur_26-05'


def data_analysis(path):
    # importing the data and removing points computed before the path following operation
    actual_path_df = pd.read_pickle('%s/loc.pkl' % path)
    target_path_df = pd.read_pickle('%s/ref_path.pkl' % path)
    actual_path = actual_path_df.to_numpy()
    target_path = target_path_df.to_numpy()
    print('Target path size:', len(target_path[:, 0]), ' |  Actual path size:', len(actual_path[:, 0]))

    # plotting both teach and repeat paths
    plt.figure(figsize=(8, 6))
    plt.rc('font', family='serif')
    plt.rc('axes', axisbelow=True)
    plt.grid()
    plt.scatter(target_path[:, 0], target_path[:, 1], s=4, c='#000000', marker='o', label='Target path (teach)')
    plt.scatter(actual_path[:, 0], actual_path[:, 1], s=4, c='#cd6ff6', marker='o', label='Actual path (repeat)')
    plt.legend()
    plt.xlabel('x / m')
    plt.ylabel('y / m')
    plt.savefig('%s/PathFollowingPlot' % path)

    # computing a KDTree (nearest-neighbor lookup) to find the nearest target path point for each actual path point
    ref_path_tree = KDTree(target_path)
    nearest_distances, distance_id = ref_path_tree.query(actual_path)
    median = np.median(nearest_distances)
    print('Error median:', np.round(median, 4), 'm')

    # evaluating the error (= nearest distance)
    fig, ax = plt.subplots()
    plt.ylabel('Cross-track error (XTE) / m')
    my_medianprops = dict(linestyle='-', linewidth=2, color='#fa5600')
    ax.boxplot(nearest_distances, medianprops=my_medianprops, showfliers=False)
    ax.xaxis.set_tick_params(labelbottom=False)
    ax.set_xticks([])
    plt.savefig('%s/ErrorBoxPlot' % path)

    # plotting paths with
    plt.figure(figsize=(8, 6))
    plt.rc('font', family='serif')
    plt.rc('axes', axisbelow=True)
    plt.grid()
    plt.scatter(target_path[:, 0], target_path[:, 1], s=6, c='C1', marker='o', label='Target path (teach)')
    plt.scatter(actual_path[:, 0], actual_path[:, 1], s=6, c=nearest_distances, norm=mpl.colors.LogNorm(),
                marker='o', label='Actual path (repeat)', cmap='viridis')
    plt.colorbar(label='Cross-track error (XTE) / m')
    plt.legend()
    plt.xlabel('x / m')
    plt.ylabel('y / m')
    plt.savefig('%s/PathFollowingPlotWithErrorColormap' % path)
    plt.show()


if __name__ == '__main__':
    data_analysis(files_path)
