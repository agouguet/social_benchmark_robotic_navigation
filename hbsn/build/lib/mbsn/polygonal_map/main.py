import matplotlib
from matplotlib import pyplot as plt
from importlib.resources import files

from mbsn.polygonal_map.polygonal_map import PolygonalMap

if __name__ == "__main__":
    scenario = "small"
    map_config_path = str(files('mbsn.data').joinpath(scenario+'/'))

    map = PolygonalMap(map_config_path, type="hexagon", area_minimum=0.3)

    poly = map.polygon
    grid = map.grid

    matplotlib.use('Qt5Agg')
    fig, (ax1, ax2) = plt.subplots(1, 2)
    map.plot(ax1)

    for id, poly in grid.items():
        poly.plot(ax2)

    figManager = plt.get_current_fig_manager()
    ax1.axis('off')
    ax1.set_aspect('equal', adjustable='box')
    ax2.axis('off')
    ax2.set_aspect('equal', adjustable='box')
    plt.show()