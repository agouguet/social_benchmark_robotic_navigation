import os
from os.path import dirname
import matplotlib
from matplotlib import pyplot as plt, image
import numpy as np
import yaml
import cv2
from PIL import Image
from numpy import asarray
# from importlib.resources import files
from importlib_resources import files

from .py_floor_plan_segmenter.modules import do_segment

script_dir = files('mbsn.polygonal_map').joinpath("py_floor_plan_segmenter/custom.yml")
with open(script_dir) as f:
    config = yaml.load(f, Loader=yaml.SafeLoader)
    sigma_start = config["compute_labels_list"]["sigma_start"]
    sigma_step = config["compute_labels_list"]["sigma_step"]

def open_image(path):
    try:
        if os.path.exists(path):
            return cv2.imread(path)
    except cv2.error as e:
            print(f'Could not open file: {e}')

class Map():
    def __init__(self, map_directory_path) -> None:
        self.path = map_directory_path
        self.config = self.open_config()
        self._color = None
        self._gray = None
        self._segmented = None

    def open_config(self):
        try:
            map_config = open(self.path + "/map.yaml")
            return yaml.safe_load(map_config)
        except OSError as e:
            print(f'Could not open file: {e}')
        
    @property
    def color(self):
        if self._color is not None:
            return self._color
        
        self._color = open_image(self.path + "/" + self.config["image"])
        return self._color

    @property
    def gray(self):
        if self._gray is not None:
            return self._gray
        
        self._gray = cv2.cvtColor(self.color.copy(), cv2.COLOR_BGR2GRAY)
        self._gray = np.float32(np.invert(self._gray).astype(float) / 255)
        return self._gray
    
    @property
    def segmented(self, save=True):
        if self._segmented != None:
            return self._segmented
        
        if os.path.exists(self.path + "/segmented.png"):
            self._segmented = open_image(self.path + "segmented.png")
            return self._segmented
        else:
            print("Segment map...")
            self._segmented = do_segment(self.gray, **config)
            if save:
                cv2.imwrite(self.path + "/segmented.png", self._segmented) # Save the image
                print("Segmented image saved in share folder.")
            return self._segmented
            
if __name__ == "__main__":
    map_config_path = "/home/adam/Desktop/SBSN/ros2_ws/src/assets/maps/hospital/"

    map = Map(map_config_path)

    matplotlib.use('Qt5Agg')
    fig = plt.figure()
    axes = fig.add_subplot(111)
    axes.imshow(map.color)
    figManager = plt.get_current_fig_manager()
    # figManager.window.showMaximized()
    axes.axis('off')
    axes.set_aspect('equal', adjustable='box')
    plt.show()