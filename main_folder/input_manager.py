"""
Plans to use this script to work with configuration files and inputs
"""

def get_default_values(path):
    try:
        with open(path + "trackbar_values", "r") as file:
            return [int(x) for x in file.read().split()]
    except FileNotFoundError:
        return [127, 127, 127, 255, 255, 255, 1, 1, 1, 1]

def save_default_values(path, array):
    with open(path + "trackbar_values", "w") as file:
            file.write(" ".join(array))