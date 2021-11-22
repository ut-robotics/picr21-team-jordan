def get_default_values(path, file_name):
    """Looks for default threshold values for image processing in config file path"""
    try:
        with open(path + file_name, "r") as file:
            return [int(x) for x in file.read().split()]
    except FileNotFoundError:
        return [127, 127, 127, 255, 255, 255, 1, 1, 1, 1]


def save_default_values(path, file_name, array):
    with open(path + file_name, "w") as file:
        file.write(" ".join(array))
