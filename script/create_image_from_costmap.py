from PIL import Image
import os

# Directory where the map image is located
MAP_DIRECTORY = 'map'

# Name of the map file without the extension
MAP_NAME = 'turtlebot3_world'

MAP_PATH = os.path.join('..', MAP_DIRECTORY, MAP_NAME + '.png')
COST_MAP_PATH = os.path.join('..', MAP_DIRECTORY, MAP_NAME + '.txt')


def read_cost_map(file_path: str, delimiter: str) -> list:
    """
    Read the cost map data from a text file.

    Args:
        file_path (str): The path to the cost map text file.
        delimiter (str): The delimiter used in the text file to separate values.

    Returns:
        list: A 2D list representing the cost map data.
    """
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            row = [int(value) for value in line.strip().split(delimiter)]
            data.append(row)
    return data


def create_image(data: list, output_path: str) -> None:
    """
    Create an image from the given cost map data.

    Args:
        data (list): A 2D list representing the cost map data.
        output_path (str): The path to save the generated image.
    """
    height = len(data)
    width = len(data[0])

    image = Image.new('L', (width, height))

    pixel = [255 - value for row in data for value in row]
    image.putdata(pixel)

    image.save(output_path)


def create_image_from_cost_map(cost_map_path: str, map_path: str, delimiter: str) -> None:
    """
    Create an image from a cost map text file.

    Args:
        cost_map_path (str): The path to the cost map text file.
        map_path (str): The path to save the generated image.
        delimiter (str): The delimiter used in the text file to separate values.
    """
    data = read_cost_map(cost_map_path, delimiter)
    create_image(data, map_path)
    print(f'Saved to {map_path}')


def main() -> None:
    create_image_from_cost_map(COST_MAP_PATH, MAP_PATH, '\t')


if __name__ == '__main__':
    main()
