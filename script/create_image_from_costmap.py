from PIL import Image
import os
import argparse


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


def create_image(data: list, output_path: str, show_image: bool) -> None:
    """
    Create an image from the given cost map data.

    Args:
        data (list): A 2D list representing the cost map data.
        output_path (str): The path to save the generated image.
        show_image (bool): Whether to show the generated image.
    """
    height = len(data)
    width = len(data[0])

    image = Image.new('L', (width, height))

    pixel = [255 - value for row in data for value in row]
    image.putdata(pixel)

    image.save(output_path)
    if show_image:
        image.show('Map')
    image.close()


def create_image_from_cost_map(cost_map_path: str, map_path: str, delimiter: str, show_image: bool) -> None:
    """
    Create an image from a cost map text file.

    Args:
        cost_map_path (str): The path to the cost map text file.
        map_path (str): The path to save the generated image.
        delimiter (str): The delimiter used in the text file to separate values.
        show_image (bool): Whether to show the generated image.
    """
    data = read_cost_map(cost_map_path, delimiter)
    create_image(data, map_path, show_image)
    print(f'[INFO] Saved map image into {map_path}')


def main() -> None:
    parse = argparse.ArgumentParser(description='Create an image from a cost map text file.')
    parse.add_argument('-i', '--input', dest='input', help='The file path of the input cost map.')
    parse.add_argument('-o', '--output', dest='output', help='The file path to save the generated map image',
                       default='map.png')
    parse.add_argument('-n', '--no-show', dest='show', help='Don\'t show the original map image',
                       action='store_false')
    args = parse.parse_args()

    if not args.input:
        parse.print_help()
        return

    create_image_from_cost_map(args.input, args.output, '\t', args.show)

    print('[INFO] Done.')


if __name__ == '__main__':
    main()
