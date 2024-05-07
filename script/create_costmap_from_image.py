from PIL import Image
import cv2
import os
import argparse


def create_cost_map_from_image(map_path: str, cost_map_path: str, show_image: bool) -> None:
    """
    Convert an image file representing a map into a text-based cost map file.

    Args:
        map_path (str): The file path of the input map image.
        cost_map_path (str): The file path to save the generated cost map.
        show_image (bool): Whether to show the original map image.

    Returns:
        None
    """
    image = Image.open(map_path)

    image = image.convert('L')  # Convert image to grayscale
    pixel = image.load()

    width, height = image.size

    # Invert pixel values to represent darker areas as higher cost
    for y in range(height):
        for x in range(width):
            pixel[x, y] = 255 - pixel[x, y]

    # Save the cost map to a text file
    with open(cost_map_path, 'w') as f:
        for y in range(height):
            for x in range(width):
                f.write(f'{pixel[x, y]}\t')
            f.write('\n')

    print(f'[INFO] Saved cost map into {cost_map_path}')

    if show_image:
        # Display the original map image for visualization
        print('[INFO] Press Q to quit')
        gray_image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        cv2.imshow('Map', gray_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main() -> None:
    parse = argparse.ArgumentParser(description='Create a cost map from an image file.')
    parse.add_argument('-i', '--input', dest='input', help='The file path of the input map image')
    parse.add_argument('-o', '--output', dest='output', help='The file path to save the generated cost map',
                       default='cost_map.txt')
    parse.add_argument('-n', '--no-show', dest='show_image', help='Don\'t show the original map image',
                       action='store_false')
    args = parse.parse_args()

    if not args.input:
        parse.print_help()
        return

    create_cost_map_from_image(args.input, args.output, args.show_image)

    print('[INFO] Done.')


if __name__ == "__main__":
    main()
