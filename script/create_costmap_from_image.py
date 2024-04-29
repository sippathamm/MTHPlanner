from PIL import Image
import cv2
import os

# Directory where the map image is located
MAP_DIRECTORY = 'map'

# Name of the map file without the extension
MAP_NAME = 'validated'

MAP_PATH = os.path.join('..', MAP_DIRECTORY, MAP_NAME + '.png')
COST_MAP_PATH = os.path.join('..', MAP_DIRECTORY, MAP_NAME + '.txt')


def create_cost_map_from_image(map_path: str, cost_map_path: str) -> None:
    """
    Convert an image file representing a map into a text-based cost map file.

    Args:
        map_path (str): The file path of the input map image.
        cost_map_path (str): The file path to save the generated cost map.

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

    print(f'Saved to {cost_map_path}')

    # Display the original map image for visualization
    gray_image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    cv2.imshow('Map', gray_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main() -> None:
    create_cost_map_from_image(MAP_PATH, COST_MAP_PATH)


if __name__ == "__main__":
    main()
