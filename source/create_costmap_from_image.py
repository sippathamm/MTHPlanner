from PIL import Image
import cv2


def create_cost_map_from_image(map: str, cost_map: str) -> None:
    image = Image.open(map)
    image = image.convert('L')
    pixel = image.load()

    width, height = image.size

    for y in range(height):
        for x in range(width):
            pixel[x, y] = 255 - pixel[x, y]

    with open(cost_map, 'w') as f:
        for y in range(height):
            for x in range(width):
                f.write(f'{pixel[x, y]}\t')
            f.write('\n')

    print(f'Saved to {cost_map}')

    gray_image = cv2.imread(map, cv2.IMREAD_GRAYSCALE)
    cv2.imshow('map', gray_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main() -> None:
    map_directory = 'map'
    map = f'../{map_directory}/scenario2.png'
    cost_map = f'../{map_directory}/scenario2.txt'

    create_cost_map_from_image(map, cost_map)


if __name__ == "__main__":
    main()
