from PIL import Image


def read_cost_map(file_path: str, splitter: str) -> list:
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            row = [int(value) for value in line.strip().split(splitter)]
            data.append(row)
    return data


def create_image(data: list, output_path: str) -> None:
    height = len(data)
    width = len(data[0])

    image = Image.new('L', (width, height))

    pixel = [255 - value for row in data for value in row]
    image.putdata(pixel)

    image.save(output_path)


def create_image_from_cost_map(cost_map: str, map: str, splitter: str) -> None:
    data = read_cost_map(cost_map, splitter)
    create_image(data, map)
    print(f'Saved to {map}')


def main() -> None:
    cost_map = f'../out.txt'
    map = f'../AStar.png'

    create_image_from_cost_map(cost_map, map, '\t')


if __name__ == '__main__':
    main()
