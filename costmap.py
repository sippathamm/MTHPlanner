from PIL import Image
import matplotlib.pyplot as plt


def read_text_file(file_path):
    """Read data from the text file."""
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            row = [int(value) for value in line.strip().split('\t')]
            data.append(row)
    return data


def create_image(data, output_image_path):
    """Create an image from the data with inverted colors and save it."""
    height = len(data)
    width = len(data[0])

    # Create a new image with L mode (8-bit pixels, black and white)
    img = Image.new('L', (width, height))

    # Invert the pixel values
    inverted_pixels = [255 - value for row in data for value in row]
    img.putdata(inverted_pixels)

    # Save the image
    img.save(output_image_path)


def draw_points(x, y, image_path, output_image_path):
    """Draw points on an image and save it."""
    img = Image.open(image_path)

    # Create a scatter plot of points on the image
    plt.imshow(img, cmap='gray')  # Display the image as a background
    plt.scatter(x, y, color='red', marker='o', s=1)
    plt.axis('equal')  # Equal scaling to maintain aspect ratio
    plt.savefig(output_image_path)
    plt.close()


if __name__ == "__main__":
    # Replace 'out3.txt' and 'output_image.png' with your file paths
    input_file_path = 'out4.txt'
    output_image_path = 'costmap.png'

    # Read data from the text file
    data = read_text_file(input_file_path)

    # Create an image from the data and save it
    create_image(data, output_image_path)

    # Replace the lists 'x' and 'y' with your coordinates
    # Provided data
    data = [
        (214, 159), (213, 160), (213, 161), (212, 162), (212, 163), (211, 164), (211, 165),
        (211, 166), (210, 167), (210, 168), (209, 169), (209, 170), (209, 171), (208, 172),
        (208, 173), (207, 174), (207, 175), (206, 176), (206, 177), (206, 178), (205, 179),
        (205, 180), (204, 181), (204, 182), (203, 182), (203, 183), (202, 184), (201, 185),
        (201, 186), (200, 186), (200, 187), (199, 187), (199, 188), (198, 188), (198, 189),
        (197, 190), (196, 191), (196, 192), (195, 192), (195, 193), (195, 194), (194, 195),
        (194, 196), (193, 197), (193, 198), (193, 199), (192, 200), (192, 201), (192, 202),
        (192, 203), (192, 204), (191, 205), (191, 206), (191, 207), (191, 208), (190, 209),
        (190, 210), (190, 211), (189, 213), (189, 214), (189, 215), (189, 216), (188, 217),
        (188, 218), (187, 219), (187, 220), (187, 221), (187, 222), (186, 223), (186, 224),
        (186, 225), (185, 226), (185, 227), (185, 228), (184, 229), (184, 230), (184, 231),
        (183, 232), (183, 233), (183, 234), (182, 235), (182, 236), (182, 237), (181, 238),
        (181, 239), (181, 240),
    ]

    # Separate into X and Y lists
    x_values = [x for x, y in data]
    y_values = [y for x, y in data]

    # Display the results
    print("X values:", x_values)
    print("Y values:", y_values)

    # Draw points on the image and save it
    draw_points(x_values, y_values, output_image_path, 'output_combined.png')

    print(f"Image created and saved to {output_image_path}")
