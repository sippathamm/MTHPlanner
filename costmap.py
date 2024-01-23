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
    plt.scatter(x, y, color='red', marker='o', s=0.2)
    plt.axis('equal')  # Equal scaling to maintain aspect ratio
    plt.savefig(output_image_path)
    plt.close()


if __name__ == "__main__":
    # Replace 'out3.txt' and 'output_image.png' with your file paths
    input_file_path = 'Costmap.txt'
    output_image_path = 'costmap.png'

    # Read data from the text file
    data = read_text_file(input_file_path)

    # Create an image from the data and save it
    create_image(data, output_image_path)

    # Replace the lists 'x' and 'y' with your coordinates
    # Provided data
    Waypoint = [(214, 159), (213, 160), (213, 161), (213, 163), (213, 164), (213, 166), (213, 167), (213, 169),
                (213, 170), (213, 172), (213, 173), (212, 175), (212, 176), (212, 178), (212, 179), (212, 180),
                (212, 182), (211, 183), (211, 185), (211, 186), (211, 187), (210, 189), (210, 190), (210, 191),
                (209, 192), (209, 194), (208, 195), (208, 196), (207, 197), (207, 198), (206, 200), (206, 201),
                (205, 202), (205, 203), (204, 204), (204, 205), (203, 206), (202, 207), (202, 208), (201, 209),
                (200, 210), (200, 211), (199, 212), (198, 213), (198, 214), (197, 215), (196, 216), (195, 217),
                (194, 218), (194, 219), (193, 220), (193, 221), (192, 222), (191, 222), (191, 223), (190, 224),
                (190, 225), (189, 226), (188, 227), (188, 228), (187, 229), (186, 230), (186, 231), (185, 231),
                (185, 232), (184, 232), (184, 233), (184, 234), (183, 234), (183, 235), (182, 235), (182, 236),
                (182, 237), (181, 237), (181, 238), (181, 239), (180, 239), (181, 240), ]

    # Separate into X and Y lists
    x_values = [x for x, y in Waypoint]
    y_values = [y for x, y in Waypoint]

    # Display the results
    print("X values:", x_values)
    print("Y values:", y_values)

    # Draw points on the image and save it
    draw_points(x_values, y_values, output_image_path, 'output_combined.png')

    print(f"Image created and saved to {output_image_path}")
