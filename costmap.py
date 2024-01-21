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
    Waypoint = [(214, 159), (213, 159), (213, 160), (213, 161), (213, 162), (212, 163), (212, 164), (212, 165),
                (211, 166), (211, 167), (211, 168), (210, 169), (210, 170), (210, 171), (209, 172), (209, 173),
                (209, 174), (208, 175), (207, 176), (207, 177), (207, 178), (206, 179), (206, 180), (205, 180),
                (205, 181), (204, 182), (204, 183), (203, 184), (202, 185), (202, 186), (201, 186), (201, 187),
                (200, 188), (199, 189), (199, 190), (198, 190), (198, 191), (197, 192), (196, 193), (196, 194),
                (195, 195), (194, 196), (193, 197), (193, 198), (192, 198), (192, 199), (191, 200), (191, 201),
                (190, 201), (190, 202), (190, 203), (189, 204), (189, 205), (189, 206), (188, 207), (188, 208),
                (188, 209), (188, 210), (188, 211), (187, 212), (187, 213), (187, 214), (187, 215), (186, 216),
                (186, 217), (186, 218), (186, 219), (185, 221), (185, 222), (185, 223), (184, 224), (184, 225),
                (184, 226), (183, 228), (183, 229), (183, 230), (182, 232), (182, 233), (182, 234), (182, 236),
                (181, 237), (181, 238), (181, 240), ]

    # Separate into X and Y lists
    x_values = [x for x, y in Waypoint]
    y_values = [y for x, y in Waypoint]

    # Display the results
    print("X values:", x_values)
    print("Y values:", y_values)

    # Draw points on the image and save it
    draw_points(x_values, y_values, output_image_path, 'output_combined.png')

    print(f"Image created and saved to {output_image_path}")
