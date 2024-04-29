from PIL import Image
import matplotlib.pyplot as plt
import os

# Directory where the map image is located
MAP_DIRECTORY = 'map'

# Name of the map file without the extension
MAP_NAME = 'turtlebot3_world'

# Name of the plotted image
OUTPUT_NAME = 'Turtlebot3_World'

MAP_PATH = os.path.join('..', MAP_DIRECTORY, MAP_NAME + '.png')
OUTPUT_PATH = os.path.join('..', OUTPUT_NAME + '.png')


def plot(start: tuple[float, float], goal: tuple[float, float],
         waypoint: list[tuple[float, float]], waypoint_color: str,
         breakpoint: list[tuple[float, float]], breakpoint_color: str,
         map_path: str, output: str) -> None:
    """
    Plot the waypoints and breakpoints on the map image.

    Args:
        start (tuple): Starting point coordinates (x, y).
        goal (tuple): Goal point coordinates (x, y).
        waypoint (list): List of waypoint coordinates [(x1, y1), (x2, y2), ...].
        waypoint_color (str): Color of the waypoints.
        breakpoint (list): List of breakpoint coordinates [(x1, y1), (x2, y2), ...].
        breakpoint_color (str): Color of the breakpoints.
        map_path (str): Path to the map image.
        output (str): Path to save the plotted image.
    """
    map_image = Image.open(map_path)
    map_width, map_height = map_image.size

    start_x, start_y = start[0], start[1]
    goal_x, goal_y = goal[0], goal[1]

    plt.figure(figsize=(map_width / 100, map_height / 100))
    plt.imshow(map_image, cmap='gray')

    # Starting point coordinates and label.
    plt.text(start_x + 30, start_y + 20,
             'Start', color='white', rotation=0, fontsize=12, ha='center', va='center')
    plt.scatter(start_x, start_y, color='red', s=10, zorder=10)

    # Goal point coordinates and label.
    plt.text(goal_x - 30, goal_y - 20,
             'Goal', color='white', rotation=0, fontsize=12, ha='center', va='center')
    plt.scatter(goal_x, goal_y, color='green', s=10, zorder=10)

    # Plot breakpoints
    plt.scatter([x for x, y in breakpoint], [y for x, y, in breakpoint],
                color=breakpoint_color, marker='o', s=5, zorder=2)

    # Plot Waypoints
    plt.plot([x for x, y in waypoint], [y for x, y, in waypoint],
             color=waypoint_color, linewidth=2, zorder=1)

    plt.axis('off')
    plt.savefig(output, dpi=100, bbox_inches='tight', pad_inches=0)
    plt.close()


def main() -> None:
    waypoint = [(211, 235), (210.103, 231.735), (209.105, 228.496), (207.903, 225.31), (206.398, 222.201), (204.486, 219.197), (202.126, 216.329), (199.51, 213.655), (196.888, 211.239), (194.513, 209.144), (192.636, 207.435), (191.431, 206.143), (190.775, 205.182), (190.469, 204.434), (190.312, 203.779), (190.105, 203.101), (189.695, 202.307), (189.119, 201.409), (188.461, 200.445), (187.805, 199.455), (187.233, 198.475), (186.808, 197.545), (186.501, 196.701), (186.261, 195.98), (186.038, 195.417), (185.78, 195.049), (185.434, 194.87), (184.939, 194.706), (184.232, 194.34), (183.249, 193.556), (181.926, 192.138), (180.225, 189.932), (178.201, 187.045), (175.935, 183.647), (173.508, 179.909), (171, 176), ]
    breakpoint = [(204.486, 219.197), (192.636, 207.435), (190.105, 203.101), (187.233, 198.475), (185.78, 195.049), (181.926, 192.138), ]

    start = waypoint[0]
    goal = waypoint[-1]

    plot(start, goal,
         waypoint, 'blue',
         breakpoint, 'deeppink',
         MAP_PATH, OUTPUT_PATH)

    print('[INFO] Done.')


if __name__ == "__main__":
    main()
