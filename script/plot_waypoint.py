from PIL import Image
import matplotlib.pyplot as plt
import os
import argparse
import configparser
import re


def plot(start: tuple[float, float], start_size: int,
         goal: tuple[float, float], goal_size: int,
         breakpoints: list[tuple[float, float]], breakpoint_color: str, breakpoint_size: int,
         waypoints: list[tuple[float, float]], waypoint_color: str, waypoint_line_width: int,
         map_path: str, saved_path: str,
         dpi: int, show: bool) -> None:
    """
    Plot the waypoints and breakpoints on the map image.

    Args:
        start (tuple): Starting point coordinates (x, y).
        start_size (int): Size of the starting point.
        goal (tuple): Goal point coordinates (x, y).
        goal_size (int): Size of the goal point.
        breakpoints (list): List of breakpoint coordinates [(x1, y1), (x2, y2), ...].
        breakpoint_color (str): Color of the breakpoints.
        breakpoint_size (int): Size of the breakpoints.
        waypoints (list): List of waypoint coordinates [(x1, y1), (x2, y2), ...].
        waypoint_color (str): Color of the waypoints.
        waypoint_line_width (int): Width of the waypoints.
        map_path (str): Path to the map image.
        saved_path (str): Path to save the plotted image.
        dpi (int): DPI of the plotted image.
        show (bool): Whether to show the plotted image.
    """
    # Open the map image
    map_image = Image.open(map_path)
    map_width, map_height = map_image.size

    # Extract start and goal coordinates
    start_x, start_y = start[0], start[1]
    goal_x, goal_y = goal[0], goal[1]

    # Create a figure with appropriate size
    plt.figure(figsize=(map_width / 100, map_height / 100))

    # Display the map image
    plt.imshow(map_image, cmap='gray')

    # Plot the start and goal points
    plt.scatter(start_x, start_y, color='red', s=start_size, zorder=10, label='Start')
    plt.scatter(goal_x, goal_y, color='green', s=goal_size, zorder=10, label='Goal')

    # Plot the breakpoints
    plt.scatter([x for x, y in breakpoints], [y for x, y in breakpoints],
                color=breakpoint_color, marker='o', s=breakpoint_size, zorder=2, label='Breakpoint')

    # Plot the waypoints
    plt.plot([x for x, y in waypoints], [y for x, y in waypoints],
             color=waypoint_color, linewidth=waypoint_line_width, zorder=1, label='Waypoint')

    # Add legend and turn off axis
    plt.legend(loc='best')
    plt.axis('off')

    # Save the plotted image
    plt.savefig(saved_path, dpi=dpi, bbox_inches='tight', pad_inches=0)

    # Optionally display the plot
    if show:
        print('[INFO] Press Q to quit')
        plt.show()

    # Close the plot
    plt.close()


def main() -> None:
    parse = argparse.ArgumentParser(description='Plot the waypoints and breakpoints on the map image.')
    parse.add_argument('-i', '--input', dest='input', help='The file path of the configuration file.',
                       default='config.ini')
    parse.add_argument('-n', '--no-show', dest='show', help='Don\'t show the plotted image',
                       action='store_false')
    args = parse.parse_args()

    if not args.input:
        parse.print_help()
        return

    config = configparser.ConfigParser()
    config.read(args.input)

    map_path = str(config['Default']['map'])
    saved_path = str(config['Default']['saved'])
    breakpoints_input = str(config['Plot']['breakpoints'])
    waypoints_input = str(config['Plot']['waypoints'])
    dpi = int(config['Figure']['dpi'])
    starting_point_size = int(config['Figure']['StartingPointSize'])
    goal_point_size = int(config['Figure']['GoalPointSize'])
    breakpoint_color = str(config['Figure']['BreakpointColor'])
    breakpoint_size = int(config['Figure']['BreakpointSize'])
    waypoint_color = str(config['Figure']['WaypointColor'])
    waypoint_line_width = int(config['Figure']['WaypointLineWidth'])

    breakpoints = []
    matches = re.findall(r'\(.*?\)', breakpoints_input)
    for match in matches:
        bp = re.findall(r'-?\d+\.?\d*', match)
        breakpoints.append((float(bp[0]), float(bp[1])))

    waypoints = []
    matches = re.findall(r'\(.*?\)', waypoints_input)
    for match in matches:
        wp = re.findall(r'-?\d+\.?\d*', match)
        waypoints.append((float(wp[0]), float(wp[1])))

    if not waypoints:
        print('[ERROR] Waypoints cannot be empty.')
        return

    start = waypoints[0]
    goal = waypoints[-1]

    plot(start, starting_point_size,
         goal, goal_point_size,
         breakpoints, breakpoint_color, breakpoint_size,
         waypoints, waypoint_color, waypoint_line_width,
         map_path, saved_path,
         dpi, args.show)

    print('[INFO] Done.')


if __name__ == "__main__":
    main()
