from PIL import Image
from create_image_from_costmap import read_cost_map

import matplotlib.pyplot as plt
import numpy as np

import os


def draw(starting_point_x: int, starting_point_y: int,
         goal_point_x: int, goal_point_y: int,
         waypoint_x: list, waypoint_y: list, waypoint_color: str,
         breakpoint_x: list, breakpoint_y: list, breakpoint_color: str,
         map_path: str, output: str) -> None:
    map_image = Image.open(map_path)
    map_width, map_height = map_image.size

    plt.figure(figsize=(map_width / 100, map_height / 100))

    plt.imshow(map_image, cmap='gray')

    plt.scatter(starting_point_x, starting_point_y, color='red', s=10, zorder=10)
    plt.scatter(goal_point_x, goal_point_y, color='green', s=10, zorder=10)

    plt.text(starting_point_x + 30, starting_point_y + 20, 'Start', color='white', rotation=0, fontsize=12, ha='center', va='center')
    plt.text(goal_point_x - 30, goal_point_y - 20, 'Goal', color='white', rotation=0, fontsize=12, ha='center', va='center')

    plt.scatter(breakpoint_x, breakpoint_y, color=breakpoint_color, marker='o', s=5, zorder=2)
    plt.plot(waypoint_x, waypoint_y, color=waypoint_color, linewidth=2, zorder=1)

    # plt.axis('equal')
    plt.axis('off')
    plt.savefig(output, dpi=100, bbox_inches='tight', pad_inches=0)
    plt.close()


def main() -> None:
    map_directory = 'map'
    map_name = 'turtlebot3_world.png'
    map_path = os.path.join('..', map_directory, map_name)

    waypoint = [(211, 235), (210.103, 231.735), (209.105, 228.496), (207.903, 225.31), (206.398, 222.201), (204.486, 219.197), (202.126, 216.329), (199.51, 213.655), (196.888, 211.239), (194.513, 209.144), (192.636, 207.435), (191.431, 206.143), (190.775, 205.182), (190.469, 204.434), (190.312, 203.779), (190.105, 203.101), (189.695, 202.307), (189.119, 201.409), (188.461, 200.445), (187.805, 199.455), (187.233, 198.475), (186.808, 197.545), (186.501, 196.701), (186.261, 195.98), (186.038, 195.417), (185.78, 195.049), (185.434, 194.87), (184.939, 194.706), (184.232, 194.34), (183.249, 193.556), (181.926, 192.138), (180.225, 189.932), (178.201, 187.045), (175.935, 183.647), (173.508, 179.909), (171, 176), ]
    waypoint_x = [x for x, y in waypoint]
    waypoint_y = [y for x, y in waypoint]

    starting_point_x, starting_point_y = waypoint_x[0], waypoint_y[0]
    goal_point_x, goal_point_y = waypoint_x[-1], waypoint_y[-1]

    breakpoint = [(204.486, 219.197), (192.636, 207.435), (190.105, 203.101), (187.233, 198.475), (185.78, 195.049), (181.926, 192.138), ]
    breakpoint_x = [x for x, y in breakpoint]
    breakpoint_y = [y for x, y in breakpoint]

    draw(starting_point_x, starting_point_y,
         goal_point_x, goal_point_y,
         waypoint_x, waypoint_y, 'blue',
         breakpoint_x, breakpoint_y, 'deeppink',
         map_path, '../Path Planning.png')

    print('[INFO] Done.')


if __name__ == "__main__":
    main()
