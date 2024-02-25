from PIL import Image
import matplotlib.pyplot as plt
import numpy as np

from CreateImageFromCostMap import read_cost_map


def draw(starting_point_x: int, starting_point_y: int,
         goal_point_x: int, goal_point_y: int,
         waypoint_x: list, waypoint_y: list, waypoint_color: str,
         breakpoint_x: list, breakpoint_y: list, breakpoint_color: str,
         collision_waypoint_x: list, collision_waypoint_y: list, collision_waypoint_color: str,
         map: str, output: str) -> None:
    map_image = Image.open(map)
    map_width, map_height = map_image.size

    plt.figure(figsize=(map_width / 100, map_height / 100))

    plt.imshow(map_image, cmap='gray')

    plt.scatter(starting_point_x, starting_point_y, color='red', s=1000, zorder=10)
    plt.scatter(goal_point_x, goal_point_y, color='green', s=1000, zorder=10)

    plt.text(starting_point_x - 100, starting_point_y, 'Start', rotation=90, fontsize=48, ha='center', va='center')
    plt.text(goal_point_x + 100, goal_point_y, 'Goal', rotation=90, fontsize=48, ha='center', va='center')

    plt.scatter(collision_waypoint_x, collision_waypoint_y, color=collision_waypoint_color, marker='o', s=100, zorder=3)
    plt.scatter(breakpoint_x, breakpoint_y, color=breakpoint_color, marker='o', s=500, zorder=2)
    plt.plot(waypoint_x, waypoint_y, color=waypoint_color, linewidth=5, zorder=1)

    # plt.axis('equal')
    plt.axis('off')
    plt.savefig(output, dpi=100, bbox_inches='tight', pad_inches=0)
    plt.close()


def collision_check(cost_map: str, waypoint: list) -> (int, list):
    cost_map = read_cost_map(cost_map, '\t')

    height, width = len(cost_map), len(cost_map[0])

    cost_map = np.array(cost_map)

    collision_count = 0
    collision_waypoint = []

    x = np.array([point[0] for point in waypoint])
    y = np.array([point[1] for point in waypoint])

    dx = np.abs(np.diff(x))
    dy = np.abs(np.diff(y))

    sx = np.where(x[:-1] > x[1:], -1, 1)
    sy = np.where(y[:-1] > y[1:], -1, 1)

    x1 = x[:-1]
    y1 = y[:-1]

    if np.any(dx > dy):
        err = dx / 2.0
        while np.any(x1 != x[1:]):
            mask = (x1 >= 0) & (x1 < width) & (y1 >= 0) & (y1 < height)
            collision_points = np.where(cost_map[y1.astype(int), x1.astype(int)] >= 253)[0]
            if len(collision_points) > 0:
                collision_count += len(collision_points)
                collision_waypoint.extend(waypoint[i] for i in collision_points)
                break
            err -= dy
            mask = err < 0
            y1[mask] += sy[mask]
            err[mask] += dx[mask]
            x1 += sx
    else:
        err = dy / 2.0
        while np.any(y1 != y[1:]):
            mask = (x1 >= 0) & (x1 < width) & (y1 >= 0) & (y1 < height)
            collision_points = np.where(cost_map[y1.astype(int), x1.astype(int)] >= 253)[0]
            if len(collision_points) > 0:
                collision_count += len(collision_points)
                collision_waypoint.extend(waypoint[i] for i in collision_points)
                break
            err -= dx
            mask = err < 0
            x1[mask] += sx[mask]
            err[mask] += dy[mask]
            y1 += sy

    return collision_count, collision_waypoint


def main() -> None:
    map_directory = 'map'
    map = f'../{map_directory}/scenario2.png'
    cost_map = f'../{map_directory}/scenario2.txt'

    waypoint = [(250, 300), (261, 310), (273, 321), (285, 331), (296, 342), (308, 352), (319, 363), (331, 373), (342, 384), (354, 394), (365, 404), (376, 415), (388, 425), (399, 435), (410, 446), (420, 456), (431, 466), (442, 476), (452, 486), (463, 495), (473, 505), (483, 515), (493, 524), (502, 534), (512, 543), (521, 552), (530, 562), (539, 571), (547, 580), (556, 588), (564, 597), (572, 606), (580, 614), (587, 623), (594, 631), (601, 640), (608, 648), (615, 657), (621, 667), (627, 676), (633, 686), (639, 696), (644, 707), (649, 718), (654, 730), (659, 743), (664, 756), (668, 770), (672, 784), (676, 800), (680, 816), (684, 834), (687, 852), (691, 872), (694, 893), (696, 915), (699, 938), (702, 962), (704, 988), (706, 1015), (708, 1044), (710, 1074), (712, 1105), (713, 1138), (715, 1172), (716, 1208), (718, 1244), (719, 1281), (720, 1319), (722, 1358), (723, 1397), (725, 1437), (727, 1478), (729, 1519), (731, 1560), (733, 1601), (736, 1642), (738, 1684), (742, 1725), (745, 1766), (749, 1807), (753, 1847), (757, 1887), (762, 1926), (768, 1965), (774, 2003), (780, 2040), (787, 2076), (794, 2110), (802, 2144), (811, 2177), (820, 2208), (830, 2238), (841, 2266), (852, 2294), (864, 2320), (876, 2345), (888, 2369), (902, 2392), (915, 2414), (929, 2435), (944, 2455), (959, 2474), (974, 2493), (990, 2510), (1006, 2527), (1022, 2543), (1039, 2559), (1056, 2574), (1073, 2589), (1090, 2603), (1108, 2616), (1126, 2629), (1144, 2642), (1162, 2655), (1180, 2667), (1198, 2679), (1217, 2691), (1235, 2702), (1254, 2714), (1273, 2726), ]
    waypoint_x = [x for x, y in waypoint]
    waypoint_y = [y for x, y in waypoint]

    starting_point_x, starting_point_y = waypoint_x[0], waypoint_y[0]
    goal_point_x, goal_point_y = waypoint_x[-1], waypoint_y[-1]

    breakpoint = [(564.575, 597.577), (708.652, 1044.17), (811.617, 2177.14), ]
    breakpoint_x = [x for x, y in breakpoint]
    breakpoint_y = [y for x, y in breakpoint]

    collision_count, collision_waypoint = collision_check(cost_map, waypoint)
    # collision_waypoint = []  # Plot no collision waypoints

    n = len(waypoint)
    success = (1 - (collision_count / n)) * 100

    print('Collision count:', collision_count)
    print('Collision waypoint:', collision_waypoint)
    print('Success:', success)

    collision_waypoint_x = [x for x, y in collision_waypoint]
    collision_waypoint_y = [y for x, y in collision_waypoint]

    draw(starting_point_x, starting_point_y,
         goal_point_x, goal_point_y,
         waypoint_x, waypoint_y, 'blue',
         breakpoint_x, breakpoint_y, 'deeppink',
         collision_waypoint_x, collision_waypoint_y, 'red',
         map, '../Path Planning.png')


if __name__ == "__main__":
    main()
