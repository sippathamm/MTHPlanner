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
    map = f'../{map_directory}/scenario1.png'
    cost_map = f'../{map_directory}/scenario1.txt'

    waypoint = [(404, 2402), (413.028, 2400.21), (422.049, 2398.4), (431.057, 2396.59), (440.045, 2394.74), (449.006, 2392.87), (457.935, 2390.95), (466.823, 2388.98), (475.666, 2386.96), (484.455, 2384.86), (493.184, 2382.7), (501.847, 2380.45), (510.437, 2378.11), (518.948, 2375.67), (527.372, 2373.12), (535.704, 2370.46), (543.936, 2367.67), (552.061, 2364.75), (560.075, 2361.69), (567.968, 2358.49), (575.736, 2355.12), (583.371, 2351.59), (590.867, 2347.88), (598.217, 2343.99), (605.414, 2339.91), (612.453, 2335.64), (619.325, 2331.15), (626.025, 2326.45), (632.546, 2321.53), (638.881, 2316.38), (645.024, 2310.98), (650.974, 2305.34), (656.753, 2299.45), (662.389, 2293.32), (667.91, 2286.94), (673.344, 2280.32), (678.719, 2273.46), (684.064, 2266.36), (689.406, 2259.02), (694.773, 2251.45), (700.193, 2243.64), (705.694, 2235.6), (711.305, 2227.32), (717.053, 2218.82), (722.967, 2210.08), (729.074, 2201.12), (735.402, 2191.94), (741.979, 2182.53), (748.835, 2172.89), (755.995, 2163.04), (763.489, 2152.96), (771.345, 2142.67), (779.59, 2132.16), (788.253, 2121.44), (797.361, 2110.5), (806.943, 2099.35), (817.027, 2087.98), (827.64, 2076.41), (838.811, 2064.63), (850.567, 2052.65), (862.938, 2040.46), (875.939, 2028.06), (889.547, 2015.46), (903.725, 2002.63), (918.438, 1989.58), (933.651, 1976.29), (949.327, 1962.76), (965.432, 1948.98), (981.929, 1934.94), (998.783, 1920.63), (1015.96, 1906.05), (1033.42, 1891.18), (1051.13, 1876.02), (1069.06, 1860.57), (1087.16, 1844.8), (1105.41, 1828.72), (1123.77, 1812.32), (1142.2, 1795.59), (1160.66, 1778.52), (1179.13, 1761.1), (1197.56, 1743.32), (1215.92, 1725.19), (1234.18, 1706.68), (1252.3, 1687.79), (1270.23, 1668.52), (1287.96, 1648.85), (1305.44, 1628.78), (1322.63, 1608.3), (1339.5, 1587.4), (1356.02, 1566.07), (1372.15, 1544.31), (1387.86, 1522.11), (1403.16, 1499.48), (1418.07, 1476.43), (1432.58, 1452.99), (1446.73, 1429.17), (1460.53, 1404.97), (1473.98, 1380.43), (1487.11, 1355.54), (1499.92, 1330.33), (1512.44, 1304.8), (1524.67, 1278.99), (1536.63, 1252.89), (1548.34, 1226.53), (1559.8, 1199.91), (1571.04, 1173.06), (1582.07, 1145.99), (1592.89, 1118.71), (1603.53, 1091.24), (1614.01, 1063.59), (1624.32, 1035.78), (1634.49, 1007.82), (1644.54, 979.734), (1654.47, 951.525), (1664.3, 923.214), (1674.05, 894.813), (1683.72, 866.339), (1693.34, 837.806), (1702.92, 809.229), (1712.47, 780.622), (1722, 752), ]
    waypoint_x = [x for x, y in waypoint]
    waypoint_y = [y for x, y in waypoint]

    starting_point_x, starting_point_y = waypoint_x[0], waypoint_y[0]
    goal_point_x, goal_point_y = waypoint_x[-1], waypoint_y[-1]

    breakpoint = [(645.024, 2310.98), (862.938, 2040.46), (1372.15, 1544.31), ]
    breakpoint_x = [x for x, y in breakpoint]
    breakpoint_y = [y for x, y in breakpoint]

    collision_count, collision_waypoint = collision_check(cost_map, waypoint)
    collision_waypoint = []  # Disable collision waypoints plot

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
