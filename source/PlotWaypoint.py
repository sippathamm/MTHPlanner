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

    plt.text(starting_point_x, starting_point_y - 100, 'Start', rotation=0, fontsize=48, ha='center', va='center')
    plt.text(goal_point_x, goal_point_y + 100, 'Goal', rotation=0, fontsize=48, ha='center', va='center')

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

    waypoint = [(2619, 421), (2614.89, 423.252), (2610.81, 425.498), (2606.79, 427.732), (2602.88, 429.949), (2599.1, 432.142), (2595.49, 434.306), (2592.08, 436.434), (2588.9, 438.522), (2585.99, 440.562), (2583.39, 442.549), (2581.08, 444.497), (2578.91, 446.491), (2576.68, 448.639), (2574.21, 451.046), (2571.28, 453.818), (2567.72, 457.062), (2563.32, 460.883), (2557.88, 465.387), (2551.22, 470.681), (2543.13, 476.87), (2533.51, 484.016), (2522.56, 491.999), (2510.57, 500.653), (2497.84, 509.815), (2484.65, 519.32), (2471.3, 529.004), (2458.07, 538.701), (2445.27, 548.247), (2433.17, 557.477), (2422.08, 566.227), (2412.2, 574.366), (2403.39, 581.899), (2395.45, 588.863), (2388.17, 595.297), (2381.32, 601.24), (2374.71, 606.729), (2368.12, 611.804), (2361.32, 616.502), (2354.12, 620.862), (2346.3, 624.923), (2337.68, 628.721), (2328.21, 632.292), (2317.87, 635.672), (2306.65, 638.896), (2294.53, 641.998), (2281.5, 645.013), (2267.53, 647.976), (2252.6, 650.923), (2236.72, 653.888), (2219.85, 656.906), (2202.01, 660.002), (2183.36, 663.16), (2164.1, 666.354), (2144.41, 669.558), (2124.48, 672.746), (2104.5, 675.892), (2084.68, 678.97), (2065.18, 681.954), (2046.22, 684.818), (2027.98, 687.536), (2010.61, 690.088), (1994.15, 692.478), (1978.61, 694.716), (1963.99, 696.811), (1950.29, 698.775), (1937.51, 700.617), (1925.66, 702.348), (1914.75, 703.976), (1904.76, 705.513), (1895.71, 706.968), (1887.55, 708.354), (1880.09, 709.697), (1873.07, 711.024), (1866.24, 712.363), (1859.35, 713.744), (1852.16, 715.192), (1844.41, 716.738), (1835.86, 718.407), (1826.26, 720.23), (1815.36, 722.233), (1802.98, 724.439), (1789.22, 726.839), (1774.25, 729.421), (1758.25, 732.17), (1741.38, 735.074), (1723.83, 738.119), (1705.75, 741.29), (1687.32, 744.575), (1668.71, 747.959), (1650.09, 751.429), (1631.62, 754.969), (1613.4, 758.551), (1595.49, 762.143), (1577.98, 765.716), (1560.94, 769.239), (1544.46, 772.68), (1528.6, 776.01), (1513.44, 779.198), (1499.07, 782.213), (1485.56, 785.024), (1472.98, 787.608), (1461.33, 789.967), (1450.61, 792.112), (1440.82, 794.051), (1431.97, 795.794), (1424.04, 797.352), (1417.04, 798.733), (1410.96, 799.948), (1405.81, 801.006), (1401.57, 801.916), (1398.2, 802.696), (1395.38, 803.387), (1392.76, 804.039), (1389.97, 804.699), (1386.65, 805.419), (1382.44, 806.245), (1376.98, 807.228), (1369.9, 808.415), (1360.85, 809.857), (1349.46, 811.602), (1335.52, 813.678), (1319.45, 816.023), (1301.82, 818.557), (1283.18, 821.198), (1264.11, 823.864), (1245.17, 826.472), (1226.94, 828.943), (1209.97, 831.192), (1194.83, 833.14), (1182.1, 834.703), (1172.13, 835.828), (1164.46, 836.572), (1158.44, 837.018), (1153.4, 837.25), (1148.67, 837.353), (1143.59, 837.411), (1137.51, 837.507), (1129.76, 837.727), (1119.67, 838.153), (1106.58, 838.87), (1090.03, 839.93), (1070.39, 841.254), (1048.21, 842.728), (1024.06, 844.242), (998.49, 845.684), (972.07, 846.94), (945.361, 847.901), (918.924, 848.453), (893.32, 848.484), (869.112, 847.883), (846.731, 846.608), (826.086, 844.899), (806.955, 843.065), (789.118, 841.415), (772.354, 840.26), (756.44, 839.91), (741.156, 840.674), (726.281, 842.861), (711.593, 846.782), (696.871, 852.746), (681.943, 860.957), (666.827, 871.199), (651.593, 883.146), (636.308, 896.477), (621.041, 910.868), (605.86, 925.996), (590.833, 941.538), (576.029, 957.169), (561.516, 972.568), (547.361, 987.411), (533.63, 1001.43), (520.368, 1014.58), (507.617, 1026.88), (495.419, 1038.33), (483.817, 1048.95), (472.851, 1058.75), (462.565, 1067.74), (453, 1075.94), (444.198, 1083.35), (436.202, 1089.99), (429.032, 1095.87), (422.628, 1101.07), (416.911, 1105.66), (411.798, 1109.71), (407.21, 1113.29), (403.066, 1116.48), (399.284, 1119.36), (395.786, 1121.99), (392.488, 1124.45), (389.312, 1126.82), (386.185, 1129.16), (383.065, 1131.5), (379.919, 1133.87), (376.716, 1136.29), (373.421, 1138.77), (370.002, 1141.36), (366.427, 1144.06), (362.662, 1146.9), (358.674, 1149.9), (354.431, 1153.09), (349.909, 1156.49), (345.129, 1160.07), (340.117, 1163.82), (334.904, 1167.71), (329.517, 1171.74), (323.987, 1175.86), (318.341, 1180.07), (312.609, 1184.35), (306.819, 1188.66), (301, 1193), ]
    waypoint_x = [x for x, y in waypoint]
    waypoint_y = [y for x, y in waypoint]

    starting_point_x, starting_point_y = waypoint_x[0], waypoint_y[0]
    goal_point_x, goal_point_y = waypoint_x[-1], waypoint_y[-1]

    breakpoint = [(2583.39, 442.549), (2543.13, 476.87), (2422.08, 566.227), (2346.3, 624.923), (2219.85, 656.906), (2027.98, 687.536), (1895.71, 706.968), (1815.36, 722.233), (1650.09, 751.429), (1485.56, 785.024), (1401.57, 801.916), (1349.46, 811.602), (1182.1, 834.703), (1106.58, 838.87), (869.112, 847.883), (696.871, 852.746), (547.361, 987.411), (436.202, 1089.99), (389.312, 1126.82), (354.431, 1153.09), ]
    breakpoint_x = [x for x, y in breakpoint]
    breakpoint_y = [y for x, y in breakpoint]

    collision_count, collision_waypoint = collision_check(cost_map, waypoint)
    collision_waypoint = []  # Uncomment to not plot collision waypoints

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
