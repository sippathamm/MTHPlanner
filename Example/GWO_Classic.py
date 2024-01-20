import numpy as np

n_variable = 2

lower_bound = np.array([0, 0])
upper_bound = np.array([5, 5])

population = 50
iteration = 500


def random_position():
    return lower_bound + np.random.rand(population, n_variable) * (upper_bound - lower_bound)


def fitness_function(x):
    x0, x1 = x[:, 0], x[:, 1]
    return (x0 - 3.14) ** 2 + (x1 - 2.72) ** 2 + np.sin(3 * x0 + 1.41) + np.sin(4 * x1 - 1.73)


def get_best(position, cost):
    index_with_cost = dict()

    for i in range(population):
        index_with_cost[i] = cost[i]

    index_with_cost = sorted(index_with_cost.items(), key=lambda x: x[1])

    best_position = [position[index_with_cost[i][0]] for i in range(3)]
    best_cost = [index_with_cost[i][1] for i in range(3)]

    return best_position, best_cost


def get_new_position(a, best_position, position):
    new_position = [0, 0]

    for i in range(3):
        A = 2 * a * np.random.rand() - a
        C = 2 * np.random.rand()
        D = np.abs(C * best_position[0] - position)
        X = best_position[0] - A * D
        new_position += X

    new_position /= 3

    if new_position[0] < lower_bound[0]:
        new_position[0] = lower_bound[0]

    if new_position[0] > upper_bound[0]:
        new_position[0] = upper_bound[0]

    if new_position[1] < lower_bound[1]:
        new_position[1] = lower_bound[1]

    if new_position[1] > upper_bound[1]:
        new_position[1] = upper_bound[1]

    return new_position


position = random_position()
cost = fitness_function(position)

print('Initial Position: ')
print(position)
print('Initial Cost: ')
print(cost)

print('=====================')

for i in range(iteration):
    print('Iteration:', i)

    best_position, best_cost = get_best(position, cost)

    a = 2 * (1 - i / iteration)

    for j, each in enumerate(position):
        new_position = get_new_position(a, best_position, each)
        new_cost = fitness_function(np.array([new_position]))

        if new_cost < cost[j]:
            position[j] = new_position
            cost[j] = new_cost[0]

    best_position, best_cost = get_best(position, cost)

    print('Best position:')
    print(best_position)
    print('Best cost:')
    print(best_cost)

    print('----------------')
