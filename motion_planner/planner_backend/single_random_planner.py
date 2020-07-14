from random import uniform, randint

from motion_planner import *


class SingleRandomPlanner(object):
    def __init__(self, space_info, optimization_objective, iteration_count=200, edge_size=0.5,
                 intermediate_point_count=1, chromosome_count=1,
                 shift_size=0.5, shift_angle_size=np.pi / 2):
        self._tree = Graph(np.array([None] * 3))
        self._trajectory = None
        self._cost = None

        self._space_info = space_info
        self._optimization_objective = optimization_objective
        self._iteration_count = iteration_count
        self._edge_size = edge_size
        self._intermediate_point_count = intermediate_point_count
        self._chromosome_count = chromosome_count
        self._shift_size = shift_size
        self._shift_ange_size = shift_angle_size

        self._min_x, self._max_x, self._min_y, self._max_y = space_info.state.boundaries

    @property
    def tree(self):
        return self._tree

    @property
    def trajectory(self):
        return self._trajectory

    @property
    def cost(self):
        return self._cost

    def get_cost(self, trajectory):
        return self._optimization_objective.cost(trajectory)

    def insert_node(self, parent_position, child_position):
        self._tree.add_vertex(child_position)
        self._tree.add_edge(parent_position, child_position, None)

    def build_trajectory(self, start_position, end_position):
        population, costs = self.initialize_population(start_position, end_position)
        print('Initial population:\n', population)
        print('Initial population costs:\n', costs)

        for _ in range(self._iteration_count):
            mutated_population, mutated_costs = self.mutation_random_shift(start_position, end_position)
            population.extend(mutated_population)
            costs.extend(mutated_costs)
            sorted_population_with_costs = sorted(zip(population, costs), key=lambda t: t[1])[:self._chromosome_count]
            population, costs = zip(*sorted_population_with_costs)
            population = list(population)
            costs = list(costs)

        self.create_tree(population[0])
        self._trajectory = self.tree.vertices
        self._cost = costs[0]

    def create_tree(self, chromosome):
        self._tree = Graph(chromosome[0])
        for index in range(1, len(chromosome)):
            self.insert_node(chromosome[index - 1], chromosome[index])

    def initialize_population(self, start_position, end_position):
        population = []
        costs = []
        for _ in range(self._chromosome_count):
            new_chromosome = self.get_chromosome(start_position, end_position)
            population.append(new_chromosome)
            costs.append(self.get_cost(new_chromosome))
        return population, costs

    def get_chromosome(self, start_position, end_position):
        new_chromosome = [start_position]
        for _ in range(self._intermediate_point_count):
            random_position = self._space_info.state.sample(self._min_x, self._max_x, self._min_y, self._max_y)
            new_chromosome.append(random_position)
        new_chromosome.append(end_position)
        return new_chromosome

    def mutation_shift(self, chromosome):
        mutated_chromosome = [chromosome[0]]
        for index in range(1, len(chromosome) - 1):
            shift_direction_angle = uniform(-np.pi, np.pi)
            shift = np.array([self._shift_size * np.cos(shift_direction_angle),
                              self._shift_size * np.sin(shift_direction_angle),
                              randint(-1, 2) * self._shift_ange_size])
            mutated_chromosome.append(np.add(chromosome[index], shift))
        mutated_chromosome.append(chromosome[-1])
        return mutated_chromosome

    def mutation_random_shift(self, start_position, end_position):
        return self.initialize_population(start_position, end_position)
