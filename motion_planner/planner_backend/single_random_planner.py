from random import uniform, randint

from motion_planner import *


class SingleRandomPlanner(object):
    def __init__(self, space_info, iteration_count=0, edge_size=0, intermediate_point_count=1, chromosome_count=1,
                 shift_size=0.5, shift_angle_size=np.pi / 2):
        self.optimization_objective = None
        self._tree = Graph(np.array([None] * 3))
        self._trajectory = None
        self._cost = None

        self.space_info = space_info
        self.iteration_count = iteration_count
        self.edge_size = edge_size
        self.intermediate_point_count = intermediate_point_count
        self.chromosome_count = chromosome_count
        self.shift_size = shift_size
        self.shift_ange_size = shift_angle_size

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
        return self.optimization_objective.cost(trajectory)

    def insert_node(self, parent_position, child_position):
        self._tree.add_vertex(child_position)
        self._tree.add_edge(parent_position, child_position, None)

    def get_trajectory(self, start_position, end_position):
        population, costs = self.initialize_population(start_position, end_position)
        print('Initial population:\n', population)
        print('Initial population costs:\n', costs)

        for _ in range(self.iteration_count):
            mutated_population = []
            mutated_costs = []
            for chromosome in population:
                mutated_chromosome = self.mutation_shift(chromosome)
                mutated_population.append(mutated_chromosome)
                mutated_costs.append(self.get_cost(mutated_chromosome))
            population.extend(mutated_population)
            costs.extend(mutated_costs)
            sorted_population_with_costs = sorted(zip(population, costs), key=lambda t: t[1])[:self.chromosome_count]
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
        for _ in range(self.chromosome_count):
            new_chromosome = self.get_chromosome(start_position, end_position)
            population.append(new_chromosome)
            costs.append(self.get_cost(new_chromosome))
        return population, costs

    def get_chromosome(self, start_position, end_position):
        new_chromosome = [start_position]
        for _ in range(self.intermediate_point_count):
            random_position = self.space_info.state.sample(self._min_x, self._max_x, self._min_y, self._max_y)
            new_chromosome.append(random_position)
        new_chromosome.append(end_position)
        return new_chromosome

    def mutation_shift(self, chromosome):
        mutated_chromosome = [chromosome[0]]
        for index in range(1, len(chromosome) - 1):
            shift_direction_angle = uniform(-np.pi, np.pi)
            shift = np.array([self.shift_size * np.cos(shift_direction_angle),
                              self.shift_size * np.sin(shift_direction_angle),
                              randint(-1, 2) * self.shift_ange_size])
            mutated_chromosome.append(np.add(chromosome[index], shift))
        mutated_chromosome.append(chromosome[-1])
        return mutated_chromosome