from motion_planner import *


class GeneticPlanner(object):
    def __init__(self, space_info, optimization_objective, iteration_count=200, intermediate_point_count=1,
                 chromosome_count=1, mutations=None, mutation_parameters=None):
        if mutations is None:
            mutations = [RandomSampleMutation]
        if mutation_parameters is None:
            mutation_parameters = [{}] * len(mutations)
        self._tree = Graph(np.array([None] * 3))
        self._trajectory = None
        self._cost = None

        self._space_info = space_info
        self._optimization_objective = optimization_objective
        self._iteration_count = iteration_count
        self._intermediate_point_count = intermediate_point_count
        self._chromosome_count = chromosome_count
        self._mutations = [mutations[index](space_info, **mutation_parameters[index]) for index in
                           range(len(mutations))]

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

    def get_chromosome_cost(self, trajectory):
        return self._optimization_objective.cost(trajectory)

    def get_population_cost(self, population):
        population_costs = []
        for chromosome in population:
            population_costs.append(self.get_chromosome_cost(chromosome))
        return population_costs

    def insert_node(self, parent_position, child_position):
        self._tree.add_vertex(child_position)
        self._tree.add_edge(parent_position, child_position, None)

    def build_trajectory(self, start_position, end_position):
        self._tree = Graph(start_position)
        population = self.initialize_population(start_position, end_position)
        population_costs = self.get_population_cost(population)
        trajectory = population[0]
        trajectory_cost = population_costs[0]
        for index in range(1, len(trajectory)):
            self.insert_node(trajectory[index - 1], trajectory[index])

        for _ in range(self._iteration_count):
            mutated_population = population
            for mutation in self._mutations:
                mutated_population = mutation.mutate(mutated_population)
            mutated_population_costs = self.get_population_cost(mutated_population)
            population.extend(mutated_population)
            population_costs.extend(mutated_population_costs)
            sorted_population_with_costs = sorted(zip(population, population_costs), key=lambda t: t[1])[
                                           :self._chromosome_count]
            population, population_costs = zip(*sorted_population_with_costs)
            population = list(population)
            population_costs = list(population_costs)

            old_trajectory = trajectory
            trajectory = population[0]
            trajectory_cost = population_costs[0]

            if not np.array_equal(old_trajectory, trajectory):
                for index in range(1, len(trajectory)):
                    self.insert_node(trajectory[index - 1], trajectory[index])
        self._trajectory = [tuple(position) for position in trajectory]
        self._cost = trajectory_cost

    def create_tree(self, chromosome):
        self._tree = Graph(chromosome[0])
        for index in range(1, len(chromosome)):
            self.insert_node(chromosome[index - 1], chromosome[index])

    def initialize_population(self, start_position, end_position):
        population = []
        for _ in range(self._chromosome_count):
            new_chromosome = self.get_chromosome(start_position, end_position)
            population.append(new_chromosome)
        return population

    def get_chromosome(self, start_position, end_position):
        new_chromosome = [start_position]
        for _ in range(self._intermediate_point_count):
            random_position = self._space_info.state.sample(self._min_x, self._max_x, self._min_y, self._max_y)
            new_chromosome.append(random_position)
        new_chromosome.append(end_position)
        return new_chromosome
