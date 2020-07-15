from motion_planner import Mutation
import numpy as np


class SteerMutation(Mutation):
    def __init__(self, space_info, edge_size=0.5):
        self._min_x, self._max_x, self._min_y, self._max_y = space_info.state.boundaries
        self._edge_size = edge_size
        super().__init__(space_info)

    def mutate(self, population):
        new_population = []
        for chromosome in population:
            new_chromosome = self.steer_chromosome(chromosome)
            new_population.append(new_chromosome)
        return new_population

    def steer_chromosome(self, chromosome):
        new_chromosome = [chromosome[0]]
        for old_intermediate_point in chromosome[1:-1]:
            random_point = self._space_info.state.sample(self._min_x, self._max_x, self._min_y, self._max_y)
            new_intermediate_point = self._space_info.state.steer(old_intermediate_point, random_point, self._edge_size)
            new_chromosome.append(new_intermediate_point)
        new_chromosome.append(chromosome[-1])
        return new_chromosome
