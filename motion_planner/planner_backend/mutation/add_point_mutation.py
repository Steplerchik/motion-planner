import numpy as np

from motion_planner import Mutation


class AddPointMutation(Mutation):
    def __init__(self, space_info, probability=1.0):
        self._min_x, self._max_x, self._min_y, self._max_y = space_info.state.boundaries
        self._probability = probability
        super().__init__(space_info)

    def mutate(self, population):
        add_point = np.random.choice([True, False],
                                     p=[self._probability, 1 - self._probability])
        if not add_point:
            return population
        new_population = []
        for chromosome in population:
            new_chromosome = self.get_chromosome(chromosome)
            new_population.append(new_chromosome)
        return new_population

    def get_chromosome(self, chromosome):
        divide_index = np.random.randint(1, len(chromosome) - 1)
        new_chromosome = chromosome[:divide_index]
        random_point = self._space_info.state.interpolate(tuple(chromosome[divide_index - 1]),
                                                          tuple(chromosome[divide_index]))
        new_chromosome.append(random_point)
        new_chromosome.extend(chromosome[divide_index:])
        return new_chromosome
