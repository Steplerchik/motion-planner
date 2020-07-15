import numpy as np

from motion_planner import AddPointMutation


class RemovePointMutation(AddPointMutation):
    def __init__(self, space_info, probability=1.0):
        super().__init__(space_info, probability)

    def get_chromosome(self, chromosome):
        if len(chromosome) == 2:
            return chromosome
        new_chromosome = chromosome
        remove_index = np.random.randint(1, len(chromosome) - 1)
        del new_chromosome[remove_index]
        return new_chromosome
