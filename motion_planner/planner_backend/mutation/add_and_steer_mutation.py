import numpy as np

from motion_planner import Mutation, AddPointMutation, SteerMutation


class AddAndSteerMutation(Mutation):
    def __init__(self, space_info, probability=1.0, edge_size=0.5):
        self._add_mutation = AddPointMutation(space_info, probability)
        self._steer_mutation = SteerMutation(space_info, edge_size)
        super().__init__(space_info)

    def mutate(self, population):
        added_population = self._add_mutation.mutate(population)
        added_and_steered_population = self._steer_mutation.mutate(added_population)
        return added_and_steered_population
