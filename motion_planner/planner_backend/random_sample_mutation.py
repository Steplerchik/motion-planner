from motion_planner import Mutation


class RandomSampleMutation(Mutation):
    def __init__(self, space_info, **kwargs):
        self._min_x, self._max_x, self._min_y, self._max_y = space_info.state.boundaries
        super().__init__(space_info)

    def mutate(self, population):
        start_position = population[0][0]
        end_position = population[0][-1]
        intermediate_point_count = len(population[0]) - 2
        new_population = []
        for _ in range(len(population)):
            new_chromosome = self.get_chromosome(start_position, end_position, intermediate_point_count)
            new_population.append(new_chromosome)
        return new_population

    def get_chromosome(self, start_position, end_position, intermediate_point_count):
        new_chromosome = [start_position]
        for _ in range(intermediate_point_count):
            random_position = self._space_info.state.sample(self._min_x, self._max_x, self._min_y, self._max_y)
            new_chromosome.append(random_position)
        new_chromosome.append(end_position)
        return new_chromosome
