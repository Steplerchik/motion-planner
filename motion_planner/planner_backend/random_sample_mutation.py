from motion_planner import Mutation


class RandomSampleMutation(Mutation):
    def __init__(self, space_info, intermediate_point_count=1):
        self._min_x, self._max_x, self._min_y, self._max_y = space_info.state.boundaries
        self._intermediate_point_count = intermediate_point_count
        super().__init__(space_info)

    def mutate(self, population):
        start_position = population[0][0]
        end_position = population[0][-1]
        new_population = []
        for _ in range(len(population)):
            new_chromosome = self.get_chromosome(start_position, end_position)
            new_population.append(new_chromosome)
        return new_population

    def get_chromosome(self, start_position, end_position):
        new_chromosome = [start_position]
        for _ in range(self._intermediate_point_count):
            random_position = self._space_info.state.sample(self._min_x, self._max_x, self._min_y, self._max_y)
            new_chromosome.append(random_position)
        new_chromosome.append(end_position)
        return new_chromosome
