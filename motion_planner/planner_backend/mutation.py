from abc import ABC, abstractmethod


class Mutation(ABC):
    def __init__(self, space_info):
        self._space_info = space_info

    @abstractmethod
    def mutate(self, population):
        raise NotImplementedError()
