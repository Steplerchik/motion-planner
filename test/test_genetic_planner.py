import unittest

from motion_planner import *


class TestPlanner(unittest.TestCase):
    def test_planner_1(self):
        problem_definition_factory = ProblemDefinitionFactory(planner_type=GeneticPlanner,
                                                              end_position=np.array([2, 5, 0]),
                                                              labyrinth=labyrinth.third(),
                                                              heuristic=NavigationFunctionDoubleNF1,
                                                              planner_parameters={
                                                                  'intermediate_point_count': 2,
                                                                  'chromosome_count': 10,
                                                                  'iteration_count': 30,
                                                                  'mutations': [AddAndSteerMutation,
                                                                                RemovePointMutation,
                                                                                RandomSampleMutation],
                                                                  'mutation_parameters': [
                                                                      {'probability': 0.9,
                                                                       'edge_size': 3.0},
                                                                      {'probability': 0.9},
                                                                      {'intermediate_point_count': 2}]
                                                              },
                                                              optimization_objective_parameters={'penalty_weight': 10000000.0}
                                                              )
        problem = problem_definition_factory.make_optimization_problem()
        problem.solve()
        cost = problem.cost
        planner_cost = problem.planner.cost

        self.assertTrue(cost == planner_cost)


if __name__ == '__main__':
    unittest.main()
