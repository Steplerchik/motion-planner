import unittest

from motion_planner import *


class TestPlanner(unittest.TestCase):
    def test_planner_1(self):
        problem_definition_factory = ProblemDefinitionFactory(planner_type=GeneticPlanner,
                                                              labyrinth=labyrinth.second(),
                                                              planner_parameters={
                                                                  'intermediate_point_count': 4,
                                                                  'chromosome_count': 10,
                                                                  'iteration_count': 10,
                                                                  'mutations': [RandomSampleMutation,
                                                                                AddPointMutation,
                                                                                RemovePointMutation,
                                                                                SteerMutation
                                                                                ],
                                                                  'mutation_parameters': [
                                                                      {'intermediate_point_count': 4},
                                                                      {'probability': 0.5},
                                                                      {'probability': 0.5},
                                                                      {'edge_size': 0.5}]
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
