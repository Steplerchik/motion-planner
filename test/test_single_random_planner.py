import unittest

from motion_planner import *


class TestPlanner(unittest.TestCase):
    def test_planner_1(self):
        problem_definition_factory = ProblemDefinitionFactory(planner_type=SimpleRandomPlanner,
                                                              planner_parameters={
                                                                  'intermediate_point_count': 2,
                                                                  'chromosome_count': 5,
                                                                  'iteration_count': 5,
                                                                  'mutations': [RandomSampleMutation,
                                                                                SteerMutation],
                                                                  'mutation_parameters': [{}, {'edge_size': 1.0}]
                                                              }
                                                              )
        problem = problem_definition_factory.make_optimization_problem()
        problem.solve()
        cost = problem.cost
        print("Real cost: %.2f [m]" % cost)
        planner_cost = problem.planner.cost
        print("Planner cost: %.2f [m]" % planner_cost)
        self.assertTrue(cost == planner_cost)


if __name__ == '__main__':
    unittest.main()
