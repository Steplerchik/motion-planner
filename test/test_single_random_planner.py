import unittest

from motion_planner import *


class TestPlanner(unittest.TestCase):
    def test_planner_1(self):
        problem_definition_factory = ProblemDefinitionFactory(planner_type=SimpleRandomPlanner,
                                                              planner_parameters={
                                                                  'intermediate_point_count': 1,
                                                                  'chromosome_count': 5,
                                                                  'iteration_count': 5,
                                                                  'mutations': [RandomSampleMutation,
                                                                                SteerMutation,
                                                                                AddPointMutation,
                                                                                RemovePointMutation],
                                                                  'mutation_parameters': [
                                                                      {'intermediate_point_count': 1},
                                                                      {'edge_size': 0.5},
                                                                      {'probability': 0.8},
                                                                      {'probability': 0.2}
                                                                  ]
                                                              }
                                                              )
        problem = problem_definition_factory.make_optimization_problem()
        problem.solve()
        cost = problem.cost
        print("Real cost: %.2f [m]" % cost)
        planner_cost = problem.planner.cost
        print("Planner cost: %.2f [m]" % planner_cost)
        self.assertTrue(cost == planner_cost)

        start = problem_definition_factory.planner_factory.start_position
        finish = problem_definition_factory.planner_factory.end_position
        obstacle_points = problem_definition_factory.planner_factory.obstacle_points
        planner = problem.planner
        plot_rrt(planner, start, finish, obstacle_points)


if __name__ == '__main__':
    unittest.main()
