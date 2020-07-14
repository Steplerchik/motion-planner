import unittest

from motion_planner import *


class TestPlanner(unittest.TestCase):
    def test_planner_1(self):
        problem_definition_factory = ProblemDefinitionFactory(planner_type=SingleRandomPlanner,
                                                              planner_parameters={
                                                                  'intermediate_point_count': 1,
                                                                  'chromosome_count': 10,
                                                                  'iteration_count': 5
                                                              }
                                                              )
        problem_definition_factory.problem.solve()
        cost = problem_definition_factory.problem.cost
        print("Real cost: %.2f [m]" % cost)
        planner_cost = problem_definition_factory.problem.planner.cost
        print("Planner cost: %.2f [m]" % planner_cost)

        start = problem_definition_factory.planner_factory.start_position
        finish = problem_definition_factory.planner_factory.end_position
        obstacle_points = problem_definition_factory.planner_factory.obstacle_points
        planner = problem_definition_factory.problem.planner
        plot_rrt(planner, start, finish, obstacle_points)
        plot_cost_map(problem_definition_factory.navigation_function)

        self.assertTrue(cost == planner_cost)


if __name__ == '__main__':
    unittest.main()
