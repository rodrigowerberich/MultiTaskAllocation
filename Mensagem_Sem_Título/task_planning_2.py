from enum import Enum
import math
from functools import reduce
import copy
import heapq

class MuTaskType(Enum):
    t0 = 't0'
    t1 = 't1'
    t2 = 't2'
    t3 = 't3'


class RobotType(Enum):
    a1 = 'a1'
    a2 = 'a2'
    a3 = 'a3'


def mu_task_type_effort(mu_task_type, robot_type):
    effort_dict = {RobotType.a1:
                       {MuTaskType.t0: 0,
                        MuTaskType.t1: 1,
                        MuTaskType.t2: 5,
                        MuTaskType.t3: 8.7},
                   RobotType.a2:
                       {MuTaskType.t0: 0,
                        MuTaskType.t1: 6,
                        MuTaskType.t2: math.inf,
                        MuTaskType.t3: 16},
                   RobotType.a3:
                       {
                           MuTaskType.t0: 0,
                           MuTaskType.t1: -5,
                           MuTaskType.t2: 6,
                           MuTaskType.t3: 9.18
                       }
                   }
    return effort_dict[robot_type][mu_task_type]


def mu_task_reward(mu_task_type):
    reward_dict = {MuTaskType.t0: 0,
                   MuTaskType.t1: 12.12,
                   MuTaskType.t2: 6.625,
                   MuTaskType.t3: 0.8}
    return reward_dict[mu_task_type]


class MuTask:
    def __init__(self, name, mu_task_type, position, previous=None):
        self.name = name
        self.type = mu_task_type
        self.position = position
        self.previous = previous

    def __str__(self):
        # if self.previous is not None:
        #     return 'Mu'+str(self.name)+str(self.previous)
        return 'Mu'+str(self.name)+','+str(self.position)

    def __repr__(self):
        return str(self)

    def get_all_pre_requisites(self):
        pre_requisites = []
        if self.previous is not None:
            pre_requisites += self.previous
            pre_requisites = reduce(lambda pre, task: pre+task.get_all_pre_requisites(), self.previous, pre_requisites)
        return pre_requisites

    def add_task_as_previous(self, mu_task):
        if self.previous is None:
            self.previous = []
        self.previous.append(mu_task)

    def add_tasks_as_previous(self, mu_tasks):
        if self.previous is None:
            self.previous = []
        self.previous += mu_tasks


class Node:
    def __init__(self, tasks, cost):
        self.tasks = tasks
        self.cost = cost

    def __le__(self, other):
        return self.cost <= other.cost

    def __lt__(self, other):
        return self.cost < other.cost

    def __str__(self):
        return 'Node: ('+str(self.tasks)+', '+str(round(self.cost, 2))+')'

    def __repr__(self):
        return str(self)


def has_all_prerequisite(task, node_to_expand):
    current_tasks = node_to_expand.tasks
    if task.previous is None:
        return True
    return all([pre_task in current_tasks for pre_task in task.previous])


def expand_open(open_list, tasks):
    node_to_expand = heapq.heappop(open_list)
    # print('Node to expand', node_to_expand)
    expand_task = node_to_expand.tasks[-1]
    expand_candidates = list(filter(lambda task: has_all_prerequisite(task, node_to_expand), tasks))
    expand_candidates = list(filter(lambda task: task not in node_to_expand.tasks, expand_candidates))
    # print('Expand candidate: ', expand_candidates)
    if len(expand_candidates) is not 0:
        for expand_candidate in expand_candidates:
            cost = point_distance(expand_candidate.position, expand_task.position)
            heapq.heappush(open_list, Node(node_to_expand.tasks+[expand_candidate], node_to_expand.cost+cost))
        return None
    return node_to_expand


class Task:
    def __init__(self, name, mu_tasks, priority, deadline):
        self.name = name
        self.mu_tasks = mu_tasks
        self.priority = priority
        self.deadline = deadline

    def __str__(self):
        return 'T'+str(self.name)

    def __repr__(self):
        return str(self)

    def total_reward(self):
        reward = 0
        for task in self.mu_tasks:
            reward += mu_task_reward(task.type)
            reward = reduce(lambda total, pre_task: total+mu_task_reward(pre_task.type), task.get_all_pre_requisites(),
                            reward)
        return reward

    def total_mu_tasks_effort(self, robot):
        return Task.calculate_mu_tasks_effort(self.mu_tasks, robot)

    def total_movement_effort(self, robot):
        return Task.calculate_movement_effort(self.mu_tasks, robot)

    def total_effort(self, robot):
        return self.total_movement_effort(robot).cost+self.total_mu_tasks_effort(robot)

    @staticmethod
    def calculate_movement_effort(partial_mu_tasks, robot):
        mu_tasks = copy.deepcopy(partial_mu_tasks)
        robot_location_task = MuTask(robot.name, MuTaskType.t0, robot.position)
        first_mu_tasks = reduce(lambda l, mu_t: l + mu_t.get_all_pre_requisites(), mu_tasks, [])
        first_mu_tasks = list(filter(lambda l: l.previous is None, first_mu_tasks))
        for mu_task in first_mu_tasks:
            mu_task.previous = [robot_location_task]

        all_mu_tasks = reduce(lambda l, mu_t: l + mu_t.get_all_pre_requisites(), partial_mu_tasks, partial_mu_tasks)
        # print('All mu tasks:', all_mu_tasks)
        open_list = [Node([robot_location_task], 0)]
        # print(open_list)
        result = None
        while result is None:
            result = expand_open(open_list, all_mu_tasks)
        return result

    @staticmethod
    def calculate_mu_tasks_effort(partial_mu_tasks, robot):
        all_mu_tasks = reduce(lambda l, mu_t: l+mu_t.get_all_pre_requisites(), partial_mu_tasks, partial_mu_tasks)
        mu_tasks_effort = reduce(lambda total, mu_task: total+mu_task_type_effort(mu_task.type, robot.type),
                                 all_mu_tasks, 0)
        return mu_tasks_effort


class Robot:
    def __init__(self, name, robot_type, position):
        self.name = name
        self.type = robot_type
        self.position = position

    def __str__(self):
        return 'R'+str(self.name)

    def __repr__(self):
        return str(self)


def point_distance(x, y):
    return math.sqrt((math.pow(x[0] - y[0], 2) + (math.pow(x[1]-y[1], 2))))


def connection_probability(position):
    distance_to_base = point_distance([0, 0], position)
    if distance_to_base <= 20:
        return (-0.05*distance_to_base)+1
    return 0


def find_best_scenario(best_scenario, curr_scenario):
    if best_scenario[4] < curr_scenario[4]:
        return curr_scenario
    else:
        return best_scenario


def find_best_scenario2(best_scenario, curr_scenario, ignore_list):
    inside_ignore_list = any([curr_scenario[1].name == ignored[1].name for ignored in ignore_list])
    if best_scenario[3] < curr_scenario[3] and not inside_ignore_list:
        return curr_scenario
    else:
        return best_scenario


def make_a_plan(robots, tasks):
    robots = copy.deepcopy(robots)
    tasks = copy.deepcopy(tasks)
    t = 0
    scenarios = []
    robots_planning = {}
    for robot in robots:
        robots_planning[robot.name] = []
    while tasks != []:
        for robot in robots:
            if robots_planning[robot.name] == [] or robots_planning[robot.name][-1][1] <= t:
                for task in tasks:
                    effort = task.total_effort(robot)
                    reward = task.total_reward()
                    scenarios.append((robot, task, effort, reward, reward / effort))
        while scenarios != []:
            max_benefit = reduce(find_best_scenario, scenarios)
            robots_planning[max_benefit[0].name] += [(max_benefit[1].total_movement_effort(max_benefit[0]),
                                                      t + math.ceil(max_benefit[2]))]
            max_benefit[0].position = robots_planning[max_benefit[0].name][-1][0].tasks[-1].position
            tasks.remove(max_benefit[1])
            scenarios = list(filter(lambda x: x[0] is not max_benefit[0] and x[1] is not max_benefit[1], scenarios))
        next_times = [robots_planning[key][-1][1] for key in robots_planning]
        t = min(next_times)
    return robots_planning


def make_a_plan2(robots, tasks):
    robots = copy.deepcopy(robots)
    tasks = copy.deepcopy(tasks)
    t = 0
    scenarios = []
    robot_finder = {}
    task_finder = {}
    robots_planning = {}
    for robot in robots:
        robots_planning[robot.name] = []
        robot_finder[robot.name] = robot
    for task in tasks:
        task_finder[task.name] = task

    while tasks != []:
        separated_scenarios = [[(robot, task, task.total_effort(robot), task.total_reward()/task.total_effort(robot))
                                for task in tasks]
                               for robot in robots
                               if robots_planning[robot.name] == [] or robots_planning[robot.name][-1][2] <= t]
        scenarios = list(reduce(lambda x, y: x+y, separated_scenarios, []))
        ignore_list = []
        momentary_plan = {}
        last_added = []
        saved_scenario = []
        i = 0
        while scenarios != [] and i < 10:
            max_benefit = reduce(lambda x, y: find_best_scenario2(x, y, ignore_list), scenarios)
            if max_benefit[3] != 0:
                momentary_plan[max_benefit[0].name] = (max_benefit[1].total_movement_effort(max_benefit[0]),
                                                       max_benefit[1],
                                                       t + math.ceil(max_benefit[2]))
                saved_scenario += [copy.deepcopy(scenarios)]
                last_added += [max_benefit]
                scenarios = list(filter(lambda x: x[0] is not max_benefit[0] and x[1] is not max_benefit[1], scenarios))
                ignore_list = []
            else:
                momentary_plan[last_added[-1][0].name] = None
                scenarios = saved_scenario.pop(-1)
                ignore_list += [last_added.pop(-1)]
            i += 1
        for robot_name in momentary_plan:
            robots_planning[robot_name].append(momentary_plan[robot_name])
            robot_finder[robot_name].position = momentary_plan[robot_name][0].tasks[-1].position
            tasks.remove(task_finder[momentary_plan[robot_name][1].name])

        next_times = [robots_planning[key][-1][2] for key in robots_planning]
        t = min(next_times)
    return robots_planning


if __name__ == '__main__':
    # mu_tasks = [MuTask('0',  MuTaskType.t1, [8,   9]),
    #             MuTask('1',  MuTaskType.t1, [1,   7]),
    #             MuTask('2',  MuTaskType.t1, [-4, -6]),
    #             MuTask('3',  MuTaskType.t1, [-2,  8]),
    #             MuTask('4',  MuTaskType.t1, [0,   9]),
    #             MuTask('5',  MuTaskType.t1, [2,   8]),
    #             MuTask('6',  MuTaskType.t2, [9,   3]),
    #             MuTask('7',  MuTaskType.t1, [-9,  3]),
    #             MuTask('8',  MuTaskType.t1, [0,  -2]),
    #             MuTask('9',  MuTaskType.t3, [4,  -3]),
    #             MuTask('10', MuTaskType.t3, [-8, -1]),
    #             MuTask('11', MuTaskType.t3, [3,   2]),
    #             MuTask('12', MuTaskType.t3, [-5,  0]),
    #             MuTask('13', MuTaskType.t3, [0,  -6]),
    #             MuTask('14', MuTaskType.t3, [7,   1]),
    #             MuTask('15', MuTaskType.t3, [0,   3]),
    #             MuTask('16', MuTaskType.t3, [0,   4]),
    #             MuTask('17', MuTaskType.t3, [8,  -5])]
    # mu_tasks[6].add_tasks_as_previous([mu_tasks[2], mu_tasks[5]])
    # mu_tasks[2].add_tasks_as_previous([mu_tasks[0], mu_tasks[1]])
    # mu_tasks[5].add_tasks_as_previous([mu_tasks[3], mu_tasks[4]])
    # mu_tasks[8].add_tasks_as_previous([mu_tasks[7]])
    # mu_tasks[11].add_tasks_as_previous([mu_tasks[9], mu_tasks[10]])
    # mu_tasks[17].add_tasks_as_previous([mu_tasks[15], mu_tasks[16]])
    # mu_tasks[15].add_tasks_as_previous([mu_tasks[14]])

    # for mu_task in mu_tasks:
    #     print(mu_task, mu_task.get_all_pre_requisites(), mu_task.position, connection_probability(mu_task.position))

    # tasks = [Task('0', [mu_tasks[6]], 3, 180),
    #          Task('1', [mu_tasks[8]], 3, 60),
    #          Task('2', [mu_tasks[11]], 3, 300),
    #          Task('3', [mu_tasks[12], mu_tasks[13]], 3, 60),
    #          Task('4', [mu_tasks[17]], 3, 120)]

    robots = [Robot('Alfred', RobotType.a1, [0, 0]),
              Robot('Dick', RobotType.a2, [2, 2]),
              Robot('Rachel', RobotType.a3, [-5, -4])]

    # print(make_a_plan(robots, tasks))

    # mu_tasks = [MuTask('0', MuTaskType.t1, [2,   6]),
    #             MuTask('1', MuTaskType.t1, [-2, -8]),
    #             MuTask('2', MuTaskType.t1, [-4, -6]),
    #             MuTask('3', MuTaskType.t1, [-5, 6]),
    #             MuTask('4', MuTaskType.t1, [-3, 2]),
    #             MuTask('5', MuTaskType.t1, [-3, 7]),
    #             MuTask('6', MuTaskType.t2, [11, -6]),
    #             MuTask('7', MuTaskType.t1, [7, 6]),
    #             MuTask('8', MuTaskType.t1, [2, 0]),
    #             MuTask('9', MuTaskType.t3, [-2, -5]),
    #             MuTask('10', MuTaskType.t3, [-8, -1]),
    #             MuTask('11', MuTaskType.t3, [3, 2]),
    #             MuTask('12', MuTaskType.t3, [-5, 0]),
    #             MuTask('13', MuTaskType.t3, [-6, -8]),
    #             MuTask('14', MuTaskType.t3, [7, 1]),
    #             MuTask('15', MuTaskType.t3, [-8, -7]),
    #             MuTask('16', MuTaskType.t3, [8, -7]),
    #             MuTask('17', MuTaskType.t3, [8, -5])]
    # # T0
    # mu_tasks[2].add_tasks_as_previous([mu_tasks[0]])
    # # T1
    # mu_tasks[4].add_tasks_as_previous([mu_tasks[3]])
    # mu_tasks[5].add_tasks_as_previous([mu_tasks[4]])
    # # T2
    # mu_tasks[8].add_tasks_as_previous([mu_tasks[6]])
    # mu_tasks[9].add_tasks_as_previous([mu_tasks[7]])
    # mu_tasks[10].add_tasks_as_previous([mu_tasks[8], mu_tasks[9]])
    # # T3
    # mu_tasks[12].add_tasks_as_previous([mu_tasks[11]])
    # # T4
    # mu_tasks[14].add_tasks_as_previous([mu_tasks[13]])
    # mu_tasks[16].add_tasks_as_previous([mu_tasks[15]])
    #
    # tasks = [Task('0', [mu_tasks[1], mu_tasks[2]], 3, 180),
    #          Task('1', [mu_tasks[5]], 3, 60),
    #          Task('2', [mu_tasks[10]], 3, 300),
    #          Task('3', [mu_tasks[12]], 3, 60),
    #          Task('4', [mu_tasks[14], mu_tasks[16], mu_tasks[17]], 3, 120)]

    mu_tasks = [MuTask('0', MuTaskType.t1, [2,   6]),
                MuTask('1', MuTaskType.t2, [-2, -8]),
                MuTask('2', MuTaskType.t3, [-4, -6])]

    tasks = [Task('0', [mu_tasks[0]], 3, 180),
             Task('1', [mu_tasks[1]], 3, 60),
             Task('2', [mu_tasks[2]], 3, 300)]

    # separated_scenarios = [[(robot, task, task.total_effort(robot), task.total_reward() / task.total_effort(robot))
    #                         for task in tasks]
    #                        for robot in robots]
    # for robot_scenario in separated_scenarios:
    #     print(robot_scenario)

    print(make_a_plan2(robots, tasks))

    # print('Executing at 0 ---------------------------')
    # print(tasks[4].total_movement_effort(robots[0]), 'until 35')
    # print(tasks[0].total_movement_effort(robots[1]), 'until 63')
    # print(tasks[3].total_movement_effort(robots[2]), 'until 16')
    # print('Planning at 16---------------------------')
    # robots[2].position = tasks[3].total_movement_effort(robots[2]).tasks[-1].position
    # tasks = [tasks[1], tasks[2]]
    # for robot in robots:
    #     for task in tasks:
    #         effort = task.total_effort(robot)
    #         reward = task.total_reward()
    #         print(robot.name, task, effort, reward, reward/effort, task.deadline - effort)
    # print('Executing at 16 ---------------------------')
    # print(tasks[1].total_movement_effort(robots[2]), 'until 49')
    # print('Planning at 35---------------------------')
    # robots[0].position = tasks[1].total_movement_effort(robots[0]).tasks[-1].position
    # tasks = [tasks[0]]
    # for robot in robots:
    #     for task in tasks:
    #         effort = task.total_effort(robot)
    #         reward = task.total_reward()
    #         print(robot.name, task, effort, reward, reward/effort, task.deadline - effort)
    # print('Executing at 35 ---------------------------')
    # print(tasks[0].total_movement_effort(robots[0]), 'until 60')