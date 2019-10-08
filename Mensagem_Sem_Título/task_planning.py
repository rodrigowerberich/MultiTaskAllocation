import math
from functools import reduce
import heapq
import copy

robot_types = ['a1', 'a2']
task_types = ['t0', 't1', 't2', 't3']
effort_function = {'a1': {'t0': 0, 't1': 1, 't2': 4, 't3': 7},
                   'a2': {'t0': 0, 't1': 3, 't2': math.inf, 't3': 1}}
reward_function = {'t1': 0,
                   't2': 7,
                   't3': 3}


def calculate_tasks_effort(effort, task_and_robot):
    task = task_and_robot[0]
    robot = task_and_robot[1]
    effort += task.effort(robot)
    if task.previous is not None:
        robot_n = [robot for _ in task.previous]
        previous_tasks_with_robot = list(zip(task.previous, robot_n))
        return reduce(calculate_tasks_effort, previous_tasks_with_robot, effort)
    return effort


class Task:
    def __init__(self, name, location, task_type, priority=3, previous=None):
        self.name = name
        self.location = location
        self.type = task_type
        self.priority = priority
        self.previous = previous

    def __str__(self):
        return 'Task ' + self.name
        # return 'Task: ('+str(self.location)+', '+str(self.type)+', '+str(self.priority)+', '+str(self.previous)+')'

    def __repr__(self):
        return str(self)

    def effort(self, robot):
        return effort_function[robot.type][self.type]

    def effort_all_previous(self, robot):
        if self.previous is not None:
            robot_n = [robot for _ in self.previous]
            previous_tasks_with_robot = list(zip(self.previous, robot_n))
            return reduce(calculate_tasks_effort, previous_tasks_with_robot, self.effort(robot))
        return self.effort(robot)


class Robot:
    def __init__(self, robot_type, location=[0, 0]):
        self.type = robot_type
        self.location = location

    def __str__(self):
        return 'Robot: ('+str(self.location)+', '+str(self.type)+')'

    def __repr__(self):
        return str(self)


def point_distance(x, y):
    return math.sqrt((math.pow(x[0] - y[0], 2) + (math.pow(x[1]-y[1], 2))))


def calculate_task_path_cost_rec(robot, location, task):
    distance = point_distance(location, task.location)
    if task.previous is not None:
        distance += calculate_task_path_cost_rec(task.location, task.previous[0])
    else:
        distance += point_distance(task.location, robot.location)
    return distance


def calculate_task_path_cost(robot, task):
    distance = 0
    if task.previous is not None:
        distance += calculate_task_path_cost_rec(robot, task.location, task.previous[0])
    else:
        distance = point_distance(task.location, robot.location)
    return distance


def get_tasks_previous(x, task):
    if task.previous is not None:
        previous = [e for e in task.previous]
        x += previous
    return x


class Location:
    def __init__(self, x=[0, 0], name=''):
        self.x = x
        self.previous = None
        self.name = name

    def __str__(self):
        if self.previous is None:
            return self.name
        else:
            return self.name

    def __repr__(self):
        return str(self)

    @staticmethod
    def from_task(task):
        location = Location()
        location.x = task.location
        location.name = task.name
        if task.previous is not None:
            location.previous = [Location.from_task(task_previous) for task_previous in task.previous]
        return location


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


def expand_open(open_list, robot):
    node_to_expand = heapq.heappop(open_list)
    expand_task = node_to_expand.tasks[-1]
    expand_candidates = []
    for task in node_to_expand.tasks:
        if task.previous is not None:
            for previous_task in task.previous:
                if previous_task not in node_to_expand.tasks:
                    if previous_task not in expand_candidates:
                        if previous_task.previous is not None:
                            expand_candidates += [previous_task]
    if len(expand_candidates) is 0:
        if expand_task.previous is not None:
            expand_candidates += [expand_task.previous[0]]
    # print('Expand candidates: '+str(expand_candidates))
    if len(expand_candidates) is not 0:
        for expand_candidate in expand_candidates:
            cost = point_distance(expand_candidate.location, expand_task.location)
            cost += expand_candidate.effort(robot)
            heapq.heappush(open_list, Node(node_to_expand.tasks+[expand_candidate], node_to_expand.cost+cost))
        return None
    return node_to_expand


def set_origin_task(location, origin):
    if location.previous is not None:
        for previous_location in location.previous:
            set_origin_task(previous_location, origin)
    else:
        location.previous = [origin]


def calculate_robot_task_cost(robot, task):
    task = copy.deepcopy(task)
    robot_location_task = Task('Robot', robot.location, 't0')
    set_origin_task(task, robot_location_task)
    open_list = [Node([task], task.effort(robot))]
    # print(open_list)
    result = None
    while result is None:
        result = expand_open(open_list, robot)
    return result


if __name__ == '__main__':
    tasks = [Task('T0', (8, 9), 't1'),
             Task('T1', (1, 7), 't1'),
             Task('T2', (-4, -6), 't1'),
             Task('T3', (-2, 8), 't1'),
             Task('T4', (0, 9), 't1'),
             Task('T5', (2, 8), 't1'),
             Task('T6', (9, 3), 't3'),
             Task('T7', (-9, 3), 't3'),
             Task('T8', (0, -2), 't3'),
             Task('T9', (4, -3), 't3'),
             Task('T10', (-8, -1), 't3'),
             Task('T11', (3, 2), 't3'),
             Task('T12', (-5, 0), 't3'),
             Task('T13', (0, -6), 't3'),
             Task('T14', (7, 1), 't3'),
             Task('T15', (0, 3), 't3'),
             Task('T16', (0, 4), 't3'),
             Task('T17', (8, -5), 't3')]
    tasks[6].previous = [tasks[2], tasks[5]]
    tasks[2].previous = [tasks[0], tasks[1]]
    tasks[5].previous = [tasks[3], tasks[4]]
    tasks[8].previous = [tasks[7]]
    tasks[11].previous = [tasks[9], tasks[10]]
    tasks[17].previous = [tasks[15], tasks[16]]
    tasks[15].previous = [tasks[14]]
    tasks_with_precedence = list(filter(lambda task: task.previous is not None, tasks))
    tasks_preceding = reduce(get_tasks_previous, tasks, [])
    tasks_without_precedence = list(filter(lambda task: task.previous is None, tasks))
    end_tasks = list(filter(lambda task_with_precedence: task_with_precedence not in tasks_preceding,
                            tasks_with_precedence))
    end_tasks += list(filter(lambda task_without_precedence: task_without_precedence not in tasks_preceding,
                            tasks_without_precedence))
    print(tasks_with_precedence)
    print(tasks_preceding)
    print(tasks_without_precedence)
    print(end_tasks)
    robots = [Robot(robot_types[0], [0, 0]), Robot(robot_types[0], [-5, -4]), Robot(robot_types[1], [4, 4])]
    costs = [[calculate_robot_task_cost(robot, task) for task in end_tasks] for robot in robots]

    robots[2].location = tasks[17].location
    r2_costs_t2 = [calculate_robot_task_cost(robots[2], task) for task in [tasks[8], tasks[12], tasks[13]]]

    robots[1].location = tasks[11].location
    r1_costs_t3 = [calculate_robot_task_cost(robots[1], task) for task in [tasks[12], tasks[13]]]

    robots[1].location = tasks[12].location
    r1_costs_t4 = [calculate_robot_task_cost(robots[1], task) for task in [tasks[13]]]

    print(costs)
    print(r2_costs_t2)
    print(r1_costs_t3)
    print(r1_costs_t4)

    robot_location_task = Task('Robot', robots[0].location, 't0')
    set_origin_task(tasks[12], robot_location_task)
    set_origin_task(tasks[13], robot_location_task)
    open_list = [Node([tasks[12], tasks[13]], tasks[13].effort(robots[0]))]
    # print(open_list)
    result = None
    while result is None:
        result = expand_open(open_list, robots[0])

    print(result)