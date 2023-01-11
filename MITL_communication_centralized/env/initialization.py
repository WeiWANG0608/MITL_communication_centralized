"""
Author Wei Wang (wei7@kth.se)

This file is used to initialize the environment and agents
- initial_env(): initialize environment with obs_percent% of obstacles.
- initial_work_region(): initialize work region for each heavy-duty robot
- initial_agents(): initialize agents positions randomly
- initial_specification(): initialize task specifications randomly
- get_tasks(): used to distinguish own, collaboration and meet tasks
- make_world(): objectivize each agent as an object
"""


import random
import copy
import time

import numpy as np
from core import World, Agent_heavy, Agent_light


def initial_env(rows, cols, obs_percent):
    num_obs = round(rows * cols * obs_percent / 100)
    obs_list = []
    i = 0
    while i < num_obs:
        ob = [random.randint(0, rows - 1), random.randint(0, cols - 1)]
        if ob not in obs_list:
            obs_list.append(ob)
            i += 1
    return obs_list


def initial_work_region(num_heavy, rows, cols):  #2, 3, 2
    n_col = int(num_heavy / 2)
    work_region = []
    for i in range(n_col):
        work_region.append([0, rows - 1, i * cols, (i + 1) * cols - 1])
    for i in range(n_col, 0, -1):
        work_region.append([rows, 2 * rows - 1, (i - 1) * cols, i * cols - 1])
    return work_region


def initial_agents(regions, obs, num_heavy, num_light):
    heavy_list = []
    light_list = []
    i = 0
    while i < num_heavy:
        h_pos = [random.randint(regions[i][0], regions[i][1] - 1),
                 random.randint(regions[i][2], regions[i][3] - 1)]
        if h_pos not in obs and h_pos not in heavy_list:
            heavy_list.append(h_pos)
            i += 1
    j = 0
    while j < num_light:
        l_pos = [random.randint(regions[0][0], regions[0][1] - 1),
                 random.randint(regions[0][2], regions[0][3] - 1)]
        if l_pos not in obs and l_pos not in light_list and l_pos not in heavy_list:
            light_list.append(l_pos)
            j += 1
    return heavy_list, light_list


def initial_specification(obs, agents_h, agents_l, work_regions, states):
    """
    MITL Specifications varphi_i
        - Independent tasks (varphi_i^{own}): ['operator', 'starting time', 'ending time', 'cell']
        - Collaboration tasks (varphi_j^{collab}): ['operator', 'starting time', 'ending time', 'cell', 'Rj', 'col']
        - Meet tasks (varphi_j^{meet}): ['operator', 'starting time', 'ending time', 'cell', 'Rj', 'meet']
    """
    num = len(agents_h)
    operators = ['G', 'E']
    types = ['', '', '', '', '', '', 'col', 'meet']
    recipients = [i for i in range(num)]
    specs = [[] for i in range(num)]
    for i in range(num):
        num_task = random.randint(1,1)
        n = 0
        num_meet = 0
        cells = []
        while n < num_task:
            pos = [random.randint(work_regions[i][0], work_regions[i][1] - 1),
                   random.randint(work_regions[i][2], work_regions[i][3] - 1)]
            if pos not in obs and pos not in agents_h and pos not in agents_l:
                cells.append(pos)
                n += 1
        up_t = 5
        for j in range(num_task):
            task = []
            operator = random.choice(operators)
            task.append(operator)
            start = random.randint(up_t, (j + 1) * 40)
            task.append(str(start))
            end = random.randint(start + 1, start + 10)
            task.append(str(end))
            up_t = end
            for key, value in states[i].items():
                if cells[j][0] == value[0] and cells[j][1] == value[1]:
                    task.append(key)
            if num_meet > 0:
                type = random.choice([x for x in types if x != 'meet'])
            else:
                type = random.choice(types)
                if type == 'meet':
                    num_meet = 1
            if type != '':
                recipient = random.choice([x for x in recipients if x != i])
                task.append('R' + str(recipient))
                task.append(type)
            specs[i].append(task)
    return specs


def print_specs(specs):
    spec_print = copy.deepcopy(specs)
    for i in range(len(spec_print)):
        for j in range(len(spec_print[i])):
            if len(spec_print[i][j]) > 4:
                spec_print[i][j][4] = 'R'+str(int(spec_print[i][j][4][1:len(spec_print[i][j][4])])+1)
    return spec_print


def get_tasks(specs, states_work_region):
    tasks = [[] for i in range(3)]
    for i in range(len(specs)):
        for j in range(len(specs[i])):
            if len(specs[i][j]) > 4:
                if specs[i][j][5] == 'col':
                    tasks[1].append(states_work_region[i][specs[i][j][3]])
                if specs[i][j][5] == 'meet':
                    tasks[2].append(states_work_region[i][specs[i][j][3]])
            else:
                tasks[0].append(states_work_region[i][specs[i][j][3]])
    return tasks


def get_work_map(map_ori, map_list, work_region):  # [6, 2], [[0,2,0,1], [3,5,0,1]] [0 or 1]
    map_row, map_col = map_ori.shape  # 6, 2
    new_map = np.ones((map_row, map_col), dtype=int)
    for pi in range(map_list[work_region][0], map_list[work_region][1] + 1):
        for pj in range(map_list[work_region][2], map_list[work_region][3] + 1):
            new_map[pi, pj] = map_ori[pi, pj]
    return new_map


def get_global_specification(specs, states_ori, states_work_region, req=False):
    for i in range(len(specs)):
        for j in range(len(specs[i])):
            if len(specs[i][j]) > 0:
                if not req:
                    current_pos = states_work_region[i][specs[i][j][3]]
                    # get the state in the global map
                    for key, value in states_ori.items():
                        if current_pos[0] == value[0] and current_pos[1] == value[1]:
                            current_state = key
                            specs[i][j][3] = current_state
                else:
                    for k in range(len(specs[i][j])):
                        current_pos = states_work_region[i][specs[i][j][k][3]]
                        # get the state in the global map
                        for key, value in states_ori.items():
                            if current_pos[0] == value[0] and current_pos[1] == value[1]:
                                current_state = key
                                specs[i][j][k][3] = current_state
    return specs


def make_world(agents_h, agents_l, r_sens_h, r_sens_l, v_h, v_l, tasks, requests):
    num = len(agents_h)
    agents_h = [np.array(item) for item in agents_h]
    world = World()

    world.agents_heavy = [Agent_heavy() for i in range(num)]
    for i, agent_heavy in enumerate(world.agents_heavy):
        agent_heavy.name = 'H%d' % i
        agent_heavy.p_pos = agents_h[i]
        agent_heavy.r_sens = r_sens_h
        agent_heavy.velocity = v_h
        agent_heavy.path.append(np.array(agents_h[i]))
        agent_heavy.task_left = tasks[i]
        agent_heavy.working_region = i
        agent_heavy.request_send = requests[i]

    world.agents_light = Agent_light()
    world.agents_light.name = 'L%d' % 1
    world.agents_light.p_pos = agents_l
    world.agents_light.r_sens = r_sens_l
    world.agents_light.velocity = v_l
    world.agents_light.path.append(np.array(agents_l))
    world.agents_light.num_round = 0
    world.agents_light.request_list = [[] for i in range(num)]
    return world


def get_milli_time():
    return round(time.time()*1000)


def initial_specification_time(obs, agents_h, agents_l, work_regions, states, mm):
    """
    MITL Specifications varphi_i
        - Independent tasks (varphi_i^{own}): ['operator', 'starting time', 'ending time', 'cell']
        - Collaboration tasks (varphi_j^{collab}): ['operator', 'starting time', 'ending time', 'cell', 'Rj', 'col']
        - Meet tasks (varphi_j^{meet}): ['operator', 'starting time', 'ending time', 'cell', 'Rj', 'meet']
    """
    num = len(agents_h)
    operators = ['G', 'E']
    recipients = [i for i in range(num)]
    specs = [[] for i in range(num)]
    task_list = []
    num_meet = 0
    if mm < 3:
        num_task = 1
        if num > 2:
            types = ['' for i in range(3)] + ['meet']
        else:
            types = ['', 'meet']
    elif mm < 6:
        num_task = 2
        if num > 2:
            types = ['' for i in range(7)] + ['meet']
        else:
            types = ['' for i in range(3)] + ['meet']
    else:
        num_task = 2
        if num > 2:
            types = ['' for i in range(7)] + ['meet']
        else:
            types = ['' for i in range(3)] + ['meet']
    for i in range(num):
        n = 0
        cells = []
        while n < num_task:
            pos = [random.randint(work_regions[i][0], work_regions[i][1] - 1),
                   random.randint(work_regions[i][2], work_regions[i][3] - 1)]
            if pos not in obs and pos not in agents_h and pos not in agents_l and pos not in task_list:
                cells.append(pos)
                task_list.append(pos)
                n += 1
        up_t = 3
        for j in range(num_task):
            task = []
            operator = random.choice(operators)
            task.append(operator)
            start = random.randint(up_t, up_t + 15)
            task.append(str(start))
            end = random.randint(start + 1, start + 5)
            task.append(str(end))
            up_t = end
            ty = ''
            for key, value in states[i].items():
                if cells[j][0] == value[0] and cells[j][1] == value[1]:
                    task.append(key)
            if num_meet > 0:
                ty = random.choice([x for x in types if x != 'meet'])
                types.remove(ty)
            else:
                if j > 0:
                    ty = random.choice(types)
                    types.remove(ty)
                else:
                    if num_task > 1:
                        ty = random.choice([x for x in types if x != 'meet'])
                    else:
                        ty = random.choice(types)
                    types.remove(ty)
                if ty == 'meet':
                    task[0] = 'G'
                    num_meet = 1
            if ty != '':
                recipient = random.choice([x for x in recipients if x != i])
                task.append('R' + str(recipient))
                task.append(ty)
            specs[i].append(task)
    return specs


def initial_specification_rate(obs, agents_h, agents_l, work_regions, states, mm):
    """
    MITL Specifications varphi_i
        - Independent tasks (varphi_i^{own}): ['operator', 'starting time', 'ending time', 'cell']
        - Collaboration tasks (varphi_j^{collab}): ['operator', 'starting time', 'ending time', 'cell', 'Rj', 'col']
        - Meet tasks (varphi_j^{meet}): ['operator', 'starting time', 'ending time', 'cell', 'Rj', 'meet']
    """
    num = len(agents_h)
    operators = ['G', 'E']
    recipients = [i for i in range(num)]
    specs = [[] for i in range(num)]
    task_list = []
    num_meet = 0
    deliver_dif = random.randint(0, 5) + 2

    if mm < 3:
        num_task = 1
        if num > 2:
            types = ['' for i in range(3)] + ['meet']
        else:
            types = ['', 'meet']
    elif mm < 6:
        num_task = 2
        if num > 2:
            types = ['' for i in range(7)] + ['meet']
        else:
            types = ['' for i in range(3)] + ['meet']
    else:
        num_task = 2
        if num > 2:
            types = ['' for i in range(7)] + ['meet']
        else:
            types = ['' for i in range(3)] + ['meet']
    for i in range(num):
        n = 0
        cells = []
        while n < num_task:
            pos = [random.randint(work_regions[i][0], work_regions[i][1] - 1),
                   random.randint(work_regions[i][2], work_regions[i][3] - 1)]
            if pos not in obs and pos not in agents_h and pos not in agents_l and pos not in task_list:
                cells.append(pos)
                task_list.append(pos)
                n += 1
        up_t = 2
        for j in range(num_task):
            # [0]: operator, [1]: start_time, [2]: end_time
            # [3]: key,      [4]:receiver,    [5]: type
            task = ['' for i in range(6)]
            ty = types[0]
            types.remove(ty)
            if ty == '':
                operator = random.choice(operators)
                task[0] = operator
                if i == 0:
                    if j == 0:
                        start = random.randint(2, 14)
                        task[1] = str(start)
                    else:
                        start = random.randint(up_t+2, 20)
                        task[1] = str(start)
                else:
                    start = random.randint(deliver_dif + 2, 18)
                    task[1] = str(start)
                end = random.randint(start + 1, start + 3)
                task[2] = str(end)
                up_t = end
                for key, value in states[i].items():
                    if cells[j][0] == value[0] and cells[j][1] == value[1]:
                        task[3] = key
                specs[i].append(task[0:4])
            if ty != '':
                task[0] = 'G'
                start = deliver_dif
                task[1] = str(start)
                end = random.randint(start + 1, start + 3)
                task[2] = str(end)
                # up_t = end
                for key, value in states[i].items():
                    if cells[j][0] == value[0] and cells[j][1] == value[1]:
                        task[3] = key
                recipient = random.choice([x for x in recipients if x != i])
                task[4] = 'R' + str(recipient)
                task[5] = ty
                specs[i].append(task)
    return specs


def initial_specification_compare(obs, agents_h, agents_l, work_regions, states, mm):
    """
    MITL Specifications varphi_i
        - Independent tasks (varphi_i^{own}): ['operator', 'starting time', 'ending time', 'cell']
        - Collaboration tasks (varphi_j^{collab}): ['operator', 'starting time', 'ending time', 'cell', 'Rj', 'col']
        - Meet tasks (varphi_j^{meet}): ['operator', 'starting time', 'ending time', 'cell', 'Rj', 'meet']
    """
    num = len(agents_h)
    operators = ['G', 'E']
    recipients = [i for i in range(num)]
    specs = [[] for i in range(num)]
    task_list = []
    num_meet = 0
    num_task = 2
    types = ['' for i in range(num * num_task-1)] + ['meet']

    for i in range(num):
        n = 0
        cells = []
        while n < num_task:
            pos = [random.randint(work_regions[i][0], work_regions[i][1] - 1),
                   random.randint(work_regions[i][2], work_regions[i][3] - 1)]
            if pos not in obs and pos not in agents_h and pos not in agents_l and pos not in task_list:
                cells.append(pos)
                task_list.append(pos)
                n += 1
        up_t = 1
        for j in range(num_task):
            task = []
            operator = random.choice(operators)
            task.append(operator)
            start = random.randint(up_t+1, up_t + 4)
            task.append(str(start))
            end = random.randint(start + 1, start + 2)
            task.append(str(end))
            up_t = end
            ty = ''
            for key, value in states[i].items():
                if cells[j][0] == value[0] and cells[j][1] == value[1]:
                    task.append(key)
            if num_meet > 0:
                ty = random.choice([x for x in types if x != 'meet'])
                types.remove(ty)
            else:
                ty = random.choice(types)
                types.remove(ty)
                if ty == 'meet':
                    task[0] = 'G'
                    if j > 0:
                        tt = random.randint(1, 3)
                        task[1] = str(int(task[1])+tt)
                        task[2] = str(int(task[2]) + tt)
                    num_meet = 1
            if ty != '':
                recipient = random.choice([x for x in recipients if x != i])
                task.append('R' + str(recipient))
                task.append(ty)
            specs[i].append(task)
    return specs
