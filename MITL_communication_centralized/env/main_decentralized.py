import math
from datetime import datetime
import numpy as np
import copy

from initialization import make_world
from visualization import draw_path
from heavy_path_generation_decentralized import heavy_path_decentralized, \
    get_independent_requests_tasks, get_path_cell2coord_decentralized
from request_exchange import collect_deliver, complete_path


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    UNDERLINE = '\033[4m'


def decentralized(map_original, map_work_region, map_obs_work, work_region, states_work_region, specifications,
                  agents_heavy, agents_light, file_folder, fig_folder, region_row, region_col):
    num_agent_heavy = len(agents_heavy)
    """
    --------------- light-duty robot module ---------------
    """
    # initial the parameters of agents
    r_i = 1

    if len(map_original) < 11:
        r_0 = 1
        # hatch distance
        h_a = 2 * r_0 - 1
    else:
        r_0 = 2
        # hatch distance
        h_a = 2 * r_0 - 2
    # velocity / transition weight constraints
    v_heavy = 1
    v_light = max([math.ceil(
        ((2 * (work_region[i][3] - work_region[i][2]) + h_a) * v_heavy) /
        ((2 * r_0) - h_a)) for i in range(num_agent_heavy)])

    """
    --------------- Mission decomposition ---------------
    """

    path_existence = [[] for i in range(num_agent_heavy)]
    initial_path = [[] for i in range(num_agent_heavy)]
    independent_path = [[] for i in range(num_agent_heavy)]
    independent_task = [[] for i in range(num_agent_heavy)]
    independent_request = [[] for i in range(num_agent_heavy)]
    independent_path_2d = [[] for i in range(num_agent_heavy)]

    for i in range(num_agent_heavy):
        flag = "R" + str(i)
        # check if the initial specifications are satisfied
        initial_path[i] = heavy_path_decentralized(map_work_region[i], specifications[i], file_folder, flag,
                                                   region_row, region_col, agents_heavy[i]).get_path()
        # check if robot need to cooperate and provide best promise.  (bool) coop, promise
        if len(initial_path[i]) == 0:
            path_existence[i] = False
            print(path_existence[i])
            print("========There is no path satisfying the specification!========")
        else:
            path_existence[i] = True
    if all(item is True for item in path_existence):
        for i in range(num_agent_heavy):
            independent_task, independent_request = get_independent_requests_tasks(specifications, map_work_region,
                                                                                   agents_heavy, file_folder,
                                                                                   region_row, region_col)
            independent_path[i] = heavy_path_decentralized(map_work_region[i], independent_task[i], file_folder, flag,
                                                           region_row, region_col, agents_heavy[i]).get_path()
            independent_path_2d[i] = get_path_cell2coord_decentralized(independent_path[i], states_work_region[i])

        print(f"{bcolors.HEADER}======== ***** The Pre-computation Result ****** ========")
        print(f"{bcolors.HEADER}------- The Original Task Specification -------")
        print(specifications)
        print(f"{bcolors.HEADER}------------ If all the paths exist ------------")
        print(path_existence)
        print(f"{bcolors.HEADER}---- The Independent Task Specification ----")
        print(independent_task)
        print(f"{bcolors.HEADER}--------- The Independent Request ---------")
        print(independent_request)
        # print(f"{bcolors.HEADER}-------The Path Under The Specification--------")
        # print(independent_path)

        # # save all the info of Mission Decomposition
        # info = ''
        # for i in range(num_agent_heavy):
        #     info = info + str(agents_heavy[i]) + ';' + ','.join(str(j) for j in specifications[i]) + ';' + ','.join(
        #         str(j) for j in independent_task[i]) + ';' + ','.join(str(j) for j in independent_request[i]) + '&'
        # text_file = open(file_folder + "/" + 'initialization_info' + ".txt", "w")
        # text_file.write(info)
        # text_file.close()

        # make the world (core.py)
        world = make_world(agents_heavy, agents_light, r_i, r_0, v_heavy, v_light, independent_task,
                           independent_request)

        """
        Start two round SWEEP
        """
        # start sweeping
        # maximum 2 rounds to guarantee the requests exchange

        num_round_exchange = 2

        no_updated_path, path_detect_round, independent_path_2d, record_working_region, record_task_left, deliver_t = \
            collect_deliver(world, map_obs_work, num_round_exchange, num_agent_heavy, work_region,
                            states_work_region, independent_path_2d, h_a, r_0, v_light, file_folder)

        if no_updated_path:
            print("No path exist for decentralized approach")
            return no_updated_path, deliver_t

        else:
            print("Path exist for decentralized approach")
            # """
            # Path of agile light-duty robot
            # """
            # path_detect_round[1] = path_detect_round[1][len(path_detect_round[0]): len(path_detect_round[1])]
            # for n in range(len(path_detect_round)):
            #     mm = copy.deepcopy(map_original)
            #     draw_path(mm, path_detect_round[n], fig_folder, "Trajectory of R0 Round " + str(n + 1))
            #
            # """
            # Get the data ready for plot and video
            # """
            #
            # independent_path_2d, record_working_region, record_task_left = \
            #     complete_path(world, v_light, independent_path_2d, states_work_region, record_working_region,
            #                   record_task_left)
            #
            # record_path_all = world.agents_light.path
            # for i in range(num_agent_heavy):
            #     record_path_all = np.concatenate((record_path_all, world.agents_heavy[i].path), axis=1)

            return no_updated_path, deliver_t
