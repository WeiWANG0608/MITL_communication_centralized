import time
from datetime import datetime
import numpy as np
import copy
import os
import random
from initialization import get_tasks, get_work_map, initial_env, initial_agents, \
    initial_work_region, initial_specification_time, print_specs, get_milli_time, initial_specification_rate
from visualization import draw_map
from heavy_WTS_centralized import total_states_centralized
from main_centralized import centralized
from main_decentralized import decentralized
import statistics


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    UNDERLINE = '\033[4m'


if __name__ == '__main__':
    dirname = os.path.dirname
    ABPATH = os.path.join(dirname(dirname(__file__)))

    """create the folder for the fig, file, output(video)"""
    now = datetime.today().strftime('%Y-%m-%d')
    fig_folder = ABPATH + '/figures/' + str(now)
    file_folder = ABPATH + '/files/' + str(now)
    output_folder = ABPATH + '/files/' + str(now) + '/output'
    if not (os.path.exists(fig_folder)):
        os.makedirs(fig_folder)
    if not (os.path.exists(file_folder)):
        os.makedirs(file_folder)
    if not (os.path.exists(output_folder)):
        os.makedirs(output_folder)

    """
    Initial the environment
    Get the number robots: num_agent_heavy, num_agent_light
    Get the size of the work region: region_row, region_col
    Get the whole map with obstacles: 2*(N/2) work regions, obs_percentage, obs_list
    Get the map and state space of the env: map_original, map_obstacle, states_original
    Get the map and state space of work regions for each heavy-duty robot: work_region, map_work_region, states_work_region
    Get initial position of all robots: agents_heavy, agents_light
    """
    region_list = [[3, 2], [3, 3], [4, 3], [4, 4], [5, 5], [10, 10]]
    region_row, region_col = region_list[5]

    num_exp = 0
    duration = [[] for i in range(2)]
    average_duration = [[] for i in range(2)]
    # generate_round1 + "," +  synthesis_round1 + "," + generate_round2 + "," + synthesis_round2 + "," + if_path
    # + "," + total_time (s)+ "\n"
    f = open(file_folder + "/output/" + 'Time_cen_' + str(region_row) + "*" + str(region_col) + ".txt", "w")
    pass
    f.close()

    # generate_R1 + "," +  synthesis_R1 + "," + generate_R2 + "," + synthesis_R2 + "," + if_path
    # + "," + total_time (s)+ "\n"
    f = open(file_folder + "/output/" + 'Time_de_' + str(region_row) + "*" + str(region_col) + ".txt", "w")
    pass
    f.close()
    while num_exp < 20:
        print(f"{bcolors.OKGREEN}--------- No.", num_exp, " 2 Agents ---------")
        num_agent_heavy = random.choice([2])
        num_agent_light = 1
        # region_row, region_col = [2, 3]
        mm = min(region_row, region_col)
        map_row, map_col = [region_row * 2, int(region_col * (num_agent_heavy / 2))]
        obs_percentage = 0
        obs_list = initial_env(region_row * 2, region_col * (num_agent_heavy / 2), obs_percentage)
        map_original = np.zeros((map_row, map_col), dtype=int)
        map_obstacle = copy.deepcopy(map_original)
        for i in range(len(obs_list)):
            map_obstacle[obs_list[i][0]][obs_list[i][1]] = 1
        cell_dict_ori = total_states_centralized(map_obstacle)
        work_region = initial_work_region(num_agent_heavy, region_row, region_col)
        map_work_region = [get_work_map(map_obstacle, work_region, i) for i in range(num_agent_heavy)]
        cell_work_region = [total_states_centralized(map_work_region[i]) for i in range(num_agent_heavy)]

        agents_heavy, [agents_light] = initial_agents(work_region, obs_list, num_agent_heavy, num_agent_light)
        print("Init heavy_duty agents: ", agents_heavy, ". Init light_duty agent: ", agents_light)

        # get the MITL task specification
        specifications = initial_specification_time(obs_list, agents_heavy, agents_light, work_region, cell_work_region, mm)
        specification_print = print_specs(specifications)
        for i in range(num_agent_heavy):
            print("MITL Specification of R", i + 1, ": ", specification_print[i])

        task_type = get_tasks(specifications, cell_work_region)
        map_obs_work = copy.deepcopy(map_obstacle)
        # draw_map(map_obs_work, task_type, num_agent_heavy, fig_folder)
        specifications_centralized = copy.deepcopy(specifications)

        duration_centralized = 0
        if mm < 4:
            print(f"{bcolors.HEADER}--------- Start centralized approach ---------")
            print(f"{bcolors.ENDC}================================================")
            start_centralized = get_milli_time()
            solution_not_exist_cen = centralized(specifications_centralized, cell_dict_ori, cell_work_region, map_obs_work,
                                                 agents_heavy, file_folder, fig_folder, region_row, region_col)

            if not solution_not_exist_cen:
                duration_centralized = (get_milli_time()-start_centralized)/1000

                f = open(file_folder + "/output/" + 'Time_cen_' + str(region_row) + "*" + str(region_col) + ".txt", "a")  # append mode
                f.write(str(not solution_not_exist_cen) + ",")
                f.write(str(duration_centralized) + "\n")
                f.close()

            else:
                duration_centralized = (get_milli_time() - start_centralized) / 1000

                f = open(file_folder + "/output/" + 'Time_cen_' + str(region_row) + "*" + str(region_col) + ".txt",
                         "a")  # append mode
                f.write(str(not solution_not_exist_cen) + ",")
                f.write(str(duration_centralized) + "\n")
                f.close()

                continue
        else:
            solution_not_exist_cen = False
            print(f"{bcolors.OKBLUE}--------- Centralized approach stop running (scale) ---------")
            duration_centralized = -1
            f = open(file_folder + "/output/" + 'Time_cen_' + str(region_row) + "*" + str(region_col) + ".txt",
                     "a")  # append mode
            f.write(str(False) + ",")
            f.write(str(duration_centralized) + "\n")
            f.close()

        print(f"{bcolors.HEADER}--------- Start decentralized approach ---------")
        print(f"{bcolors.ENDC}================================================")
        start_decentralized = get_milli_time()
        solution_not_exist_dec = decentralized(map_original, map_work_region, map_obs_work, work_region, cell_work_region,
                                               specifications, agents_heavy, agents_light, file_folder, fig_folder,
                                               region_row, region_col)
        print(f"{bcolors.OKBLUE}--------- There is a path ---------", not solution_not_exist_dec)
        duration_decentralized = 0
        if not solution_not_exist_dec:
            duration_decentralized = (get_milli_time() - start_decentralized)/1000

            f = open(file_folder + "/output/" + 'Time_de_' + str(region_row) + "*" + str(region_col) + ".txt",
                     "a")  # append mode
            f.write(str(not solution_not_exist_dec) + ",")
            f.write(str(duration_decentralized) + "\n")
            f.close()

        else:
            duration_decentralized = (get_milli_time() - start_decentralized) / 1000

            f = open(file_folder + "/output/" + 'Time_de_' + str(region_row) + "*" + str(region_col) + ".txt",
                     "a")  # append mode
            f.write(str(not solution_not_exist_dec) + ",")
            f.write(str(duration_decentralized) + "\n")
            f.close()

            # continue

        if not solution_not_exist_cen and not solution_not_exist_dec and duration_decentralized<100:
            duration[0].append(duration_centralized)
            duration[1].append(duration_decentralized)
            num_exp += 1

    average_duration = [sum(duration[i])/len(duration[i]) for i in range(len(duration))]
    std = [statistics.stdev(duration[i]) for i in range(len(duration))]

    with open(file_folder + "/output/" + 'Execution_Time_' + str(region_row) + "*" + str(region_col) + "_2.txt", "w") as f:
        for line in duration:
            f.write(f"{line}\n")
    f.close()

    with open(file_folder + "/output/" + 'Average_Time_' + str(region_row) + "*" + str(region_col) + "_2.txt", "w") as f:
        for line in average_duration:
            f.write(f"{line}\n")
        for line in std:
            f.write(f"{line}\n")

    f.close()

    with open(file_folder + "/output/" + 'Time_cen_' + str(region_row) + "*" + str(region_col) + ".txt") as oldfile, \
            open(file_folder + "/output/" + 'Time_cen_clean_' + str(region_row) + "*" + str(region_col) + ".txt",
                 'w') as newfile:
        for line in oldfile:
            if "False" not in line:
                newfile.write(line)

    if mm >= 4:
        with open(file_folder + "/output/" + 'Time_de_' + str(region_row) + "*" + str(region_col) + ".txt") as oldfile, \
                open(file_folder + "/output/" + 'Time_de_clean_' + str(region_row) + "*" + str(region_col) + ".txt",
                     'w') as newfile:
            for line in oldfile:
                if "False" not in line:
                    newfile.write(line)












