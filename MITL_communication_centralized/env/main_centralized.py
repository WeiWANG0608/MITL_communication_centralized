from initialization import get_global_specification
from visualization import draw_path
from heavy_path_generation_centralized import heavy_path_centralized, get_independent_requests_tasks


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    UNDERLINE = '\033[4m'


def centralized(specifications, cell_dict_ori, cell_work_region, map_obs_work, agents_heavy, file_folder, fig_folder,
                region_row, region_col):
    no_path_exist = False

    num_agent = len(agents_heavy)

    # get the state in centralized map
    get_global_specification(specifications, cell_dict_ori, cell_work_region)
    print("Specifications (centralized): ", specifications)

    flag = 'Round1'
    initial_path = []
    initial_path = heavy_path_centralized(map_obs_work, specifications, file_folder, flag, agents_heavy,
                                          cell_dict_ori, region_row, region_col).get_path()

    if len(initial_path) > 1000 or len(initial_path) < 1:
        no_path_exist = True
        print(f"{bcolors.FAIL}--------- The Path DOES NOT Exists ---------")
        return no_path_exist

    else:
        independent_task, independent_request = get_independent_requests_tasks(specifications, map_obs_work,
                                                                               agents_heavy, file_folder, cell_dict_ori,
                                                                               region_row, region_col)

        print(f"{bcolors.HEADER}======== ***** The Pre-computation Result ****** ========")
        print(f"{bcolors.HEADER}------- The Original Task Specification -------")
        print(specifications)
        print(f"{bcolors.HEADER}---- The Independent Task Specification ----")
        print(independent_task)
        print(f"{bcolors.HEADER}--------- The Independent Request ---------")
        print(independent_request)

        # save all the info of Mission Decomposition
        info = ''
        for i in range(num_agent):
            info = info + str(agents_heavy[i]) + ';' + ','.join(str(j) for j in specifications[i]) + ';' + ','.join(
                str(j) for j in independent_task[i]) + ';' + ','.join(str(j) for j in independent_request[i]) + '&'
        text_file = open(file_folder + "/" + 'initialization_info' + ".txt", "w")
        text_file.write(info)
        text_file.close()

        """Request exchange"""
        for i in range(num_agent):
            for j in range(num_agent):
                if len(independent_request[i][j]):
                    for k in range(len(independent_request[i][j])):
                        independent_task[j].append(independent_request[i][j][k][0:4])
        print(f"{bcolors.HEADER}---- The Updated Independent Task Specification ----")
        print(independent_task)

        flag = 'Round2'
        updated_path = heavy_path_centralized(map_obs_work, independent_task, file_folder, flag, agents_heavy,
                                              cell_dict_ori, region_row, region_col).get_path()
        if len(updated_path) > 1000 or len(updated_path) < 1:
            no_path_exist = True
            print(f"{bcolors.FAIL}--------- The Path of Updated Independent Tasks DOES NOT Exists ---------")
            return no_path_exist

        else:
            print(f"{bcolors.WARNING}--------- The Path of Updated Independent Tasks Exists ---------")

            """
            Get the data ready for plot and video
            """
            # independent_path_2d = [[] for i in range(num_agent)]
            # for i in range(num_agent):
            #     independent_path_2d[i] = [[updated_path[j][2*i], updated_path[j][2*i+1]] for j in range(len(updated_path))]
            #     draw_path(map_obs_work, independent_path_2d[i], fig_folder, "Trajectory (centralized) of heavy-duty robot R" + str(i + 1))

        return no_path_exist
