"""
Author Wei Wang (wei7@kth.se)

class heavy_path: used to translate the MITL specifications to a file that UPPAAL can recognize
                  and generate the network of timed automata, as well as translate the WTS into
                  a file for instance solving.

- get_work_map(): get the map of each work regions
- get_path_cell2coord(): get the path from cell name to coordinate
- if_collab_meet_promise(): check the request is collab or meet, if meet request then whether it needs promise
- get_promise_request(): get the meet promise and meet request
- get_cooperation_map(): get the new map after accepting the requests
- get_update_path(): get the new path upon the specification after accepting the requests
- get_independent_requests_tasks(): get the independent task and independent requests
"""

import time
from datetime import datetime
from initialization import get_milli_time
import numpy as np
from heavy_WTS_centralized import TransitionSystem_centralized, total_states_centralized
from heavy_MITL2TA_centralized import MITL2TBA_centralized
import subprocess
import json
import copy
import os

dirname = os.path.dirname
ABPATH = os.path.join(dirname(dirname(__file__)))


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    UNDERLINE = '\033[4m'


class heavy_path_centralized:
    """
    Generates file "flag.txt" to identify the current verification agent Rx
    Generates file "system_infoRx.txt" for independent task verification of Rx by UPPAAL
    Generates file "system_infoCRx.txt" for independent task + independent requests verification of Rx by UPPAAL
    Read "system_dataRx.json", which is the WTS Ti generated from workspace and TBA Ai generated from specification

    /output/...
    Generate file "pathoutputRx.txt" if there is a path for Rx under independent task
    Generate file "pathoutputCRx.txt" if there is a path for Rx under independent task + independent requests
    Generate file "(repo)Rx.xml/.xtr" by UPPAAL if there is a path for Rx under independent task
    Generate file "(repo)CRx.xml/.xtr" by UPPAAL if there is a path for Rx under independent task + independent requests
    """

    def __init__(self, map_ori, specifications, filefolder, flag, agents_heavy, cell_ori, region_row, region_col):
        self.map = map_ori
        self.spec_list = specifications
        self.filefolder = filefolder
        self.flag = flag
        self.init_pos = agents_heavy
        self.cells = cell_ori
        self.num_agent = len(agents_heavy)
        self.region_row = region_row
        self.region_col = region_col
        self.states_wts, self.state_wt, self.build = self.get_product()
        # self.path = self.get_path()

    def declaration_toString(self, decl_dict):
        decl = ''
        for i in decl_dict:
            decl = decl + i + ' ' + decl_dict[i] + '\n'
        if decl == '':
            decl = 'null'
        return decl

    def get_product(self):
        start_milli = get_milli_time()
        doc_declaration = dict()
        wts_declaration = dict()
        ta_declaration = dict()

        # # heavy-duty robot module
        print("Start calculating WTS")
        wts_heavy = TransitionSystem_centralized(self.map, doc_declaration, self.init_pos)
        state_wts = wts_heavy.states_WTS()
        locations_wts = wts_heavy.generate_locations_WTS()
        labels_wts = wts_heavy.locations_invariant_WTS()
        edges_wts = wts_heavy.generate_edges_WTS()

        init_ta = '0' * self.num_agent
        print("Start calculating TA")
        ta_heavy = MITL2TBA_centralized(state_wts, locations_wts, labels_wts, edges_wts, self.spec_list,
                                        doc_declaration, ta_declaration, init_ta, self.cells)
        state_ta = ta_heavy.states_TA()
        locations_ta = ta_heavy.generate_locations_TA()
        labels_ta = ta_heavy.locations_invariant_TA()
        edges_ta = ta_heavy.get_product_wts()

        text_file = open(self.filefolder + "/" + 'flag' + ".txt", "w")
        text_file.write(self.flag)
        text_file.close()
        text_file = open(self.filefolder + "/" + 'region' + ".txt", "w")
        text_file.write(str(self.region_row) + '*' + str(self.region_col))
        text_file.close()

        "#  &"
        doc_system = "ProcessWTS() = WTS();\nProcessTA() = TA();\nsystem ProcessWTS, ProcessTA;"
        doc_decl = self.declaration_toString(doc_declaration)

        temp_decl = self.declaration_toString(wts_declaration) + '#' + self.declaration_toString(ta_declaration)
        temp_list = 'WTS#TA'

        info_all = doc_system + '&' + doc_decl + '&' + temp_decl + '&' + temp_list
        text_file = open(self.filefolder + "/" + 'system_info' + self.flag + ".txt", "w")
        text_file.write(info_all)
        text_file.close()

        # for visualization
        for i in range(len(locations_wts)):
            locations_wts[i][2] = str(int(locations_wts[i][2]) * 200)
            locations_wts[i][3] = str(int(locations_wts[i][3]) * 200)

        locations = {'wts': locations_wts, 'ta': locations_ta}
        labels = {'wts': labels_wts, 'ta': labels_ta}
        edges = {'wts': edges_wts, 'ta': edges_ta}

        data = {"locations": locations,
                "labels": labels,
                "edges": edges}

        with open(self.filefolder + "/" + 'system_data' + self.flag + '.json', 'w') as outfile:
            json.dump(data, outfile, indent=4)
        outfile.close()
        milli_time = get_milli_time() - start_milli
        if self.flag in ['Round1', 'Round2']:
            f = open(self.filefolder + "/output/" + 'Time_cen_' + str(self.region_row) + "*" + str(self.region_col)
                     + ".txt", "a")  # append mode
            f.write(str(milli_time)+",")
            f.close()

        subprocess.run([ABPATH + '/deploy.sh'])

        return state_wts, state_ta, milli_time

    def get_path(self):
        with open(self.filefolder + '/output/' + 'pathoutput' + self.flag + '.txt') as f:
            lines = f.readlines()
        path = []
        if len(lines) > 0:
            path_state = [item.strip() for item in lines]
            path = [self.states_wts[i] for i in path_state]
        # print("heavy_path_generation ", path)
        return path

    def get_java_time(self):
        with open(self.filefolder + '/output/' + 'javatime' + self.flag + '.txt') as f:
            lines = f.readlines()
        javatime = 0
        if len(lines) > 0:
            javatime = int(lines[0])
        return javatime


def get_path_cell2coord(path, ori_dict, num_agent):
    path_coord = []
    for i in range(len(path)):
        path_coord.append(ori_dict[path[i]])
    return path_coord


def if_collab_meet_promise(spec):
    ifcollab = False
    ifmeet = False
    ifpromise = False
    if len(spec) > 4:
        if spec[5] == 'col':
            ifcollab = True
        else:
            ifmeet = True
            if spec[0] == "E":
                ifpromise = True
    return ifcollab, ifmeet, ifpromise


def get_promise_request_centralized(spec, index, map_local, filefolder, flag, init_p, cells, agent_i,
                                    region_row, region_col):
    veri_spec = copy.deepcopy(spec)
    promise = copy.deepcopy(veri_spec[agent_i][index])
    request = copy.deepcopy(veri_spec[agent_i][index])
    print("request", request)
    request[1] = request[2]
    if veri_spec[agent_i][index][1] < veri_spec[agent_i][index][2]:
        for i in range(1, int(veri_spec[agent_i][index][2]) - int(veri_spec[agent_i][index][1])):
            print("******** Start getting the promise ********")
            # heavy_path(map_regions, veri_spec, file_folder, flag, agents_h, cells).get_path()

            path_heavy_promise = heavy_path_centralized(map_local, veri_spec, filefolder, flag, init_p, cells,
                                                        region_row, region_col).get_path()

            start = get_milli_time()
            while len(path_heavy_promise) == 0:
                dua = get_milli_time()-start
                if dua > 100000:
                    break
            # print("The file length is :", len(path_heavy_promise))
            if 0 < len(path_heavy_promise) < 1000:
                print("======== Find the best promise ========")
                promise = veri_spec[agent_i][index]
                request = copy.deepcopy(veri_spec[agent_i][index])
                request[0] = 'E'
                print("Meet promise is ", promise, ". Meet request is ", request)
                break
            else:
                print("======== old ========")
                veri_spec[agent_i][index][1] = str(int(veri_spec[agent_i][index][1]) + 1)
    return promise, request


def get_cooperation_map_centralized(map_ori, map_list, coop_region, target_h):
    print("The cooperative task is in region ", coop_region, " the needed robot is ", target_h)
    reg_r = int(len(map_ori) / 2)
    new_map = copy.deepcopy(map_ori)
    region_list = [i for i in range(len(map_list))]
    for i in coop_region:
        region_list.remove(i)
    region_list.remove(target_h)
    print("The current map contains the follow task regions ", coop_region, target_h)
    for i in region_list:
        for pi in range(map_list[i][0], map_list[i][1]):
            for pj in range(map_list[i][2], map_list[i][3]):
                # the boundary of work regions are accessible
                if pi != reg_r - 1 and pi != reg_r:
                    new_map[pi, pj] = 1
    # draw_map_simple(new_map, 'open boundary')
    return new_map


def get_independent_requests_tasks(specs, map_regions, agents_h, file_folder, cells, region_row, region_col):
    num = len(specs)
    independent_requests = [[[] for j in range(num)] for i in range(num)]
    independent_tasks = [[] for i in range(num)]
    running_time = 0
    checking_time = 0
    for i in range(num):
        flag = "CenR" + str(i)
        for k in range(len(specs[i])):
            ifcollab, ifmeet, ifpromise = if_collab_meet_promise(specs[i][k])
            if not ifcollab and not ifmeet:
                independent_tasks[i].append(specs[i][k])
            # collab requests
            elif ifcollab:
                j = int(specs[i][k][4][1:len(specs[i][k][4])])
                independent_requests[i][j].append(specs[i][k])
            # meet requests
            elif ifmeet and not ifpromise:
                independent_tasks[i].append(specs[i][k][0:4])
                j = int(specs[i][k][4][1:len(specs[i][k][4])])
                independent_requests[i][j].append(specs[i][k])
            elif ifmeet and ifpromise:
                veri_spec = copy.deepcopy(specs)
                veri_spec[i][k][0] = 'G'
                print("The spec to verify (before) is ", veri_spec)
                meet_promise_candi = veri_spec[i][k]
                veri_spec[i] = [item for item in veri_spec[i] if
                                (len(item) == 4 or (len(item) > 4 and item[5] != 'col'))]
                if len(independent_tasks[i]) > 0:
                    veri_spec[i][0:k] = independent_tasks[i]
                veri_path = heavy_path_centralized(map_regions, veri_spec, file_folder, flag, agents_h,
                                                   cells, region_row, region_col).get_path()
                if len(veri_path) > 0:
                    j = int(specs[i][k][4][1:len(specs[i][k][4])])
                    independent_requests[i][j].append(specs[i][k])
                    independent_tasks[i].append(meet_promise_candi[0:4])
                else:
                    index = veri_spec[i].index(meet_promise_candi)
                    meet_promise, meet_request = get_promise_request_centralized(veri_spec, index, map_regions,
                                                                                 file_folder, flag, agents_h, cells, i,
                                                                                 region_row, region_col)
                    j = int(meet_request[4][1:len(meet_request[4])])
                    independent_requests[i][j].append(meet_request)
                    independent_tasks[i].append(meet_promise)

    return independent_tasks, independent_requests
