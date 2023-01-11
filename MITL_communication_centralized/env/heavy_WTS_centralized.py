"""
Author Wei Wang (wei7@kth.se)

This file is used to generate weighted transition systems upon the initialized environment for instance solving.
"""

import numpy as np
import math
import random

W_I = 1
W_SE = 1
W_NW = 1


class TransitionSystem_centralized:
    """
    # already satisfy never W_obs in states(): self.map[pi, pj] != 1
    """

    def __init__(self, map_ori, doc_declaration, init_location):
        self.map = map_ori
        self.doc_decl = doc_declaration
        self.row = len(self.map)
        self.col = len(self.map[0])
        # self.row, self.col = self.map.shape
        self.init_loc = init_location
        self.num_agent = len(init_location)
        self.states = self.states_WTS()
        self.locations = self.generate_locations_WTS()
        self.labels_location = self.locations_invariant_WTS()
        self.edges = self.generate_edges_WTS()

    def states_WTS(self):
        states = dict()
        s = 0
        if self.num_agent == 2:
            # Player position
            for p1 in range(self.row):
                for q1 in range(self.col):
                    for p2 in range(self.row):
                        for q2 in range(self.col):
                            # already satisfy never W_obs
                            if self.map[p1, q1] != 1 and self.map[p2, q2] != 1:
                                states['s' + str(s)] = [p1, q1, p2, q2]
                                s += 1
        elif self.num_agent == 4:
            for p1 in range(self.row):
                for q1 in range(self.col):
                    for p2 in range(self.row):
                        for q2 in range(self.col):
                            for p3 in range(self.row):
                                for q3 in range(self.col):
                                    for p4 in range(self.row):
                                        for q4 in range(self.col):
                                            if self.map[p1, q1] != 1 and self.map[p2, q2] != 1 and self.map[p3, q3] != 1 \
                                                    and self.map[p4, q4] != 1:
                                                states['s' + str(s)] = [p1, q1, p2, q2, p3, q3, p4, q4]
                                                s += 1
        elif self.num_agent == 6:
            for p1 in range(self.row):
                for q1 in range(self.col):
                    for p2 in range(self.row):
                        for q2 in range(self.col):
                            for p3 in range(self.row):
                                for q3 in range(self.col):
                                    for p4 in range(self.row):
                                        for q4 in range(self.col):
                                            for p5 in range(self.row):
                                                for q5 in range(self.col):
                                                    for p6 in range(self.row):
                                                        for q6 in range(self.col):
                                                            if self.map[p1, q1] != 1 and self.map[p2, q2] != 1 and \
                                                                    self.map[p3, q3] != 1 and self.map[p4, q4] != 1 and \
                                                                    self.map[p5, q5] != 1 and self.map[p6, q6] != 1:
                                                                states['s' + str(s)] = [p1, q1, p2, q2, p3, q3, p4, q4,
                                                                                        p5, q5, p6, q6]
                                                                s += 1
        print("The size of the WTS is", s)
        # print("States in WTS ", states)
        return states

    def get_state(self, val):
        for key, value in self.states.items():
            if val == value:
                return key
        return "key doesn't exist"

    def generate_locations_WTS(self):
        """
        location_i = [location_i (name), rate of exponential, x_i_inUppaal, y_i_inUppaal, initial (true/false)]
        """
        locations = []
        list_loc = []
        for i in range(len(self.init_loc)):
            list_loc.append(self.init_loc[i][0])
            list_loc.append(self.init_loc[1][1])
        print("The pos of the initial state is ", list_loc)

        for i in self.states:
            # print("The state is ", self.states[i])
            if self.states[i] == list_loc:
                locations.append([i, "null", str(self.states[i][0]), str(self.states[i][1]), "true"])
                print("The initial position is ", i)
            else:
                locations.append([i, "null", str(self.states[i][0]), str(self.states[i][1]), "false"])

        return locations

    def locations_invariant_WTS(self):
        labels_loc = []
        for i in range(len(self.states)):
            labels_loc.append(["null"])
        return labels_loc

    def generate_edges_WTS(self):
        """
        edge_ab = [[location_a(name), location_b(name)], [_Guard], [_Sync], Update(weight), [x_la, y_la], [x_lb, y_lb]]
        """
        addDeclaration_centralized(self.doc_decl, ['int', 'w'])
        edges = list()
        key_list = list(self.states)
        for i in range(len(self.states)):
            for j in range(len(self.states)):
                # act = Idle
                if i == j:
                    edges.append([key_list[i], key_list[j], "null", "null", "w=w+" + str(W_I)])
                    # print("Add loop edges between ", self.states[key_list[i]], self.states[key_list[j]])

                length = len(self.states[key_list[0]])
                minus_list = [a - b for a, b in zip(self.states[key_list[j]], self.states[key_list[i]])]
                # each robot can only move maximum one step
                if (not any([minus_list[2 * i + 1] != 0 and minus_list[2 * i] != 0 for i in
                             range(int(len(minus_list) / 2))])) and \
                        length / 2 <= minus_list.count(0) < length and \
                        minus_list.count(1) == length - minus_list.count(0) - minus_list.count(-1):
                    edges.append([key_list[i], key_list[j], "null", "null", "w=w+" + str(W_SE)])
                    # print("Add edges ", self.states[key_list[i]], self.states[key_list[j]])

        # print("Edges in WTS ", edges)
        print("The size of the WTS - edges is", len(edges))
        return edges


def addDeclaration_centralized(decl, val):
    if val[0] in decl:
        if len(decl[val[0]]) > 0 and (val[1] not in decl[val[0]][0: len(decl[val[0]]) - 1].split(',')):
            decl[val[0]] = decl[val[0]][0: len(decl[val[0]]) - 1]
            decl[val[0]] = decl[val[0]] + ',' + val[1] + ';'
    else:
        decl[val[0]] = val[1] + ';'

    return decl


def total_states_centralized(map):
    states = dict()
    s = 0
    # Player position
    for pi in range(len(map)):
        for pj in range(len(map[0])):
            # already satisfy never W_obs
            if map[pi, pj] != 1:
                states['w' + str(s)] = [pi, pj]
                s += 1
    return states
