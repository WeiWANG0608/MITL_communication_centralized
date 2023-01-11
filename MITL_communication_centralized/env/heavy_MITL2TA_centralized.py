"""
Author Wei Wang (wei7@kth.se)

This file is used to translate the MITL specifications to a file that UPPAAL can recognize.
"""

import random
import copy
import itertools


class specification_centralized(object):
    def __init__(self):
        self.operator = ''
        self.start_time = 0
        self.end_time = 0
        self.task = ''
        self.ifCoop = False
        self.coop = ''
        self.clock = ''
        self.sync = []  # the channel of the order
        self.update = ''  # the bool value of \phi
        self.detected = False


class MITL2TBA_centralized:

    def __init__(self, states_wts, locations_wts, labels_wts, edges_wts, inputSpec, doc_declaration, ta_decl, init,
                 cell):

        self.ori_spec = inputSpec  # thw whole list for all robots
        self.num_agent = len(self.ori_spec)
        self.doc_decl = doc_declaration
        self.ta_decl = ta_decl
        self.init_loc_ta = init
        self.cell_map = cell
        self.spec_trans = self.split_specification()
        self.states_ta = self.states_TA()
        self.states_wts = states_wts
        self.locations_ta = self.generate_locations_TA()
        self.locations_wts = locations_wts
        self.labels_ta = self.locations_invariant_TA()
        self.labels_wts = labels_wts
        self.edges_ta = self.generate_edges_TA()
        self.edges_wts = edges_wts

    def eventually(self, spec, agent_i, task_i):
        """
        eventually: find all the transitions in edges_wts to the task cell, add Sync: p_i!

        edge_ab = [location_a(name), location_b(name), [_Guard], [_Sync], Update(weight), [x_la, y_la], [x_lb, y_lb]]
                  - location_a(name): l(R)x(R)y

        edges_wts_ab = [location_a(name), location_b(name)], [_Guard], [_Sync], Update(weight), [x_la, y_la], [x_lb, y_lb]
                  - location_a(name): ws
        """
        """
        automata: find all the transition in edges_ta to satisfy state, add Sync: p_i?
                                                                            Update: veri_i:= (time interval)? true:false

                  find all the transition in edges_ta leave satisfy state, add Sync: np_i?

        edge_ab = [location_a(name), location_b(name), [_Guard], [_Sync], Update(weight), [x_la, y_la], [x_lb, y_lb]]
                  - location_a(name): l(R)x(R)y
                  l01 > l11: cl11
                  l02 > l22: cl12
                  l0101 > l2121: cl12 & cl32     agent_i: the item changed (R1, R2,...), task_i: from 0 to i (T1, T2,...)
                  spec.clock[2]: agent_i
                  spec.clock[3]: task_i
        """
        agent_list = [1, 2]
        agent_list.remove(agent_i)

        # print("E - agent_i is", agent_i, " agent_j is ", agent_list[0])
        if agent_i == self.num_agent:
            for i in range(len(self.edges_wts)):

                if self.cell_map[spec.task][0] == self.states_wts[self.edges_wts[i][1]][2 * (agent_i - 1)] and \
                        self.cell_map[spec.task][1] == self.states_wts[self.edges_wts[i][1]][2 * (agent_i - 1) + 1] and \
                        self.edges_wts[i][0] != self.edges_wts[i][1]:
                    # self.states_wts[self.edges_wts[i][0]][0: 2 * (agent_i - 1)] == \
                    # self.states_wts[self.edges_wts[i][1]][0: 2 * (agent_i - 1)]:

                    if self.edges_wts[i][3] == 'null' or spec.sync[0] in self.edges_wts[i][3]:
                        self.edges_wts[i][3] = spec.sync[0] + '!'
                        # print("Sync in WTS - E11 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], " is ", self.edges_wts[i][3])
                    else:
                        old_sync = self.edges_wts[i][3][0: len(self.edges_wts[i][3]) - 1]
                        new_sync = old_sync + spec.sync[0] + '!'
                        self.edges_wts[i][3] = new_sync
                        # print("Sync in WTS get extended - E11 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], "change from", old_sync, ' to ', new_sync)

                    # print("eve", self.edges_wts[i][3])
                if self.cell_map[spec.task][0] == self.states_wts[self.edges_wts[i][0]][2 * (agent_i - 1)] and \
                        self.cell_map[spec.task][1] == self.states_wts[self.edges_wts[i][0]][2 * (agent_i - 1) + 1] and \
                        self.edges_wts[i][0] != self.edges_wts[i][1]:
                    # self.states_wts[self.edges_wts[i][0]][0: 2 * (agent_i - 1)] == \
                    # self.states_wts[self.edges_wts[i][1]][0: 2 * (agent_i - 1)]:
                    if self.edges_wts[i][3] == 'null' or spec.sync[1] in self.edges_wts[i][3]:
                        self.edges_wts[i][3] = spec.sync[1] + '!'
                        # print("Sync in WTS - E21 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], " is ", self.edges_wts[i][3])
                    else:
                        old_sync = self.edges_wts[i][3][0: len(self.edges_wts[i][3]) - 1]
                        new_sync = old_sync + spec.sync[1] + '!'
                        self.edges_wts[i][3] = new_sync
                        # print("Sync in WTS get extended - E21 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], "change from", old_sync, ' to ', new_sync)

            for i in range(len(self.edges_ta)):
                # agent_i changes from state '0' to task_i, not self loop, other robots do not change
                # or both move from '0'
                # or one move from '0' to task_i, one move to '0'

                if int(spec.clock[3]) == int(self.edges_ta[i][1][agent_i]) and \
                        self.edges_ta[i][0] != self.edges_ta[i][1] and \
                        ((self.edges_ta[i][0][agent_i] == '0' and
                          self.edges_ta[i][0][0: agent_i] == self.edges_ta[i][1][0: agent_i]) or
                         (self.edges_ta[i][0][1: len(self.edges_ta[i][0])] == '0' * self.num_agent) or
                         (self.edges_ta[i][0][agent_list[0]] != '0' and
                          self.edges_ta[i][1][agent_list[0]] == '0' and
                          self.edges_ta[i][0][agent_i] == '0' and
                          self.edges_ta[i][1][agent_i] != '0')):
                    # print("E S 1", self.edges_ta[i][0], " to ", self.edges_ta[i][1])
                    if self.edges_ta[i][3] == 'null' or spec.sync[0] in self.edges_ta[i][3]:
                        self.edges_ta[i][3] = spec.sync[0] + '?'
                        # print("Sync in TA - E31 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       " is ", self.edges_ta[i][3])
                    else:
                        old_sync = self.edges_ta[i][3][0: len(self.edges_ta[i][3]) - 1]
                        new_sync = old_sync + spec.sync[0] + '?'
                        self.edges_ta[i][3] = new_sync
                        # print("Sync in TA get extended - E31 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       "change from", old_sync, ' to ', new_sync)

                    if self.edges_ta[i][4] == 'null':
                        self.edges_ta[i][4] = spec.update + ":=(w>=" + spec.start_time + "&& w<=" + spec.end_time + \
                                              ")? true: false"
                        # print("Update in TA - E31 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       " is ", self.edges_ta[i][4])
                    else:
                        old_up = self.edges_ta[i][4]
                        new_up = old_up + '~' + spec.update + ":=(w>=" + spec.start_time + \
                                 "&& w<=" + spec.end_time + ")? true: false"
                        self.edges_ta[i][4] = new_up
                        # print("Update in TA get extended - E41", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       "change from", old_up, " to ", new_up)

                # agent_i changes from state task_i to '0', not self loop, other robots do not change
                if int(spec.clock[3]) == int(self.edges_ta[i][0][agent_i]) and \
                        self.edges_ta[i][0] != self.edges_ta[i][1] and \
                        ((self.edges_ta[i][1][agent_i] == '0' and
                          self.edges_ta[i][0][0: agent_i] == self.edges_ta[i][1][0: agent_i]) or
                         (self.edges_ta[i][1][1: len(self.edges_ta[i][1])] == '0' * self.num_agent) or
                         (self.edges_ta[i][0][agent_list[0]] == '0' and
                          self.edges_ta[i][1][agent_list[0]] != '0' and
                          self.edges_ta[i][0][agent_i] != '0' and
                          self.edges_ta[i][1][agent_i] == '0')):
                    # print("E NS 1", self.edges_ta[i][0], " to ", self.edges_ta[i][1])
                    if self.edges_ta[i][3] == 'null' or spec.sync[1] in self.edges_ta[i][3]:
                        self.edges_ta[i][3] = spec.sync[1] + '?'
                        # print("Sync in TA - E51 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       " is ", self.edges_ta[i][3])
                    else:
                        old_sync = self.edges_ta[i][3][0: len(self.edges_ta[i][3]) - 1]
                        new_sync = old_sync + spec.sync[1] + '?'
                        self.edges_ta[i][3] = new_sync
                        # print("Sync in TA get extended - E51 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       "change from", old_sync, ' to ', new_sync)
        else:
            for i in range(len(self.edges_wts)):
                if self.cell_map[spec.task][0] == self.states_wts[self.edges_wts[i][1]][2 * (agent_i - 1)] and \
                        self.cell_map[spec.task][1] == self.states_wts[self.edges_wts[i][1]][
                    2 * (agent_i - 1) + 1] and \
                        self.edges_wts[i][0] != self.edges_wts[i][1]:
                    # self.states_wts[self.edges_wts[i][0]][0: 2 * (agent_i - 1)] == \
                    # self.states_wts[self.edges_wts[i][1]][0: 2 * (agent_i - 1)] and \
                    # self.states_wts[self.edges_wts[i][0]][
                    # 2 * agent_i: len(self.states_wts[self.edges_wts[i][0]])] == \
                    # self.states_wts[self.edges_wts[i][1]][
                    # 2 * agent_i: len(self.states_wts[self.edges_wts[i][0]])]:

                    if self.edges_wts[i][3] == 'null' or spec.sync[0] in self.edges_wts[i][3]:
                        self.edges_wts[i][3] = spec.sync[0] + '!'
                        # print("Sync in WTS - E21 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], " is ", self.edges_wts[i][3])
                    else:
                        old_sync = self.edges_wts[i][3][0: len(self.edges_wts[i][3]) - 1]
                        new_sync = old_sync + spec.sync[0] + '!'
                        self.edges_wts[i][3] = new_sync
                        # print("Sync in WTS get extended - E12 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], "change from", old_sync, ' to ', new_sync)

                    # print("eve", self.edges_wts[i][3])
                if self.cell_map[spec.task][0] == self.states_wts[self.edges_wts[i][0]][2 * (agent_i - 1)] and \
                        self.cell_map[spec.task][1] == self.states_wts[self.edges_wts[i][0]][
                    2 * (agent_i - 1) + 1] and \
                        self.edges_wts[i][0] != self.edges_wts[i][1]:
                    # self.states_wts[self.edges_wts[i][0]][0: 2 * (agent_i - 1)] == \
                    # self.states_wts[self.edges_wts[i][1]][0: 2 * (agent_i - 1)] and \
                    # self.states_wts[self.edges_wts[i][0]][
                    # 2 * agent_i: len(self.states_wts[self.edges_wts[i][0]])] == \
                    # self.states_wts[self.edges_wts[i][1]][
                    # 2 * agent_i: len(self.states_wts[self.edges_wts[i][0]])]:
                    if self.edges_wts[i][3] == 'null' or spec.sync[1] in self.edges_wts[i][3]:
                        self.edges_wts[i][3] = spec.sync[1] + '!'
                        # print("Sync in WTS - E22 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], " is ", self.edges_wts[i][3])
                    else:
                        old_sync = self.edges_wts[i][3][0: len(self.edges_wts[i][3]) - 1]
                        new_sync = old_sync + spec.sync[1] + '!'
                        self.edges_wts[i][3] = new_sync
                        # print("Sync in WTS get extended - E22 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], "change from", old_sync, ' to ', new_sync)

            for i in range(len(self.edges_ta)):
                # agent_i changes from state '0' to task_i, not self loop, other robots do not change
                if int(spec.clock[3]) == int(self.edges_ta[i][1][agent_i]) and \
                        self.edges_ta[i][0] != self.edges_ta[i][1] and \
                        ((self.edges_ta[i][0][agent_i] == '0' and
                          self.edges_ta[i][0][0: agent_i] == self.edges_ta[i][1][0: agent_i] and
                          self.edges_ta[i][0][agent_i + 1: len(self.edges_ta[i][0])] ==
                          self.edges_ta[i][1][agent_i + 1: len(self.edges_ta[i][0])]) or
                         (self.edges_ta[i][0][1: len(self.edges_ta[i][0])] == '0' * self.num_agent) or
                         (self.edges_ta[i][0][agent_list[0]] != '0' and
                          self.edges_ta[i][1][agent_list[0]] == '0' and
                          self.edges_ta[i][0][agent_i] == '0' and
                          self.edges_ta[i][1][agent_i] != '0')):
                    # print("E S 2", self.edges_ta[i][0], " to ", self.edges_ta[i][1])
                    if self.edges_ta[i][3] == 'null' or spec.sync[0] in self.edges_ta[i][3]:
                        self.edges_ta[i][3] = spec.sync[0] + '?'
                        # print("Sync in TA - E32 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       " is ", self.edges_ta[i][3])
                    else:
                        old_sync = self.edges_ta[i][3][0: len(self.edges_ta[i][3]) - 1]
                        new_sync = old_sync + spec.sync[0] + '?'
                        self.edges_ta[i][3] = new_sync
                        # print("Sync in TA get extended - E32 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       "change from", old_sync, ' to ', new_sync)

                    if self.edges_ta[i][4] == 'null':
                        self.edges_ta[i][4] = spec.update + ":=(w>=" + spec.start_time + "&& w<=" + spec.end_time + \
                                              ")? true: false"
                        # print("Update in TA - E42 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       " is ", self.edges_ta[i][4])
                    else:
                        old_up = self.edges_ta[i][4]
                        new_up = old_up + '~' + spec.update + ":=(w>=" + spec.start_time + \
                                 "&& w<=" + spec.end_time + ")? true: false"
                        self.edges_ta[i][4] = new_up
                        # print("Update in TA get extended - E42", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       "change from", old_up, " to ", new_up)

                # agent_i changes from state task_i to '0', not self loop, other robots do not change
                if int(spec.clock[3]) == int(self.edges_ta[i][0][agent_i]) and \
                        self.edges_ta[i][0] != self.edges_ta[i][1] and \
                        ((self.edges_ta[i][1][agent_i] == '0' and
                          self.edges_ta[i][0][0: agent_i] == self.edges_ta[i][1][0: agent_i] and
                          self.edges_ta[i][0][agent_i + 1: len(self.edges_ta[i][0])] ==
                          self.edges_ta[i][1][agent_i + 1: len(self.edges_ta[i][0])]) or
                         (self.edges_ta[i][1][1: len(self.edges_ta[i][1])] == '0' * self.num_agent) or
                         (self.edges_ta[i][0][agent_list[0]] == '0' and
                          self.edges_ta[i][1][agent_list[0]] != '0' and
                          self.edges_ta[i][0][agent_i] != '0' and
                          self.edges_ta[i][1][agent_i] == '0')):
                    # print("E NS 2", self.edges_ta[i][0], " to ", self.edges_ta[i][1])
                    if self.edges_ta[i][3] == 'null' or spec.sync[1] in self.edges_ta[i][3]:
                        self.edges_ta[i][3] = spec.sync[1] + '?'
                        # print("Sync in TA - E52 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       " is ", self.edges_ta[i][3])
                    else:
                        old_sync = self.edges_ta[i][3][0: len(self.edges_ta[i][3]) - 1]
                        new_sync = old_sync + spec.sync[1] + '?'
                        self.edges_ta[i][3] = new_sync
                        # print("Sync in TA get extended - E52 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       "change from", old_sync, ' to ', new_sync)

        return None

    def globally(self, spec, agent_i, task_i):
        """
        globally: find all the transitions in edges_wts to the task cell, add Sync: p_i!
                           the loop transition in edges_wts at task cell, add Sync: p_i!

                           the transitions in edges_wts leave the task cell, add Sync: np_i!
        """

        """
        globally: timed automata

        find all the transition in edges_ta to satisfy state, add Sync: p_i?
                                                                  Update: cl_i:=0   
                 the loop transition in edges_ta at satisfy state, add Sync: p_i?
                                                                       Update: cl_i = cl_i+1
                                                                         veri_i:= (cl_i == end_t-start_t && time interval)? true:false
                                                                         [10, 15]: t>= 15
        """
        agent_list = [1, 2]
        agent_list.remove(agent_i)

        # print("G - agent_i is", agent_i, " agent_j is ", agent_list[0])
        if agent_i == self.num_agent:
            for i in range(len(self.edges_wts)):
                if self.cell_map[spec.task][0] == self.states_wts[self.edges_wts[i][1]][2 * (agent_i - 1)] and \
                        self.cell_map[spec.task][1] == self.states_wts[self.edges_wts[i][1]][2 * (agent_i - 1) + 1]:

                    if self.edges_wts[i][3] == 'null' or spec.sync[0] in self.edges_wts[i][3]:
                        self.edges_wts[i][3] = spec.sync[0] + '!'
                        # print("Sync in WTS - G11 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], " is ", self.edges_wts[i][3])
                    else:
                        old_sync = self.edges_wts[i][3][0: len(self.edges_wts[i][3]) - 1]
                        new_sync = old_sync + spec.sync[0] + '!'
                        self.edges_wts[i][3] = new_sync
                        # print("Sync in WTS get extended - G11 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], "change from", old_sync, ' to ', new_sync)
                    # print("globally", self.edges_wts[i][3])
                if self.cell_map[spec.task][0] == self.states_wts[self.edges_wts[i][0]][2 * (agent_i - 1)] and \
                        self.cell_map[spec.task][1] == self.states_wts[self.edges_wts[i][0]][2 * (agent_i - 1) + 1] and \
                        self.edges_wts[i][0] != self.edges_wts[i][1]:
                    if self.edges_wts[i][3] == 'null' or spec.sync[1] in self.edges_wts[i][3]:
                        self.edges_wts[i][3] = spec.sync[1] + '!'
                        # print("Sync in WTS - G21 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], " is ", self.edges_wts[i][3])
                    else:
                        old_sync = self.edges_wts[i][3][0: len(self.edges_wts[i][3]) - 1]
                        new_sync = old_sync + spec.sync[1] + '!'
                        self.edges_wts[i][3] = new_sync
                        # print("Sync in WTS get extended - G21 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], "change from", old_sync, ' to ', new_sync)
                    # print("globally", self.edges_wts[i][3])

            for i in range(len(self.edges_ta)):
                # to satisfy state
                if int(spec.clock[3]) == int(self.edges_ta[i][1][agent_i]):
                    # not loop
                    if self.edges_ta[i][0] != self.edges_ta[i][1] and ((self.edges_ta[i][0][agent_i] == '0' and
                                                                        self.edges_ta[i][0][0: agent_i] ==
                                                                        self.edges_ta[i][1][0: agent_i]) or
                                                                       (self.edges_ta[i][0][1: len(self.edges_ta[i][0])]
                                                                        == '0' * self.num_agent) or
                                                                       (self.edges_ta[i][0][agent_list[0]] != '0' and
                                                                        self.edges_ta[i][1][agent_list[0]] == '0' and
                                                                        self.edges_ta[i][0][agent_i] == '0' and
                                                                        self.edges_ta[i][1][agent_i] != '0')):
                        # print("G S 1", self.edges_ta[i][0], " to ", self.edges_ta[i][1])
                        if self.edges_ta[i][2] == 'null':
                            self.edges_ta[i][2] = "w>=" + str(int(spec.start_time) - 1)
                            # print("Guard in TA - G31 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       " is ", self.edges_ta[i][2])
                        else:
                            old_g = self.edges_ta[i][2]
                            new_g = old_g + '&&' + "w>=" + str(int(spec.start_time) - 1)
                            self.edges_ta[i][2] = new_g
                            # print('G31 TA guard', self.edges_ta[i][0], "-", self.edges_ta[i][1], "change from", old_g,
                            #       " to ", new_g)
                        if self.edges_ta[i][3] == 'null' or spec.sync[0] in self.edges_ta[i][3]:
                            self.edges_ta[i][3] = spec.sync[0] + '?'
                            # print("Sync in TA - G31 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       " is ", self.edges_ta[i][3])
                        else:
                            old_sync = self.edges_ta[i][3][0: len(self.edges_ta[i][3]) - 1]
                            new_sync = old_sync + spec.sync[0] + '?'
                            self.edges_ta[i][3] = new_sync
                            # print("Sync in TA get extended - G31 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       "change from", old_sync, ' to ', new_sync)

                        if self.edges_ta[i][4] == 'null':
                            self.edges_ta[i][4] = spec.clock + ":=0"
                            # print("Update in TA - G31 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       " is ", self.edges_ta[i][4])
                        else:
                            old_up = self.edges_ta[i][4]
                            new_up = old_up + '~' + spec.clock + ":=0"
                            self.edges_ta[i][4] = new_up
                            # print('G31 TA update', self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       "change from", old_up, ' to ', new_up)
                    # loop
                    if self.edges_ta[i][0] == self.edges_ta[i][1]:
                        if self.edges_ta[i][3] == 'null' or spec.sync[0] in self.edges_ta[i][3]:
                            self.edges_ta[i][3] = spec.sync[0] + '?'
                            # print("Sync in TA - G41 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       " is ", self.edges_ta[i][3])
                        else:
                            old_sync = self.edges_ta[i][3][0: len(self.edges_ta[i][3]) - 1]
                            new_sync = old_sync + spec.sync[0] + '?'
                            self.edges_ta[i][3] = new_sync
                            # print("Sync in TA get extended - G41 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       "change from", old_sync, ' to ', new_sync)

                        if self.edges_ta[i][4] == 'null':
                            self.edges_ta[i][
                                4] = spec.clock + "=" + spec.clock + "+1~" + spec.update + ":=(" + spec.clock + \
                                     "==" + str(int(spec.end_time) - int(spec.start_time)) + " && w>=" + \
                                     spec.start_time + " && w<=" + spec.end_time + ")? true: false"
                            # print("Update in TA - G41 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       " is ", self.edges_ta[i][4])
                        else:
                            old_up = self.edges_ta[i][4]
                            new_up = old_up + '~' + spec.clock + "=" + spec.clock + "+1~" + \
                                     spec.update + ":=(" + spec.clock + "==" + \
                                     str(int(spec.end_time) - int(spec.start_time)) + " && w>=" + \
                                     spec.start_time + " && w<=" + spec.end_time + ")? true: false"
                            self.edges_ta[i][4] = new_up
                            # print('G41 TA update', self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       "change from", old_up, ' to ', new_up)

                """
                the transition in edges_ta leave satisfy state, add Sync: np_i?
                """
                if int(spec.clock[3]) == int(self.edges_ta[i][0][agent_i]) and \
                        self.edges_ta[i][0] != self.edges_ta[i][1] and \
                        ((self.edges_ta[i][1][agent_i] == '0' and
                          self.edges_ta[i][0][0: agent_i] == self.edges_ta[i][1][0: agent_i]) or
                         (self.edges_ta[i][1][1: len(self.edges_ta[i][1])] == '0' * self.num_agent) or
                         (self.edges_ta[i][0][agent_list[0]] == '0' and
                          self.edges_ta[i][1][agent_list[0]] != '0' and
                          self.edges_ta[i][0][agent_i] != '0' and
                          self.edges_ta[i][1][agent_i] == '0')):
                    # print("G NS 1", self.edges_ta[i][0], " to ", self.edges_ta[i][1])
                    if self.edges_ta[i][3] == 'null' or spec.sync[1] in self.edges_ta[i][3]:
                        self.edges_ta[i][3] = spec.sync[1] + '?'
                        # print("Sync in TA - G51 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       " is ", self.edges_ta[i][3])
                    else:
                        old_sync = self.edges_ta[i][3][0: len(self.edges_ta[i][3]) - 1]
                        new_sync = old_sync + spec.sync[1] + '?'
                        self.edges_ta[i][3] = new_sync
                        # print("Sync in TA get extended - G51 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       "change from", old_sync, ' to ', new_sync)
        else:
            for i in range(len(self.edges_wts)):
                if self.cell_map[spec.task][0] == self.states_wts[self.edges_wts[i][1]][2 * (agent_i - 1)] and \
                        self.cell_map[spec.task][1] == self.states_wts[self.edges_wts[i][1]][2 * (agent_i - 1) + 1]:

                    # self.states_wts[self.edges_wts[i][0]][0: 2 * (agent_i - 1)] == \
                    # self.states_wts[self.edges_wts[i][1]][0: 2 * (agent_i - 1)] and \
                    # self.states_wts[self.edges_wts[i][0]][
                    # 2 * agent_i: len(self.states_wts[self.edges_wts[i][0]])] == \
                    # self.states_wts[self.edges_wts[i][1]][2 * agent_i: len(self.states_wts[self.edges_wts[i][0]])]:
                    if self.edges_wts[i][3] == 'null' or spec.sync[0] in self.edges_wts[i][3]:
                        self.edges_wts[i][3] = spec.sync[0] + '!'
                        # print("Sync in WTS - G12 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], " is ", self.edges_wts[i][3])
                    else:
                        old_sync = self.edges_wts[i][3][0: len(self.edges_wts[i][3]) - 1]
                        new_sync = old_sync + spec.sync[0] + '!'
                        self.edges_wts[i][3] = new_sync
                        # print("Sync in WTS get extended - G12 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], "change from", old_sync, ' to ', new_sync)
                    # print("globally", self.edges_wts[i][3])
                if self.cell_map[spec.task][0] == self.states_wts[self.edges_wts[i][0]][2 * (agent_i - 1)] and \
                        self.cell_map[spec.task][1] == self.states_wts[self.edges_wts[i][0]][2 * (agent_i - 1) + 1] and \
                        self.edges_wts[i][0] != self.edges_wts[i][1]:
                    # self.states_wts[self.edges_wts[i][0]][0: 2 * (agent_i - 1)] == \
                    # self.states_wts[self.edges_wts[i][1]][0: 2 * (agent_i - 1)] and \
                    # self.states_wts[self.edges_wts[i][0]][
                    # 2 * agent_i: len(self.states_wts[self.edges_wts[i][0]])] == \
                    # self.states_wts[self.edges_wts[i][1]][2 * agent_i: len(self.states_wts[self.edges_wts[i][0]])]:
                    if self.edges_wts[i][3] == 'null' or spec.sync[1] in self.edges_wts[i][3]:
                        self.edges_wts[i][3] = spec.sync[1] + '!'
                        # print("Sync in WTS - G22 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], " is ", self.edges_wts[i][3])
                    else:
                        old_sync = self.edges_wts[i][3][0: len(self.edges_wts[i][3]) - 1]
                        new_sync = old_sync + spec.sync[1] + '!'
                        self.edges_wts[i][3] = new_sync
                        # print("Sync in WTS get extended - G22 ", self.states_wts[self.edges_wts[i][0]], "-",
                        #       self.states_wts[self.edges_wts[i][1]], "change from", old_sync, ' to ', new_sync)
                    # print("globally", self.edges_wts[i][3])

            for i in range(len(self.edges_ta)):
                # to satisfy state
                if int(spec.clock[3]) == int(self.edges_ta[i][1][agent_i]):
                    # not loop
                    if self.edges_ta[i][0] != self.edges_ta[i][1] and ((self.edges_ta[i][0][agent_i] == '0' and
                                                                        self.edges_ta[i][0][0: agent_i] ==
                                                                        self.edges_ta[i][1][0: agent_i] and
                                                                        self.edges_ta[i][0][
                                                                        agent_i + 1: len(self.edges_ta[i][0])] ==
                                                                        self.edges_ta[i][1][
                                                                        agent_i + 1: len(self.edges_ta[i][0])]) or
                                                                       (self.edges_ta[i][0][1: len(self.edges_ta[i][0])]
                                                                        == '0' * self.num_agent) or
                                                                       (self.edges_ta[i][0][agent_list[0]] != '0' and
                                                                        self.edges_ta[i][1][agent_list[0]] == '0' and
                                                                        self.edges_ta[i][0][agent_i] == '0' and
                                                                        self.edges_ta[i][1][agent_i] != '0')):
                        # print("G S 2", self.edges_ta[i][0], " to ", self.edges_ta[i][1])
                        if self.edges_ta[i][2] == 'null':
                            self.edges_ta[i][2] = "w>=" + str(int(spec.start_time) - 1)
                            # print("Guard in TA - G32 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       " is ", self.edges_ta[i][2])
                        else:
                            old_g = self.edges_ta[i][2]
                            new_g = old_g + '&&' + "w>=" + str(int(spec.start_time) - 1)
                            self.edges_ta[i][2] = new_g
                            # print('G32 TA guard', self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       "change from", old_g, ' to ', new_g)
                        if self.edges_ta[i][3] == 'null' or spec.sync[0] in self.edges_ta[i][3]:
                            self.edges_ta[i][3] = spec.sync[0] + '?'
                        else:
                            old_sync = self.edges_ta[i][3][0: len(self.edges_ta[i][3]) - 1]
                            new_sync = old_sync + spec.sync[0] + '?'
                            self.edges_ta[i][3] = new_sync
                            # print("Sync in TA get extended - G32 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       "change from", old_sync, ' to ', new_sync)

                        if self.edges_ta[i][4] == 'null':
                            self.edges_ta[i][4] = spec.clock + ":=0"
                            # print("Update in TA - G32 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       " is ", self.edges_ta[i][4])
                        else:
                            old_up = self.edges_ta[i][4]
                            new_up = old_up + '~' + spec.clock + ":=0"
                            self.edges_ta[i][4] = new_up
                            # print('G32 TA update', self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       "change from", old_up, ' to ', new_up)
                    # loop
                    if self.edges_ta[i][0] == self.edges_ta[i][1]:
                        if self.edges_ta[i][3] == 'null' or spec.sync[0] in self.edges_ta[i][3]:
                            self.edges_ta[i][3] = spec.sync[0] + '?'
                            # print("Sync in TA - G42 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       " is ", self.edges_ta[i][3])
                        else:
                            old_sync = self.edges_ta[i][3][0: len(self.edges_ta[i][3]) - 1]
                            new_sync = old_sync + spec.sync[0] + '?'
                            self.edges_ta[i][3] = new_sync
                            # print("Sync in TA get extended - G42 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       "change from", old_sync, ' to ', new_sync)

                        if self.edges_ta[i][4] == 'null':
                            self.edges_ta[i][
                                4] = spec.clock + "=" + spec.clock + "+1~" + spec.update + ":=(" + spec.clock + \
                                     "==" + str(int(spec.end_time) - int(spec.start_time)) + " && w>=" + \
                                     spec.start_time + " && w<=" + spec.end_time + ")? true: false"
                            # print("Update in TA - G42 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       " is ", self.edges_ta[i][4])
                        else:
                            old_up = self.edges_ta[i][4]
                            new_up = old_up + '~' + spec.clock + "=" + spec.clock + "+1~" + \
                                     spec.update + ":=(" + spec.clock + "==" + \
                                     str(int(spec.end_time) - int(spec.start_time)) + " && w>=" + \
                                     spec.start_time + " && w<=" + spec.end_time + ")? true: false"
                            self.edges_ta[i][4] = new_up
                            # print('G42 TA update', self.edges_ta[i][0], "-", self.edges_ta[i][1],
                            #       "change from", old_up, ' to ', new_up)

                """
                the transition in edges_ta leave satisfy state, add Sync: np_i?
                """

                if int(spec.clock[3]) == int(self.edges_ta[i][0][agent_i]) and \
                        self.edges_ta[i][0] != self.edges_ta[i][1] and \
                        ((self.edges_ta[i][1][agent_i] == '0' and
                          self.edges_ta[i][0][0: agent_i] == self.edges_ta[i][1][0: agent_i] and
                          self.edges_ta[i][0][agent_i + 1: len(self.edges_ta[i][0])] ==
                          self.edges_ta[i][1][agent_i + 1: len(self.edges_ta[i][0])]) or
                         (self.edges_ta[i][1][1: len(self.edges_ta[i][1])] == '0' * self.num_agent) or
                         (self.edges_ta[i][0][agent_list[0]] == '0' and
                          self.edges_ta[i][1][agent_list[0]] != '0' and
                          self.edges_ta[i][0][agent_i] != '0' and
                          self.edges_ta[i][1][agent_i] == '0')):
                    # print("G NS 2", self.edges_ta[i][0], " to ", self.edges_ta[i][1])
                    if self.edges_ta[i][3] == 'null' or spec.sync[1] in self.edges_ta[i][3]:
                        self.edges_ta[i][3] = spec.sync[1] + '?'
                        # print("Sync in TA - G52 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       " is ", self.edges_ta[i][3])
                    else:
                        old_sync = self.edges_ta[i][3][0: len(self.edges_ta[i][3]) - 1]
                        new_sync = old_sync + spec.sync[1] + '?'
                        self.edges_ta[i][3] = new_sync
                        # print("Sync in TA get extended - G52 ", self.edges_ta[i][0], "-", self.edges_ta[i][1],
                        #       "change from", old_sync, ' to ', new_sync)

        return None

    def split_specification(self):
        """Input:    self.ori_spec = [['G', '15', '18', 'c13'], ['E', '32', '40', 'c16', 'R1']]
           Output    Object [] - obj.clock: cl(j)i (clock name, int) start from 1
                               - obj.operator: E, G
                               - obj.start_time: int
                               - obj.end_time: int
                               - obj.task: cell (location) name
                               - obj.sync: p(j)i (if the ap is sent: p(j)i! /receive: p(j)i?)
                               - obj.update: veri(j)_i
                               if cooperative:
                                 - obj.ifCoop: True (otherwise False)
                                 - obj.coop: the robot need to cooperate
        """
        speclist = [[specification_centralized() for i in range(len(self.ori_spec[j]))] for j in range(len(self.ori_spec))]
        channel_list = []
        for j in range(len(speclist)):
            for i, spec in enumerate(speclist[j]):
                spec.clock = 'cl' + str(j + 1) + str(i + 1)  # the index of the clock for agent i
                addDeclaration(self.ta_decl, ['int', 'cl' + str(j + 1) + str(i + 1)])
                spec.operator = self.ori_spec[j][i][0]  # temporal operator
                spec.start_time = self.ori_spec[j][i][1]
                spec.end_time = self.ori_spec[j][i][2]
                spec.task = self.ori_spec[j][i][3]  # target cell
                spec.sync = ['p' + str(j + 1) + str(i + 1),
                             'np' + str(j + 1) + str(i + 1)]  # the index of commands
                channel_list.append('p' + str(j + 1) + str(i + 1))
                channel_list.append('np' + str(j + 1) + str(i + 1))
                addDeclaration(self.doc_decl, ['chan', 'p' + str(j + 1) + str(i + 1)])
                addDeclaration(self.doc_decl, ['chan', 'np' + str(j + 1) + str(i + 1)])
                spec.update = 'veri' + str(j + 1) + str(i + 1)  # the index of verification character
                addDeclaration(self.doc_decl, ['bool', 'veri' + str(j + 1) + str(i + 1)])
                if len(self.ori_spec[j][i]) > 4:
                    spec.ifCoop = True
                    spec.coop = self.ori_spec[j][i][4]
        # print("The channel list is ", channel_list)
        multi_ch_string = get_all_channel_centralized(channel_list)
        # print("The combined channel list is ", multi_ch_string)
        addDeclaration(self.doc_decl, ['chan', multi_ch_string])
        # print("Doc declaration: ", self.doc_decl)
        # print("TA declaration: ", self.ta_decl)
        return speclist

    def states_TA(self):
        """
        states: l00 ~ la[R1: 1,2,...]b[R2: 1,2,...]

        :return:
        """
        states = dict()
        if len(self.ori_spec) == 2:
            for i in range(len(self.spec_trans[0]) + 1):
                for j in range(len(self.spec_trans[1]) + 1):
                    states['l' + str(i) + str(j)] = [i * random.randint(5, 20) * 10, j * random.randint(5, 20) * 10]

        if len(self.ori_spec) == 4:
            for i in range(len(self.spec_trans[0]) + 1):
                for j in range(len(self.spec_trans[1]) + 1):
                    for k in range(len(self.spec_trans[2]) + 1):
                        for l in range(len(self.spec_trans[3]) + 1):
                            states['l' + str(i) + str(j) + str(k) + str(l)] = [(i + k) * random.randint(5, 20) * 10,
                                                                               (j + l) * random.randint(5, 20) * 10]
        if len(self.ori_spec) == 6:
            for i in range(len(self.spec_trans[0]) + 1):
                for j in range(len(self.spec_trans[1]) + 1):
                    for k in range(len(self.spec_trans[2]) + 1):
                        for l in range(len(self.spec_trans[3]) + 1):
                            for m in range(len(self.spec_trans[4]) + 1):
                                for n in range(len(self.spec_trans[5]) + 1):
                                    states['l' + str(i) + str(j) + str(k) + str(l) + str(m) + str(n)] = \
                                        [(i + k + m) * random.randint(5, 20) * 10,
                                         (j + l + n) * random.randint(5, 20) * 10]

        # print("States in TA ", states)
        print("The size of the TA is", len(states))
        return states

    def get_state_TA(self, val):
        for key, value in self.states_ta.items():
            if val == value:
                return key
        return "key doesn't exist"

    def generate_locations_TA(self):
        """
        location_i = [location_i (name), rate of exponential, x_i_inUppaal, y_i_inUppaal, initial (true/false)]
        """
        locations = []
        for i in self.states_ta:
            if i[1:len(i)] == self.init_loc_ta:
                locations.append([i, "null", str(self.states_ta[i][0]), str(self.states_ta[i][1]), "true"])
            else:
                locations.append([i, "null", str(self.states_ta[i][0]), str(self.states_ta[i][1]), "false"])

        return locations

    def locations_invariant_TA(self):
        labels_loc = []
        for i in range(len(self.states_ta)):
            labels_loc.append(["null"])
        return labels_loc

    def generate_edges_TA(self):
        """
        edge_ab = [[location_a(name), location_b(name)], [_Guard], [_Sync], Update(weight), [x_la, y_la], [x_lb, y_lb]]
        """
        edges = list()
        if len(self.ori_spec) == 2:
            for i in self.states_ta:
                # edges of self loop and from/to initial state l00
                if i != 'l00':
                    edges.append(['l00', i, "null", "null", "null"])
                    edges.append([i, 'l00', "null", "null", "null"])
                    edges.append([i, i, "null", "null", "null"])

                # edges when R1 moves first
                if i[1] != '0' and i[2] == '0':
                    for j in self.states_ta:
                        if (i[1] == j[1] and i[2] != j[2]) or (j[1] == '0' and j[2] != '0'):
                            if [i, j, "null", "null", "null"] not in edges:
                                edges.append([i, j, "null", "null", "null"])
                                edges.append([j, i, "null", "null", "null"])

                # edges when R2 moves first
                if i[1] == '0' and i[2] != '0':
                    for j in self.states_ta:
                        if (i[2] == j[2] and i[1] != j[1]) or (j[1] != '0' and j[2] == '0'):
                            if [i, j, "null", "null", "null"] not in edges:
                                edges.append([i, j, "null", "null", "null"])
                                edges.append([j, i, "null", "null", "null"])

        if len(self.ori_spec) == 4:
            for i in self.states_ta:
                # edges of self loop and from/to initial state l00
                if i != 'l0000':
                    edges.append(['l0000', i, "null", "null", "null"])
                    edges.append([i, 'l0000', "null", "null", "null"])
                    edges.append([i, i, "null", "null", "null"])

                # edges when one robot moves first
                if i.count('0') == 3:
                    loop_list_1 = [1, 2, 3, 4]
                    for lp1 in range(1, 4):
                        if i[lp1] != 0:  # i: 'l x000'
                            for j in self.states_ta:
                                # connect all edges of the second level
                                if i[lp1] == j[lp1] and i != j:
                                    edges.append([i, j, "null", "null", "null"])
                                    edges.append([j, i, "null", "null", "null"])
                                loop_list_2 = copy.deepcopy(loop_list_1)
                                loop_list_2.pop(lp1)
                                for lp2 in loop_list_2:
                                    if i[lp1] == j[lp1] and j[lp2] != 0:  # j: 'l xy00'
                                        for k in self.states_ta:
                                            # connect all edges of the third level
                                            if i[lp1] == k[lp1] and j[lp2] == k[lp2] and j != k:
                                                edges.append([j, k, "null", "null", "null"])
                                                edges.append([k, j, "null", "null", "null"])
                                            loop_list_3 = copy.deepcopy(loop_list_2)
                                            loop_list_3.pop(lp2)
                                            for lp3 in loop_list_3:
                                                if i[lp1] == k[lp1] and j[lp2] == k[lp2] and k[lp3] != 0:  # k: 'l xyz0'
                                                    for l in self.states_ta:
                                                        if i[lp1] == l[lp1] and j[lp2] == l[lp2] and k[lp3] == l[lp3] \
                                                                and k != l:
                                                            edges.append([k, l, "null", "null", "null"])
                                                            edges.append([l, k, "null", "null", "null"])
                # edges when two robot moves first
                if i.count('0') == 2:
                    loop_list_1 = [1, 2, 3, 4]
                    for lp1 in range(1, 4):
                        loop_list_2 = copy.deepcopy(loop_list_1)
                        loop_list_2.pop(lp1)
                        for lp2 in loop_list_2:
                            if i[lp1] != 0 and i[lp2] != 0:  # i: 'l xy00'
                                for j in self.states_ta:
                                    if i[lp1] == j[lp1] and i[lp2] == j[lp2] and i != j:
                                        edges.append([i, j, "null", "null", "null"])
                                        edges.append([j, i, "null", "null", "null"])
                                    loop_list_3 = copy.deepcopy(loop_list_2)
                                    loop_list_3.pop(lp2)
                                    for lp3 in loop_list_3:
                                        if i[lp1] == j[lp1] and i[lp2] == j[lp2] and j[lp3] != 0:  # j: 'l xyz0'
                                            for k in self.states_ta:
                                                if i[lp1] == k[lp1] and j[lp2] == k[lp2] and j != k:
                                                    edges.append([j, k, "null", "null", "null"])
                                                    edges.append([k, j, "null", "null", "null"])
                # edges when three robot moves first
                if i.count('0') == 1:
                    loop_list_1 = [1, 2, 3, 4]
                    for lp1 in range(1, 4):
                        loop_list_2 = copy.deepcopy(loop_list_1)
                        loop_list_2.pop(lp1)
                        for lp2 in loop_list_2:
                            loop_list_3 = copy.deepcopy(loop_list_2)
                            loop_list_3.pop(lp2)
                            for lp3 in loop_list_3:
                                if i[lp1] != 0 and i[lp2] != 0 and i[lp3] != 0:  # i: 'l xyz0'
                                    for j in self.states_ta:
                                        if i[lp1] == j[lp1] and i[lp2] == j[lp2] and i[lp3] == j[lp3] and i != j:
                                            edges.append([i, j, "null", "null", "null"])
                                            edges.append([j, i, "null", "null", "null"])
        # print("Edges in TA ", edges)
        return edges

    def get_product_wts(self):
        for j in range(len(self.spec_trans)):
            for i, spec in enumerate(self.spec_trans[j]):
                if self.spec_trans[j][i].operator == 'G':
                    self.globally(self.spec_trans[j][i], j + 1, i + 1)
                if self.spec_trans[j][i].operator == 'E':
                    self.eventually(self.spec_trans[j][i], j + 1, i + 1)
        # print("The edges of TA:", self.edges_ta)
        # print("The edges of WTS:", self.edges_wts)
        return self.edges_ta


def get_all_channel_centralized(channels):
    channels_list = []
    for r in range(len(channels) + 1):
        for combination in itertools.combinations(channels, r):
            channels_list.append(''.join(combination))
    nc = [x for x in channels_list if len(x) > 4]
    new_channels = ','.join(nc)
    # print("Combined string is ", new_channels)
    return new_channels


def addDeclaration(decl, input):
    # input[0]: variable type
    # input[1]: variable names
    if input[0] in decl:
        if len(decl[input[0]]) > 0 and (input[1] not in decl[input[0]][0: len(decl[input[0]]) - 1].split(',')):
            decl[input[0]] = decl[input[0]][0: len(decl[input[0]]) - 1]
            decl[input[0]] = decl[input[0]] + ',' + input[1] + ';'
    else:
        decl[input[0]] = input[1] + ';'

    return decl
