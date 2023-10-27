import heapq
import time

import numpy as np

from rtools.utils import diagonal_distance_3d, circle_space, rect_space, diagonal_distance_2d, via_cost_function_3


class CostGraph:
    def __init__(self, grid_size: list):
        self.track_space_cost_graph = (- np.ones(grid_size)).tolist()
        """ store the space cost calculated by track width and clearance, the undefined space cost is -1 """
        self.via_space_cost_graph = (- np.ones(grid_size)).tolist()
        """ store the space cost calculated by microvia width and clearance, the undefined space cost is -1 """

    def is_extended(self, pos: list) -> bool:
        x, y, z = pos
        return self.track_space_cost_graph[x][y][z] != -1

    def extend_grid(self, pos: list, track_cost: float, via_cost: float):
        x, y, z = pos
        self.track_space_cost_graph[x][y][z] = track_cost
        self.via_space_cost_graph[x][y][z] = via_cost

    def track_space_cost(self, pos: list) -> float:
        x, y, z = pos
        return self.track_space_cost_graph[x][y][z]

    def via_space_cost(self, pos: list) -> float:
        x, y, z = pos
        return self.via_space_cost_graph[x][y][z]


def calculate_space(pos, radius, occupied_msg, shape='rect'):
    cached_trace_cost = 0
    if shape == 'circle':
        pos_list = circle_space(pos, radius)
    else:
        pos_list = rect_space(pos, radius, radius)
    for coord in pos_list:
        if str(coord) in occupied_msg:
            cached_trace_cost += occupied_msg[str(coord)]
    return cached_trace_cost


class AStarNode:
    """
    AStarNode is the node of A* Algorithm Searching Tree
    """

    def __init__(self, cur_pos: list, start_pos: list, end_pos: list, end_pos_set: set,
                 g_score: float, parent):
        self.cur_pos = cur_pos
        """ the coordinate of the current position """
        self.start_pos = start_pos
        """ the coordinate of the starting position """
        self.end_pos = end_pos
        """ the coordinate of the ending position """
        self.end_pos_set = end_pos_set
        """ the set of the ending positions """

        self.h_score = diagonal_distance_3d(cur_pos, end_pos)
        self.g_score = g_score

        self.f_score = self.g_score + self.h_score
        """ f(x) = g(x) + h(x) """
        self.min_f_score = diagonal_distance_3d(start_pos, end_pos)
        """ the theoretical value of f(x) """

        self.parent = parent
        """ parent node of current node """

    def __lt__(self, other) -> bool:
        return self.f_score < other.f_score

    def set_g_score(self, g_score_with_cost: float):
        """
        set new g_score and recalculate the f_score

        :param g_score_with_cost: new g_score
        :return:
        """
        self.g_score = g_score_with_cost
        self.f_score = self.g_score + self.h_score

    def calculate_g_score(self, direct, next_pos, occupied_msg, pad_list,
                          space_cost_graph, clearance_with_track, clearance_with_microvia):
        via_flag = False
        if self.cur_pos[2] != next_pos[2]:
            g_cost = 10
            via_flag = True
        elif self.cur_pos[0] == next_pos[0] or self.cur_pos[1] == next_pos[1]:
            g_cost = 1
        else:
            g_cost = 1.414

        if direct is not None and not via_flag and \
                direct != [next_pos[0] - self.cur_pos[0], next_pos[1] - self.cur_pos[1], next_pos[2] - self.cur_pos[2]]:
            g_cost += 0.1  # bend cost

        if via_flag or (str(next_pos) not in self.end_pos_set):
            if via_flag:
                g_cost += space_cost_graph.via_space_cost(self.cur_pos)
            if space_cost_graph.is_extended(next_pos):
                if via_flag:
                    space_cost = space_cost_graph.via_space_cost(next_pos)
                else:
                    space_cost = space_cost_graph.track_space_cost(next_pos)
            else:
                track_space_cost = calculate_space(next_pos, clearance_with_track, occupied_msg)
                via_space_cost = calculate_space(next_pos, clearance_with_microvia, occupied_msg)
                space_cost_graph.extend_grid(next_pos, track_space_cost, via_space_cost)
                if via_flag:
                    space_cost = via_space_cost
                else:
                    space_cost = track_space_cost
            g_cost += space_cost

        # # Set Via cost
        # if via_flag:
        #     g_cost_list = []
        #     for pad in pad_list:
        #         pin_hsize = pad.hsize
        #         pin_pos = pad.position
        #         if pad.shape == 'circle':
        #             distance = diagonal_distance_2d(self.cur_pos, pin_pos)
        #             g_cost_list.append(via_cost_function_3(distance, pin_hsize[0]))
        #         else:
        #             relative_x = self.cur_pos[0] - pin_pos[0]
        #             relative_y = self.cur_pos[1] - pin_pos[1]
        #             cost_x = via_cost_function_3(relative_x, pin_hsize[0])
        #             cost_y = via_cost_function_3(relative_y, pin_hsize[1])
        #             if cost_x == 1000 and cost_y == 1000:
        #                 g_cost_list.append(1000)
        #             else:
        #                 if cost_x < 1000 and cost_y < 1000:
        #                     g_cost_list.append((cost_x + cost_y) * 0.5)
        #                 elif cost_x < 1000:
        #                     g_cost_list.append(cost_x)
        #                 else:
        #                     g_cost_list.append(cost_y)
        #
        #     flag = False
        #     count = 0
        #     for tmp_cost in g_cost_list:
        #         if tmp_cost == 1000:
        #             flag = True
        #         if tmp_cost > 1:
        #             count += 1
        #
        #     if flag:
        #         g_cost += 1000
        #     else:
        #         if count == 0:
        #             count = 1
        #         g_cost += sum(g_cost_list) / count

        return self.g_score + g_cost

    def get_neighbors(self, occupied_msg: dict, pad_list: list, space_cost_graph: CostGraph,
                      clearance_with_track: int, clearance_with_microvia: int, grid_size: list) -> list:
        """
        create neighbors of current item, return the list of the neighbors

        :param occupied_msg: the dict of the occupied grids
        :param pad_list:
        :param space_cost_graph: a CostGraph that stores the space cost
        :param clearance_with_track:
        :param clearance_with_microvia:
        :param grid_size: define the size of the grid graph
        :return: the list of the neighbors
        """
        x, y, z = self.cur_pos

        direct = None
        if self.parent is not None:
            x_parent, y_parent, z_parent = self.parent.cur_pos
            direct = [x - x_parent, y - y_parent, z - z_parent]

        neighbors = []
        # go to the east
        if x < grid_size[0] - 1 and direct != [-1, 0, 0]:
            pos = [x + 1, y, z]

            g_score = self.calculate_g_score(direct, pos, occupied_msg, pad_list,
                                             space_cost_graph, clearance_with_track, clearance_with_microvia)

            item = AStarNode(pos, self.start_pos, self.end_pos, self.end_pos_set, g_score, self)
            neighbors.append(item)
        # go to the east-north
        if x < grid_size[0] - 1 and y < grid_size[1] - 1 and direct != [-1, -1, 0]:
            pos = [x + 1, y + 1, z]

            g_score = self.calculate_g_score(direct, pos, occupied_msg, pad_list,
                                             space_cost_graph, clearance_with_track, clearance_with_microvia)

            item = AStarNode(pos, self.start_pos, self.end_pos, self.end_pos_set, g_score, self)
            neighbors.append(item)
        # go to the north
        if y < grid_size[1] - 1 and direct != [0, -1, 0]:
            pos = [x, y + 1, z]

            g_score = self.calculate_g_score(direct, pos, occupied_msg, pad_list,
                                             space_cost_graph, clearance_with_track, clearance_with_microvia)

            item = AStarNode(pos, self.start_pos, self.end_pos, self.end_pos_set, g_score, self)
            neighbors.append(item)
        # go to the west-north
        if x > 0 and y < grid_size[1] - 1 and direct != [1, -1, 0]:
            pos = [x - 1, y + 1, z]

            g_score = self.calculate_g_score(direct, pos, occupied_msg, pad_list,
                                             space_cost_graph, clearance_with_track, clearance_with_microvia)

            item = AStarNode(pos, self.start_pos, self.end_pos, self.end_pos_set, g_score, self)
            neighbors.append(item)
        # go to the west
        if x > 0 and direct != [1, 0, 0]:
            pos = [x - 1, y, z]

            g_score = self.calculate_g_score(direct, pos, occupied_msg, pad_list,
                                             space_cost_graph, clearance_with_track, clearance_with_microvia)

            item = AStarNode(pos, self.start_pos, self.end_pos, self.end_pos_set, g_score, self)
            neighbors.append(item)
        # go to the west-south
        if x > 0 and y > 0 and direct != [1, 1, 0]:
            pos = [x - 1, y - 1, z]

            g_score = self.calculate_g_score(direct, pos, occupied_msg, pad_list,
                                             space_cost_graph, clearance_with_track, clearance_with_microvia)

            item = AStarNode(pos, self.start_pos, self.end_pos, self.end_pos_set, g_score, self)
            neighbors.append(item)
        # go to the south
        if y > 0 and direct != [0, 1, 0]:
            pos = [x, y - 1, z]

            g_score = self.calculate_g_score(direct, pos, occupied_msg, pad_list,
                                             space_cost_graph, clearance_with_track, clearance_with_microvia)

            item = AStarNode(pos, self.start_pos, self.end_pos, self.end_pos_set, g_score, self)
            neighbors.append(item)
        # go to the east-south
        if x < grid_size[0] - 1 and y > 0 and direct != [-1, 1, 0]:
            pos = [x + 1, y - 1, z]

            g_score = self.calculate_g_score(direct, pos, occupied_msg, pad_list,
                                             space_cost_graph, clearance_with_track, clearance_with_microvia)

            item = AStarNode(pos, self.start_pos, self.end_pos, self.end_pos_set, g_score, self)
            neighbors.append(item)

        # go to upside through a via
        if z < grid_size[2] - 1 and direct != [0, 0, -1]:
            pos = [x, y, z + 1]

            g_score = self.calculate_g_score(direct, pos, occupied_msg, pad_list,
                                             space_cost_graph, clearance_with_track, clearance_with_microvia)

            item = AStarNode(pos, self.start_pos, self.end_pos, self.end_pos_set, g_score, self)
            neighbors.append(item)
        # go to downside through a via
        if z > 0 and direct != [0, 0, 1]:
            pos = [x, y, z - 1]

            g_score = self.calculate_g_score(direct, pos, occupied_msg, pad_list,
                                             space_cost_graph, clearance_with_track, clearance_with_microvia)

            item = AStarNode(pos, self.start_pos, self.end_pos, self.end_pos_set, g_score, self)
            neighbors.append(item)

        return neighbors

    def generate_path(self) -> (list, float):
        """
        generate the routing path based on parent node
        :return: routing path, routing cost
        """
        end_item = self
        path = []
        g_cost = end_item.f_score
        while end_item.parent is not None:
            path.append(end_item.cur_pos)
            end_item = end_item.parent
        path.append(end_item.cur_pos)
        return path, g_cost


class AStarRouter:
    def __init__(self, grid_env):
        """

        :param grid_env:
        """
        self.grid_env = grid_env

        self.space_cost_graph = None
        """ a CostGraph that stores the space cost """

        self.route_cost = []

        self.episode_cost = []

        self.route_time = 0

    def run(self, max_episode):
        episode = 0
        while episode < max_episode:
            self.route_cost.clear()
            for net_i in range(self.grid_env.net_num):
                print("routing net{}".format(net_i + 1))

                if self.grid_env.netlist[net_i].two_pin_net_num != 0:
                    self.grid_env.reset(net_i)

                    # rip up the path
                    # self.grid_env.breakup()

                    self.space_cost_graph = CostGraph(self.grid_env.grid_size)
                    net_coord_set = set([])
                    route_cost = 0
                    for two_pin_net in self.grid_env.netlist[net_i].two_pin_net_list:

                        # route_start = time.time()

                        net_coord_set = net_coord_set | two_pin_net[0].pad_coord_set

                        route_start = time.time()

                        route, cost = self.route_two_pin_net(two_pin_net[0].position, net_coord_set,
                                                             two_pin_net[1].position, two_pin_net[1].pad_coord_set,
                                                             net_i)

                        route_end = time.time()
                        print('route time = {} s'.format(route_end - route_start))
                        self.route_time += route_end - route_start

                        for pos in route:
                            net_coord_set.add(str(pos))
                        # update the route and cost
                        self.grid_env.update(route, net_i)

                        route_cost += cost

                    self.route_cost.append(route_cost)

                else:
                    self.route_cost.append(0)

            self.episode_cost.append(sum(self.route_cost))

            episode += 1

    def route_two_pin_net(self, start: list, net_pin_set: set, end: list, end_set: set, net_id: int) -> (list, float):
        """
        Use A* Algorithm to find a legal path between net_pin_set and end_set

        :param net_id:
        :param start: center position of the starting pad
        :param net_pin_set: the set of the routed pads and paths from the same net
        :param end: center position of the ending pad
        :param end_set: the set of the positions of the ending pad
        :return: the routing path and cost
        """

        open_set = []
        # global index table of open set
        open_set_graph = np.zeros(self.grid_env.grid_size).tolist()

        closed_set = set([])

        for pos in net_pin_set:
            # convert the start position from string (which is hashable) to list
            start_pos = list(int(char) for char in pos.strip('[]').split(', '))
            # create a starting item and add it to open set and its global index table
            start_item = AStarNode(start_pos, start, end, end_set, 0.0, None)
            heapq.heappush(open_set, start_item)
            open_set_graph[start_pos[0]][start_pos[1]][start_pos[2]] = start_item

            if not self.space_cost_graph.is_extended(start_pos):
                # calculate space costs and store them to cost graph
                track_space_cost = calculate_space(start_pos,
                                                   self.grid_env.netlist[net_id].net_class.clearance_with_track,
                                                   self.grid_env.occupied_coord)
                via_space_cost = calculate_space(start_pos,
                                                 self.grid_env.netlist[net_id].net_class.clearance_with_microvia,
                                                 self.grid_env.occupied_coord)
                self.space_cost_graph.extend_grid(start_pos, track_space_cost, via_space_cost)

        while open_set:
            cur_item = heapq.heappop(open_set)

            if str(cur_item.cur_pos) in end_set:  # or cur_item.f_score > 1000 + cur_item.min_f_score:
                # if the solver has found a legal path or cannot find a legal path
                return cur_item.generate_path()
            else:
                closed_set.add(str(cur_item.cur_pos))

                neighbor_list = cur_item.get_neighbors(self.grid_env.occupied_coord,
                                                       self.grid_env.netlist[net_id].pad_list,
                                                       self.space_cost_graph,
                                                       self.grid_env.netlist[net_id].net_class.clearance_with_track,
                                                       self.grid_env.netlist[net_id].net_class.clearance_with_microvia,
                                                       self.grid_env.grid_size)

                for neighbor in neighbor_list:
                    if str(neighbor.cur_pos) in closed_set:
                        continue
                    else:
                        item = open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]]
                        if item != 0:
                            # the current node has expended
                            if neighbor.g_score < item.g_score:
                                item.set_g_score(neighbor.g_score)
                                item.parent = neighbor.parent
                                heapq.heapify(open_set)
                        else:
                            heapq.heappush(open_set, neighbor)
                            open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]] = neighbor
