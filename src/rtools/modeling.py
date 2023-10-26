import math

from rtools.utils import diagonal_distance_3d, circle_space_set, rect_space_set


def to_grid_coord_round_down(real_coord):
    return int(real_coord * 3 / 0.25)  # 0.25


def to_grid_coord_round_up(real_coord):
    return math.ceil(real_coord * 3 / 0.25)  # 0.25


def to_real_coord(grid_coord):
    return grid_coord / 3 * 0.25  # 0.25


class Pad:
    def __init__(self, pos, layer, shape, size, pad_type, net_id):
        self.position = [to_grid_coord_round_down(pos[0]), to_grid_coord_round_down(pos[1]), pos[2]]
        self.layer = layer
        self.shape = shape
        self.hsize = [int(to_grid_coord_round_up(size[0]) / 2), int(to_grid_coord_round_up(size[1]) / 2)]
        self.type = pad_type
        self.net_id = net_id

        self.occupied_coord_set = set([])
        self.pad_coord_set = set([])

    def generate_coord_set(self, grid_size):
        x_coord = self.position[0]
        y_coord = self.position[1]
        z_coord = self.position[2]

        self.occupied_coord_set = set([])
        self.pad_coord_set = set([])

        if self.type == 'thru_hole':
            z_min = 0
            z_max = grid_size[2]
        else:
            # pad.type == 'smd'
            z_min = z_coord
            z_max = z_coord + 1

        if self.shape == 'circle':  # ? boundary detection ?
            for z_i in range(z_max - z_min):
                self.occupied_coord_set = self.occupied_coord_set | circle_space_set([x_coord, y_coord, z_i],
                                                                                     self.hsize[0])
                self.pad_coord_set = self.pad_coord_set | circle_space_set([x_coord, y_coord, z_i], self.hsize[0] - 1)
        else:
            for z_i in range(z_max - z_min):
                self.occupied_coord_set = \
                    self.occupied_coord_set | rect_space_set([x_coord, y_coord, z_i], self.hsize[0], self.hsize[1])
                self.pad_coord_set = \
                    self.pad_coord_set | rect_space_set([x_coord, y_coord, z_i], self.hsize[0] - 1, self.hsize[1] - 1)


class NetClass:
    def __init__(self, track_width, microvia_diameter, clearance_with_track, clearance_with_microvia):
        self.track_width = to_grid_coord_round_up(track_width)
        self.microvia_diameter = to_grid_coord_round_up(microvia_diameter)

        self.clearance_with_track = to_grid_coord_round_down(clearance_with_track)
        self.clearance_with_microvia = to_grid_coord_round_down(clearance_with_microvia)


class Net:
    def __init__(self, net_id, net_name, net_class):
        self.net_id = net_id
        self.net_name = net_name
        self.net_class = net_class
        self.pad_list = []
        self.pad_num = 0

        self.two_pin_net_list = []
        self.two_pin_net_num = 0
        self.two_pin_net_route_list = []

    def generate_two_pin_net(self):
        self.two_pin_net_num = self.pad_num - 1

        two_pin_nets = []
        two_pin_set = []
        for j in range(self.pad_num - 1):
            for p in range(self.pad_num - j - 1):
                pin_start = self.pad_list[j]
                pin_end = self.pad_list[p + j + 1]
                distance = diagonal_distance_3d(pin_start.position, pin_end.position)
                two_pin_set.append(([pin_start, pin_end], distance))
        two_pin_set.sort(key=lambda x: x[1])

        connected_component = []
        for j in range(len(two_pin_set)):
            is_loop = True
            if not connected_component:
                fetched_pins = [two_pin_set[j][0][0].position, two_pin_set[j][0][1].position]
                connected_component.append(fetched_pins)
                is_loop = False
            else:
                static_c_i = [None, None]
                for c_i in range(len(connected_component)):
                    if two_pin_set[j][0][0].position not in connected_component[c_i] \
                            and two_pin_set[j][0][1].position not in connected_component[c_i]:
                        if c_i == len(connected_component) - 1:
                            if static_c_i[0] is None:
                                fetched_pins = [two_pin_set[j][0][0].position, two_pin_set[j][0][1].position]
                                connected_component.append(fetched_pins)
                                is_loop = False
                                break
                            else:
                                connected_component[static_c_i[0]].append(two_pin_set[j][0][static_c_i[1]].position)
                                is_loop = False
                                break
                        else:
                            continue
                    else:
                        if two_pin_set[j][0][0].position in connected_component[c_i] \
                                and two_pin_set[j][0][1].position not in connected_component[c_i]:
                            if static_c_i[0] is not None:
                                for pin in connected_component[c_i]:
                                    connected_component[static_c_i[0]].append(pin)
                                connected_component.pop(c_i)
                                is_loop = False
                                break
                            else:
                                if c_i == len(connected_component) - 1:
                                    connected_component[c_i].append(two_pin_set[j][0][1].position)
                                    is_loop = False
                                    break
                                else:
                                    static_c_i[0] = c_i
                                    static_c_i[1] = 1
                                    continue
                        elif two_pin_set[j][0][0].position not in connected_component[c_i] \
                                and two_pin_set[j][0][1].position in connected_component[c_i]:
                            if static_c_i[0] is not None:
                                for pin in connected_component[c_i]:
                                    connected_component[static_c_i[0]].append(pin)
                                connected_component.pop(c_i)
                                is_loop = False
                                break
                            else:
                                if c_i == len(connected_component) - 1:
                                    connected_component[c_i].append(two_pin_set[j][0][0].position)
                                    is_loop = False
                                    break
                                else:
                                    static_c_i[0] = c_i
                                    static_c_i[1] = 0
                                    continue
                        else:
                            is_loop = True
                            break
            if not is_loop:
                two_pin_nets.append(two_pin_set[j][0])
            if len(two_pin_nets) >= self.two_pin_net_num:
                break
        # adjust the order to make the starting pad is belongs to connected component
        if len(two_pin_nets) > 0:
            p = 1
            q = p
            connected_nodes = {str(two_pin_nets[0][0].position), str(two_pin_nets[0][1].position)}
            while p < len(two_pin_nets):
                if str(two_pin_nets[q][0].position) in connected_nodes:
                    connected_nodes.add(str(two_pin_nets[q][1].position))
                    two_pin_nets[p], two_pin_nets[q] = two_pin_nets[q], two_pin_nets[p]
                    p += 1
                    q = p
                elif str(two_pin_nets[q][1].position) in connected_nodes:
                    connected_nodes.add(str(two_pin_nets[q][0].position))
                    two_pin_nets[q][0], two_pin_nets[q][1] = two_pin_nets[q][1], two_pin_nets[q][0]
                    two_pin_nets[p], two_pin_nets[q] = two_pin_nets[q], two_pin_nets[p]
                    p += 1
                    q = p
                else:
                    q += 1

        self.two_pin_net_list = two_pin_nets


class GridEnv:
    def __init__(self, board_area, layer_num, net_num, net_list, net_class, pad_obstacles):
        """
        Create the grid environment based on dataset

        :param board_area:
        :param layer_num:
        :param net_num:
        :param net_list:
        :param net_class:
        :param pad_obstacles:
        """
        self.grid_size = [to_grid_coord_round_down(board_area[0] - board_area[1]),
                          to_grid_coord_round_down(board_area[2] - board_area[3]),
                          layer_num]
        """ the size of the grid graph """

        self.net_num = net_num
        """ the number of nets """

        self.netlist = self.load_net_list(net_list, net_class)
        """
        netlist: a list of the pads from each net
        """

        self.pad_obstacles = self.load_pad_obstacles(pad_obstacles)

        # self.electric_width = []
        """ 
            It is a list of the electrical width, each electrical width is also a list and has two elements.
            The first element is calculated by track width and clearance.
            The second element is calculated by microvia width and clearance.
        """

        # self.widthList = []
        """
        a list of the width of each net. The first element of the width is track_width, the second is microvia_width.
        """

        # self.pad_size_array = []
        """
            It is a list of the coordinates of the pads.
            pad_size_array[i][j] is the j-st pad's coordinates of the i-st net
        """
        # self.pin_hsize_array = []
        """
            It is a list of the pads' size.
            pin_hsize_array[i][j] is the j-st pad's size of the i-st net
            pin_hsize_array[i][j] = [x_hsize, y_hsize, shape]
        """
        # self.pin_grid_set = {}
        """ It is a dict[str, set], which use string of the coordinate to index pad's grids. """

        # self.twoPinNetCombo, self.twoPinNetNums, self.netlist = self.generate_two_pin_net()
        """
        twoPinNetCombo: a list of the two-pin nets from each net. The order of the two-pin nets is based on MST.
        twoPinNetNums: a list of the number of the two-pin nets form each net
        netlist: a list of the pad from each net
        """

        self.occupied_coord = self.generate_occupied_coord()
        """ a dict stores the occupied grids """

        # self.route_combo = []
        # self.route_cost = []
        # self.episode_cost = []

        # self.init_pad = None
        # self.goal_pad = None

        # self.twoPinNet_i = 0
        # self.multiPinNet_i = 0
        # self.netPinSet = set([])
        # self.netPinRoute = []
        # self.old_netPinRoute = []
        # self.route = []
        # self.old_route = []
        # self.cost = 0

        # self.episode = 0

    def load_net_list(self, net_list, net_class):
        net_list_tmp = []
        for i in range(self.net_num):
            net_info = net_list[i + 1]
            net = Net(net_info.netID, net_info.netName, NetClass(net_class[net_info.netClass].track_width,
                                                                 net_class[net_info.netClass].microvia_diameter,
                                                                 net_class[net_info.netClass].clearance_with_track,
                                                                 net_class[net_info.netClass].clearance_with_microvia))
            for pad_info in net_info.padList:
                pad = Pad(pad_info.position, pad_info.layer, pad_info.shape,
                          pad_info.size, pad_info.type, pad_info.netID)
                # add occupied coordinates
                pad.generate_coord_set(self.grid_size)
                net.pad_list.append(pad)
            net.pad_num = len(net.pad_list)

            net.generate_two_pin_net()

            net_list_tmp.append(net)

        return net_list_tmp

    def load_pad_obstacles(self, pad_obstacles):
        pad_list_tmp = []
        for pad_info in pad_obstacles:
            pad = Pad(pad_info.position, pad_info.layer, pad_info.shape,
                      pad_info.size, pad_info.type, pad_info.netID)
            # add occupied coordinates
            pad.generate_coord_set(self.grid_size)
            pad_list_tmp.append(pad)

        return pad_list_tmp

    def generate_occupied_coord(self) -> dict:
        """
        Create an initial dict which contain the obstacle grid

        :return: the dict of the occupied grids
        """
        occupied_dict = {}

        # Set grid graph refer to boundary list
        # TODO

        # Set the pads with a net
        for i in range(self.net_num):
            net = self.netlist[i]
            for j in range(net.pad_num):
                pad = net.pad_list[j]

                for hide_pos in pad.occupied_coord_set:
                    if hide_pos in occupied_dict:
                        occupied_dict[hide_pos] += 1000
                    else:
                        occupied_dict[hide_pos] = 1000  # Large Enough

        # Set obstacle pad
        for obs_pad in self.pad_obstacles:
            for hide_pos in obs_pad.occupied_coord_set:
                if hide_pos in occupied_dict:
                    occupied_dict[hide_pos] += 1000
                else:
                    occupied_dict[hide_pos] = 1000  # Large Enough

        return occupied_dict

    def add_pin_effect(self, pad_size_set: set):
        """
        Add the base cost of the grids of pad_size_set to the occupied grids

        :param pad_size_set: the set of the coordinates of the pad
        :return:
        """
        for hide_pos in pad_size_set:
            if hide_pos in self.occupied_coord:
                self.occupied_coord[hide_pos] += 1000
            else:
                self.occupied_coord[hide_pos] = 1000  # Large Enough

    def eliminate_pin_effect(self, pad_size_set: set):
        """
        Delete the base cost of the grids of pad_size_set to the occupied grids

        :param pad_size_set: the set of the coordinates of the pad
        :return:
        """
        for hide_pos in pad_size_set:
            self.occupied_coord[hide_pos] -= 1000
            if self.occupied_coord[hide_pos] <= 0:
                del self.occupied_coord[str(hide_pos)]

    def get_route(self, origin_route: list):
        """
        store the inflection points of the path as the route result

        :param origin_route: the routing path with all the points
        :return:
        """
        route = []
        for i in range(len(origin_route)):
            if i < 1:
                route.append(origin_route[i])
            elif i == len(origin_route) - 1:
                route.append(origin_route[i])
            else:
                x_0, y_0, z_0 = origin_route[i - 1]
                x_1, y_1, z_1 = origin_route[i]
                x_2, y_2, z_2 = origin_route[i + 1]
                direct_0 = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
                direct_1 = [x_2 - x_1, y_2 - y_1, z_2 - z_1]
                if direct_0 != direct_1:
                    route.append(origin_route[i])
        return route

    def add_occupied_coord(self, route: list, distance: list):
        """
        Add the base cost of the routing path to the occupied grid

        :param route: the routing path
        :param distance: the first element is track_width, the second is microvia_width
        :return:
        """
        d_line = distance[0]
        via_d = distance[1]

        for i in range(len(route) - 1):
            x_0, y_0, z_0 = route[i]
            x_1, y_1, z_1 = route[i + 1]
            direct = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
            if direct[0] == 0:
                if direct[1] == 0:  # go through a via
                    x_max = x_0 + via_d + 1
                    x_min = x_0 - via_d
                    y_max = y_0 + via_d + 1
                    y_min = y_0 - via_d
                    if z_0 < z_1:
                        z_max = z_1 + 1
                        z_min = z_0
                    else:
                        z_max = z_0 + 1
                        z_min = z_1
                else:  # go to the north or south
                    x_max = x_0 + d_line + 1
                    x_min = x_0 - d_line
                    if y_0 < y_1:  # go to the north
                        y_max = y_1 + d_line + 1
                        y_min = y_0 - d_line
                    else:  # go to the south
                        y_max = y_0 + d_line + 1
                        y_min = y_1 - d_line
                    z_max = z_0 + 1
                    z_min = z_0

                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]
                if z_min < 0:
                    z_min = 0
                if z_max > self.grid_size[2]:
                    z_max = self.grid_size[2]

                for x_i in range(x_max - x_min):
                    for y_i in range(y_max - y_min):
                        for z_i in range(z_max - z_min):
                            hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                            if str(hide_pos) in self.occupied_coord:
                                self.occupied_coord[str(hide_pos)] += 1000
                            else:
                                self.occupied_coord[str(hide_pos)] = 1000  # Large Enough
            elif direct[1] == 0:  # go to the east or west
                if x_0 < x_1:  # go to the east
                    x_max = x_1 + d_line + 1
                    x_min = x_0 - d_line
                else:  # go to the west
                    x_max = x_0 + d_line + 1
                    x_min = x_1 - d_line
                y_max = y_0 + d_line + 1
                y_min = y_0 - d_line
                z_max = z_0 + 1
                z_min = z_0

                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]
                if z_min < 0:
                    z_min = 0
                if z_max > self.grid_size[2]:
                    z_max = self.grid_size[2]

                for x_i in range(x_max - x_min):
                    for y_i in range(y_max - y_min):
                        for z_i in range(z_max - z_min):
                            hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                            if str(hide_pos) in self.occupied_coord:
                                self.occupied_coord[str(hide_pos)] += 1000
                            else:
                                self.occupied_coord[str(hide_pos)] = 1000  # Large Enough
            else:  # go diagonally
                if direct[0] > 0:
                    delta_x = 1
                    if direct[1] > 0:  # go to the north-east
                        delta_y = 1
                    else:  # go to the south-east
                        delta_y = -1
                else:
                    delta_x = -1
                    if direct[1] > 0:  # go to the north-west
                        delta_y = 1
                    else:  # go to the south-west
                        delta_y = -1
                while x_0 != x_1 + delta_x:
                    x_max = x_0 + d_line + 1
                    x_min = x_0 - d_line
                    y_max = y_0 + d_line + 1
                    y_min = y_0 - d_line

                    if x_min < 0:
                        x_min = 0
                    if x_max > self.grid_size[0]:
                        x_max = self.grid_size[0]
                    if y_min < 0:
                        y_min = 0
                    if y_max > self.grid_size[1]:
                        y_max = self.grid_size[1]
                    # Set the cost of the routed path Large Enough
                    for x_i in range(x_max - x_min):
                        for y_i in range(y_max - y_min):
                            hide_pos = [x_min + x_i, y_min + y_i, z_0]
                            if str(hide_pos) in self.occupied_coord:
                                self.occupied_coord[str(hide_pos)] += 1000
                            else:
                                self.occupied_coord[str(hide_pos)] = 1000  # Large Enough

                    x_0 += delta_x
                    y_0 += delta_y

    def del_occupied_coord(self, old_route: list, distance: list):
        """
        Add the base cost of the routing path to the occupied grid

        :param old_route: the routing path
        :param distance: the first element is track_width, the second is microvia_width
        :return:
        """
        d_line = distance[0]
        via_d = distance[1]

        for i in range(len(old_route) - 1):
            x_0, y_0, z_0 = old_route[i]
            x_1, y_1, z_1 = old_route[i + 1]
            direct = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
            if direct[0] == 0:
                if direct[1] == 0:  # go through a via
                    x_max = x_0 + via_d + 1
                    x_min = x_0 - via_d
                    y_max = y_0 + via_d + 1
                    y_min = y_0 - via_d
                    if z_0 < z_1:
                        z_max = z_1 + 1
                        z_min = z_0
                    else:
                        z_max = z_0 + 1
                        z_min = z_1
                else:  # go to the north or south
                    x_max = x_0 + d_line + 1
                    x_min = x_0 - d_line
                    if y_0 < y_1:  # go to the north
                        y_max = y_1 + d_line + 1
                        y_min = y_0 - d_line
                    else:  # go to the south
                        y_max = y_0 + d_line + 1
                        y_min = y_1 - d_line
                    z_max = z_0 + 1
                    z_min = z_0

                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]
                if z_min < 0:
                    z_min = 0
                if z_max > self.grid_size[2]:
                    z_max = self.grid_size[2]

                for x_i in range(x_max - x_min):
                    for y_i in range(y_max - y_min):
                        for z_i in range(z_max - z_min):
                            hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                            self.occupied_coord[str(hide_pos)] -= 1000
                            if self.occupied_coord[str(hide_pos)] <= 0:
                                del self.occupied_coord[str(hide_pos)]
            elif direct[1] == 0:  # go to the east or west
                if x_0 < x_1:  # go to the east
                    x_max = x_1 + d_line + 1
                    x_min = x_0 - d_line
                else:  # go to the west
                    x_max = x_0 + d_line + 1
                    x_min = x_1 - d_line
                y_max = y_0 + d_line + 1
                y_min = y_0 - d_line

                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]

                for x_i in range(x_max - x_min):
                    for y_i in range(y_max - y_min):
                        hide_pos = [x_min + x_i, y_min + y_i, z_0]
                        self.occupied_coord[str(hide_pos)] -= 1000
                        if self.occupied_coord[str(hide_pos)] <= 0:
                            del self.occupied_coord[str(hide_pos)]
            else:  # go diagonally
                if direct[0] > 0:
                    delta_x = 1
                    if direct[1] > 0:  # go to the north-east
                        delta_y = 1
                    else:  # go to the south-east
                        delta_y = -1
                else:
                    delta_x = -1
                    if direct[1] > 0:  # go to the north-west
                        delta_y = 1
                    else:  # go to the south-west
                        delta_y = -1
                while x_0 != x_1 + delta_x:
                    x_max = x_0 + d_line + 1
                    x_min = x_0 - d_line
                    y_max = y_0 + d_line + 1
                    y_min = y_0 - d_line

                    if x_min < 0:
                        x_min = 0
                    if x_max > self.grid_size[0]:
                        x_max = self.grid_size[0]
                    if y_min < 0:
                        y_min = 0
                    if y_max > self.grid_size[1]:
                        y_max = self.grid_size[1]
                    # Set the cost of the routed path Large Enough
                    for x_i in range(x_max - x_min):
                        for y_i in range(y_max - y_min):
                            hide_pos = [x_min + x_i, y_min + y_i, z_0]
                            self.occupied_coord[str(hide_pos)] -= 1000
                            if self.occupied_coord[str(hide_pos)] <= 0:
                                del self.occupied_coord[str(hide_pos)]

                    x_0 += delta_x
                    y_0 += delta_y

    def update(self, single_route: list, net_i: int):
        """
        Update the route and cost in the routing environment

        :param single_route: the routing path
        :param net_i:
        :return:
        """
        if single_route:
            route_result = self.get_route(single_route)

            self.netlist[net_i].two_pin_net_route_list.append(route_result)

            if len(self.netlist[net_i].two_pin_net_route_list) >= self.netlist[net_i].two_pin_net_num:
                # the multi-pin net is routed completely
                for i in range(self.netlist[net_i].pad_num):
                    self.add_pin_effect(self.netlist[net_i].pad_list[i].occupied_coord_set)

                for route in self.netlist[net_i].two_pin_net_route_list:
                    distance = [self.netlist[net_i].net_class.track_width,
                                self.netlist[net_i].net_class.microvia_diameter]
                    self.add_occupied_coord(route, distance)

    # TODO
    # def breakup(self):
    #     # breakup the routing
    #     if self.episode > 0 and self.twoPinNet_i == 0:
    #         self.old_netPinRoute = self.route_combo[self.multiPinNet_i]
    #         self.route_combo[self.multiPinNet_i] = []
    #         for old_route in self.old_netPinRoute:
    #             distance = [self.netlist[self.multiPinNet_i].net_class.track_width,
    #                         self.netlist[self.multiPinNet_i].net_class.microvia_diameter]
    #             self.del_occupied_coord(old_route, distance)

    def reset(self, net_i: int):
        """
        Reset the routing environment and prepare the next routing

        :return:
        """
        for i in range(self.netlist[net_i].pad_num):
            self.eliminate_pin_effect(self.netlist[net_i].pad_list[i].occupied_coord_set)

    def merge_route(self) -> list:
        """
        Generate the routing paths to segments
        :return: the list of route segments
        """
        merge_route_combo = []
        for net in self.netlist:
            route_list = net.two_pin_net_route_list
            merge_route_list = []
            for route in route_list:
                for i in range(len(route) - 1):
                    start = [to_real_coord(route[i][0]), to_real_coord(route[i][1]), route[i][2]]
                    end = [to_real_coord(route[i + 1][0]), to_real_coord(route[i + 1][1]), route[i + 1][2]]
                    merge_route_list.append([start, end])
            merge_route_combo.append(merge_route_list)

        return merge_route_combo
