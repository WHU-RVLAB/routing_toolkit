from math import ceil

from rtools.utils import diagonal_distance_3d, circle_space_set, oval_space_set, rect_space_set


def to_grid_coord_round_down(real_coord):
    return int(real_coord * 3 / 0.2)  # 0.25


def to_grid_coord_round_up(real_coord):
    return ceil(real_coord * 3 / 0.2)  # 0.25


def to_real_coord(grid_coord):
    return grid_coord / 3 * 0.2  # 0.25


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
        elif self.shape == 'oval':
            for z_i in range(z_max - z_min):
                self.occupied_coord_set = \
                    self.occupied_coord_set | oval_space_set([x_coord, y_coord, z_i], self.hsize[0], self.hsize[1])
                self.pad_coord_set = \
                    self.pad_coord_set | oval_space_set([x_coord, y_coord, z_i], self.hsize[0] - 1, self.hsize[1] - 1)
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
    def __init__(self, net_id, net_name, net_class, ignore):
        self.net_id = net_id
        self.net_name = net_name
        self.net_class = net_class
        self.is_ignore = ignore

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


grid_cell_base_cost = {
    'pad': 1000,
    'track': 1000,
    'via': 1000
}


class GridCell:
    def __init__(self, cell_type):
        self.cell_type = cell_type
        self.base_cost = 0
        if cell_type in grid_cell_base_cost:
            self.base_cost = grid_cell_base_cost[cell_type]

    def get_base_cost(self):
        return self.base_cost

    def add_base_cost(self, cell_type):
        if cell_type in grid_cell_base_cost:
            self.base_cost += grid_cell_base_cost[cell_type]

    def del_base_cost(self, cell_type):
        if cell_type in grid_cell_base_cost:
            self.base_cost -= grid_cell_base_cost[cell_type]

    def is_vacant(self):
        if self.base_cost <= 0:
            return True
        else:
            return False


class GridEnv:
    def __init__(self, board_area, layers_, net_num, net_list, net_class, pad_obstacles):
        """
        Create the grid environment based on dataset

        :param board_area:
        :param layers_:
        :param net_num:
        :param net_list:
        :param net_class:
        :param pad_obstacles:
        """
        self.grid_size = [to_grid_coord_round_down(board_area[0] - board_area[1]),
                          to_grid_coord_round_down(board_area[2] - board_area[3]),
                          len(layers_)]
        """ the size of the grid graph """

        self.net_num = net_num
        """ the number of nets """

        self.netlist, self.net_order = self.load_net_list(net_list, net_class)
        """ a list of the pads from each net """

        self.net_obstacles = self.load_trace_items(net_list, board_area[1], board_area[3], layers_)

        self.pad_obstacles = self.load_pad_obstacles(pad_obstacles)
        """ a list of the pads do not have a net """

        self.occupied_coord = None
        self.generate_occupied_coord()
        """ a dict stores the occupied grids """

    def load_net_list(self, net_list, net_class):
        net_list_tmp = []
        net_order = list(range(self.net_num))
        for i in net_order:
            net_info = net_list[i + 1]
            net = Net(net_info.netID - 1, net_info.netName,
                      NetClass(net_class[net_info.netClass].track_width,
                               net_class[net_info.netClass].microvia_diameter,
                               net_class[net_info.netClass].clearance_with_track,
                               net_class[net_info.netClass].clearance_with_microvia),
                      net_info.is_ignore)

            for pad_info in net_info.padList:
                pad = Pad(pad_info.position, pad_info.layer, pad_info.shape,
                          pad_info.size, pad_info.type, pad_info.netID)
                # add occupied coordinates
                pad.generate_coord_set(self.grid_size)
                net.pad_list.append(pad)
            net.pad_num = len(net.pad_list)

            net.generate_two_pin_net()

            net_list_tmp.append(net)

        return net_list_tmp, net_order

    def load_trace_items(self, net_list, x0, y0, layer_dict):
        trace_items = {
            'route': [],
            'distance': []
        }

        for i in range(self.net_num):
            net_info = net_list[i + 1]
            if net_info.is_ignore:
                for item_info in net_info.trace_items['segment']:
                    z = layer_dict[item_info.layer]
                    start = [to_grid_coord_round_down(item_info.start.X - x0),
                             to_grid_coord_round_down(item_info.start.Y - y0),
                             z]
                    end = [to_grid_coord_round_down(item_info.end.X - x0),
                           to_grid_coord_round_down(item_info.end.Y - y0),
                           z]

                    dx = abs(start[0] - end[0])
                    dy = abs(start[1] - end[1])
                    if dx != dy:
                        if dx < dy / 2:
                            end[0] = start[0]
                        elif dy < dx / 2:
                            end[1] = start[1]
                    route = [start, end]
                    distance = [to_grid_coord_round_up(item_info.width / 2), 0]

                    trace_items['route'].append(route)
                    trace_items['distance'].append(distance)

                for item_info in net_info.trace_items['via']:
                    z_list = []

                    for layer in item_info.layers:
                        z_list.append(layer_dict[layer])
                    z_min = min(z_list)
                    z_max = max(z_list)
                    start = [to_grid_coord_round_down(item_info.position.X - x0),
                             to_grid_coord_round_down(item_info.position.Y - y0),
                             z_min]
                    end = [to_grid_coord_round_down(item_info.position.X - x0),
                           to_grid_coord_round_down(item_info.position.Y - y0),
                           z_max]
                    route = [start, end]

                    distance = [0, to_grid_coord_round_up(item_info.size / 2)]

                    trace_items['route'].append(route)
                    trace_items['distance'].append(distance)

                for item_info in net_info.trace_items['arc']:
                    pass

        return trace_items

    def load_pad_obstacles(self, pad_obstacles):
        pad_list_tmp = []
        for pad_info in pad_obstacles:
            pad = Pad(pad_info.position, pad_info.layer, pad_info.shape,
                      pad_info.size, pad_info.type, pad_info.netID)
            # add occupied coordinates
            pad.generate_coord_set(self.grid_size)
            pad_list_tmp.append(pad)

        return pad_list_tmp

    def generate_occupied_coord(self):
        """
        Create an initial dict which contain the obstacle grid

        :return: the dict of the occupied grids
        """
        self.occupied_coord = {}

        # Set grid graph refer to boundary list
        # TODO

        # Set the pads with a net
        for i in range(self.net_num):
            net = self.netlist[i]
            for j in range(net.pad_num):
                pad = net.pad_list[j]
                self.add_pad_effect(pad.occupied_coord_set)

        # Set obstacle pad
        for obs_pad in self.pad_obstacles:
            self.add_pad_effect(obs_pad.occupied_coord_set)

        # Set net obstacles
        i = 0
        net_obstacles = self.net_obstacles
        for route in net_obstacles['route']:
            distance = net_obstacles['distance'][i]
            self.add_trace_coord(route, distance)
            i += 1

    def add_occupied_coord(self, hide_pos, cell_type):
        if hide_pos in self.occupied_coord:
            self.occupied_coord[hide_pos].add_base_cost(cell_type)
        else:
            self.occupied_coord[hide_pos] = GridCell(cell_type)  # Large Enough

    def del_occupied_coord(self, hide_pos, cell_type):  # ILLEGAL POS ? TODO
        self.occupied_coord[hide_pos].del_base_cost(cell_type)
        if self.occupied_coord[hide_pos].is_vacant():
            del self.occupied_coord[str(hide_pos)]

    def add_pad_effect(self, pad_size_set: set):
        """
        Add the base cost of the grids of pad_size_set to the occupied grids

        :param pad_size_set: the set of the coordinates of the pad
        :return:
        """
        # for hide_pos in pad_size_set:
        #     if hide_pos in self.occupied_coord:
        #         self.occupied_coord[hide_pos] += 1000
        #     else:
        #         self.occupied_coord[hide_pos] = 1000  # Large Enough
        for hide_pos in pad_size_set:
            self.add_occupied_coord(hide_pos, 'pad')

    def del_pad_effect(self, pad_size_set: set):
        """
        Delete the base cost of the grids of pad_size_set to the occupied grids

        :param pad_size_set: the set of the coordinates of the pad
        :return:
        """
        # for hide_pos in pad_size_set:
        #     self.occupied_coord[hide_pos] -= 1000
        #     if self.occupied_coord[hide_pos] <= 0:
        #         del self.occupied_coord[str(hide_pos)]
        for hide_pos in pad_size_set:
            self.del_occupied_coord(hide_pos, 'pad')

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

    def add_trace_coord(self, route: list, distance: list):
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
                via_flag = False
                if direct[1] == 0:  # go through a via
                    via_flag = True
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

                if via_flag:
                    for x_i in range(x_max - x_min):
                        for y_i in range(y_max - y_min):
                            for z_i in range(z_max - z_min):
                                hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                                # if str(hide_pos) in self.occupied_coord:
                                #     self.occupied_coord[str(hide_pos)] += 1000
                                # else:
                                #     self.occupied_coord[str(hide_pos)] = 1000  # Large Enough
                                self.add_occupied_coord(str(hide_pos), 'via')
                else:
                    for x_i in range(x_max - x_min):
                        for y_i in range(y_max - y_min):
                            for z_i in range(z_max - z_min):
                                hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                                # if str(hide_pos) in self.occupied_coord:
                                #     self.occupied_coord[str(hide_pos)] += 1000
                                # else:
                                #     self.occupied_coord[str(hide_pos)] = 1000  # Large Enough
                                self.add_occupied_coord(str(hide_pos), 'track')
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
                            # if str(hide_pos) in self.occupied_coord:
                            #     self.occupied_coord[str(hide_pos)] += 1000
                            # else:
                            #     self.occupied_coord[str(hide_pos)] = 1000  # Large Enough
                            self.add_occupied_coord(str(hide_pos), 'track')
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
                            # if str(hide_pos) in self.occupied_coord:
                            #     self.occupied_coord[str(hide_pos)] += 1000
                            # else:
                            #     self.occupied_coord[str(hide_pos)] = 1000  # Large Enough
                            self.add_occupied_coord(str(hide_pos), 'track')

                    x_0 += delta_x
                    y_0 += delta_y

    def del_trace_coord(self, old_route: list, distance: list):
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
                via_flag = False
                if direct[1] == 0:  # go through a via
                    via_flag = True
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

                if via_flag:
                    for x_i in range(x_max - x_min):
                        for y_i in range(y_max - y_min):
                            for z_i in range(z_max - z_min):
                                hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                                # self.occupied_coord[str(hide_pos)] -= 1000
                                # if self.occupied_coord[str(hide_pos)] <= 0:
                                #     del self.occupied_coord[str(hide_pos)]
                                self.del_occupied_coord(str(hide_pos), 'via')
                else:
                    for x_i in range(x_max - x_min):
                        for y_i in range(y_max - y_min):
                            for z_i in range(z_max - z_min):
                                hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                                # self.occupied_coord[str(hide_pos)] -= 1000
                                # if self.occupied_coord[str(hide_pos)] <= 0:
                                #     del self.occupied_coord[str(hide_pos)]
                                self.del_occupied_coord(str(hide_pos), 'track')
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
                        # self.occupied_coord[str(hide_pos)] -= 1000
                        # if self.occupied_coord[str(hide_pos)] <= 0:
                        #     del self.occupied_coord[str(hide_pos)]
                        self.del_occupied_coord(str(hide_pos), 'track')
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
                            # self.occupied_coord[str(hide_pos)] -= 1000
                            # if self.occupied_coord[str(hide_pos)] <= 0:
                            #     del self.occupied_coord[str(hide_pos)]
                            self.del_occupied_coord(str(hide_pos), 'track')

                    x_0 += delta_x
                    y_0 += delta_y

    def reset(self, net_i: int):
        """
        Reset the routing environment and prepare the next routing

        :return:
        """
        for i in range(self.netlist[net_i].pad_num):
            self.del_pad_effect(self.netlist[net_i].pad_list[i].occupied_coord_set)

    def breakup(self, net_id):
        # breakup the routing
        net = self.netlist[net_id]
        net_class = net.net_class
        old_two_pin_net_route_list = net.two_pin_net_route_list
        net.two_pin_net_route_list = []
        for old_route in old_two_pin_net_route_list:
            distance = [net_class.track_width, net_class.microvia_diameter]
            self.del_trace_coord(old_route, distance)

        return old_two_pin_net_route_list

    def update_net(self, single_route: list, net_i: int):
        """
        Update the route and cost in the routing environment

        :param single_route: the routing path
        :param net_i:
        :return:
        """
        if single_route:
            route_result = self.get_route(single_route)

            self.netlist[net_i].two_pin_net_route_list.append(route_result)

        else:
            self.netlist[net_i].two_pin_net_route_list.append(single_route)

    def recovery_route(self, old_two_pin_net_route_list, net_id):
        self.netlist[net_id].two_pin_net_route_list = old_two_pin_net_route_list

    def update(self, net_i: int):
        """
        Update the route and cost in the routing environment

        :param net_i:
        :return:
        """
        # the multi-pin net is routed completely

        for i in range(self.netlist[net_i].pad_num):
            self.add_pad_effect(self.netlist[net_i].pad_list[i].occupied_coord_set)

        for route in self.netlist[net_i].two_pin_net_route_list:
            if route:
                distance = [int(self.netlist[net_i].net_class.track_width / 2),
                            int(self.netlist[net_i].net_class.microvia_diameter / 2)]
                self.add_trace_coord(route, distance)

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
