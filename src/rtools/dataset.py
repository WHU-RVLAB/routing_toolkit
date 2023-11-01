from math import cos, sin, pi
import os

from kiutils.board import Board
from kiutils.items.brditems import Segment, Via, Arc
from kiutils.items.common import Position

from rtools.kicad_parser.kicad_pro import KiCadPro


class Pad:
    """
    Pad in dataset
    store the Pad data of the file
    """
    def __init__(self, pos, layer, shape, size, pad_type, net_id):
        self.position = pos
        self.layer = layer
        self.shape = shape
        self.size = size
        self.type = pad_type
        self.netID = net_id


# TODO
class TraceItem:
    pass


class Net:
    def __init__(self, net_id, net_name, net_class, ignore):
        self.netID = net_id
        self.netName = net_name
        self.netClass = net_class
        self.padList = []

        self.is_ignore = ignore

        self.trace_items = {
            'segment': [],
            'via': [],
            'arc': []
        }  # TODO read segment & write segment


class NetClass:
    def __init__(self, track_width, microvia_diameter, microvia_drill, clearance, min_hole_clearance):
        self.track_width = track_width
        self.microvia_diameter = microvia_diameter
        self.microvia_drill = microvia_drill
        self.clearance = clearance

        self.clearance_with_track = clearance + track_width / 2
        if clearance < min_hole_clearance:
            self.clearance_with_microvia = min_hole_clearance + microvia_diameter / 2
        else:
            self.clearance_with_microvia = clearance + microvia_diameter / 2


# TODO
class DesignRules:
    pass


class Dataset:
    def __init__(self, pcb_dir, kicad_pcb, kicad_pro, save_kicad_pcb):
        self.pcb_directory = pcb_dir
        self.pcb_filename = kicad_pcb
        self.pro_filename = kicad_pro
        self.save_filename = save_kicad_pcb
        self.grLines = None

        self.board_area = []

        self.layers = {}
        self.layers_ = {}
        self.layer_num = 0

        self.netList = []
        self.netNum = 0

        self.netClass = {}

        self.pad_obstacles = []

    def load(self):
        board = Board().from_file(self.pcb_filename)
        project = KiCadPro().from_file(self.pro_filename)

        self.grLines = board.graphicItems

        gr_x = []
        gr_y = []
        for gr_line in self.grLines:
            gr_x.append(gr_line.end.X)
            gr_y.append(gr_line.end.Y)
            gr_x.append(gr_line.start.X)
            gr_y.append(gr_line.start.Y)
        self.board_area = [max(gr_x), min(gr_x), max(gr_y), min(gr_y)]

        layers_ = {}
        layers = {}
        i = 0
        for layer in board.layers:
            if layer.type == 'signal':
                layers_[layer.name] = i
                layers[i] = layer.name
            i += 1
        self.layers = layers
        self.layers_ = layers_
        self.layer_num = len(self.layers)

        net_ignore_file = self.pcb_directory + '.netignore'
        net_ignore = []
        if os.path.isfile(net_ignore_file):
            with open(net_ignore_file, "r", encoding='utf-8') as f:  # 打开文本
                net_ignore = f.read()

        self.netList = []
        net_id = 0
        for net in board.nets:
            # parse net_class class
            if net.name != '':
                if net.name not in net_ignore:
                    ignore = False
                else:
                    ignore = True
                board_net = Net(net.number, net.name, project.netSetting.netClassPatterns[net.name], ignore)
                self.netList.append(board_net)

            else:
                board_net = Net(net.number, net.name, None, True)
                self.netList.append(board_net)
            net_id += 1
        # self.netList.pop(0)
        self.netNum = len(self.netList) - 1

        self.pad_obstacles = []
        for footprint in board.footprints:
            for pad in footprint.pads:
                if footprint.position.angle is None:
                    theta = 0
                else:
                    theta = footprint.position.angle * pi / 180
                dx = pad.position.X * cos(theta) + pad.position.Y * sin(theta)
                dy = pad.position.Y * cos(theta) - pad.position.X * sin(theta)
                x = footprint.position.X + dx
                y = footprint.position.Y + dy
                pad_pos = [x - self.board_area[1], y - self.board_area[3], layers_[footprint.layer]]
                if pad.position.angle is None:
                    alpha = 0
                else:
                    alpha = pad.position.angle * pi / 180
                size_x = pad.size.X * cos(alpha) + pad.size.Y * sin(alpha)
                size_y = pad.size.Y * cos(alpha) - pad.size.X * sin(alpha)
                pad_size = [abs(size_x), abs(size_y)]
                pad_shape = pad.shape
                if pad.net:
                    board_pad = Pad(pad_pos, footprint.layer, pad_shape, pad_size, pad.type, pad.net.number)
                    self.netList[pad.net.number].padList.append(board_pad)
                else:
                    board_pad = Pad(pad_pos, footprint.layer, pad_shape, pad_size, pad.type, None)
                    self.pad_obstacles.append(board_pad)

        net_class_real = project.netSetting.classes
        min_hole_clearance = project.board.design_setting.rules.min_hole_clearance
        for net_class in net_class_real:
            self.netClass[net_class] = NetClass(net_class_real[net_class].track_width,
                                                net_class_real[net_class].microvia_diameter,
                                                net_class_real[net_class].microvia_drill,
                                                net_class_real[net_class].clearance,
                                                min_hole_clearance)

        for trace_item in board.traceItems:
            if isinstance(trace_item, Segment):
                self.netList[trace_item.net].trace_items['segment'].append(trace_item)
            elif isinstance(trace_item, Via):
                self.netList[trace_item.net].trace_items['via'].append(trace_item)
            elif isinstance(trace_item, Arc):
                self.netList[trace_item.net].trace_items['arc'].append(trace_item)

        board.to_file()

    # TODO how to generate 'tstamp'
    def store_route(self, merge_route_combo):
        i = 1
        item_id = 0
        for net in merge_route_combo:
            net_info = self.netList[i]
            if not net_info.is_ignore:
                trace_items = {
                    'segment': [],
                    'via': [],
                    'arc': []
                }

                for segment in net:
                    start = [self.board_area[1] + segment[0][0],
                             self.board_area[3] + segment[0][1],
                             self.layers[segment[0][2]]]
                    end = [self.board_area[1] + segment[1][0],
                           self.board_area[3] + segment[1][1],
                           self.layers[segment[1][2]]]

                    start_pos = Position(start[0], start[1])
                    end_pos = Position(end[0], end[1])

                    if start[2] == end[2]:
                        width = self.netClass[net_info.netClass].track_width
                        layer = start[2]
                        item = Segment(start_pos, end_pos, width, layer, False, i, str(item_id))
                        # board.traceItems.append(item)
                        trace_items['segment'].append(item)
                    else:
                        size = self.netClass[net_info.netClass].microvia_diameter
                        drill = self.netClass[net_info.netClass].microvia_drill
                        layers = [self.layers[0], self.layers[1]]
                        item = Via('micro', False, start_pos, size, drill, layers, False, False, False, i, str(item_id))
                        # board.traceItems.append(item)
                        trace_items['via'].append(item)

                    item_id += 1

                net_info.trace_items = trace_items

            i += 1

    def save(self):
        board = Board().from_file(self.pcb_filename)

        board.traceItems.clear()
        trace_items = board.traceItems

        for net in self.netList:
            for item in net.trace_items['segment']:
                trace_items.append(item)
            for item in net.trace_items['via']:
                trace_items.append(item)
            for item in net.trace_items['arc']:
                trace_items.append(item)

        logs_dir = 'logs'
        if not os.path.isdir(logs_dir):
            os.mkdir(logs_dir)
        board.to_file(logs_dir + '/' + self.save_filename)
