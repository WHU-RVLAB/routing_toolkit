# Desc: visualization
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import ImageGrab

from rtools.modeling.grid_env import to_real_coord
from tkinter import *
import enum
import heapq
import time
import _thread


##TODO: add polygen visualiazation from matplotlib.patches import Polygon as MatplotlibPolygon


def draw_routing(grid_env):
    layer = grid_env.grid_size[2]
    ##################################
    for i in range(layer):
        x = to_real_coord(grid_env.grid_size[0])
        y = to_real_coord(grid_env.grid_size[1])
        visualizer = Visualizer(x, y)
        ####################################################
        for pad in grid_env.pad_obstacles:
            if pad.position[2] == i:
                position_x = to_real_coord(pad.position[0])
                position_y = to_real_coord(pad.position[1])
                shape = pad.shape
                size0 = to_real_coord(pad.hsize[0])
                size1 = to_real_coord(pad.hsize[1])
                abj_list = []
                # netID    = pad.pad_info.position
                if shape == 'rect':
                    obj = VisualObject(position_x - size0, position_y - size1, 'rectangle', size0 * 2,
                                       size1 * 2, 'blue')
                    abj_list.append(obj)
                    visualizer.add_object(abj_list)
                if shape == 'circle' or shape == 'roundrect' or shape == 'oval':
                    obj = VisualObject(position_x, position_y, 'circle', size0, size1, 'blue')
                    abj_list.append(obj)
                    visualizer.add_object(abj_list)
        ######################################net_pad
        ##################    通孔 两层都要 ，else
        for pad_net in grid_env.netlist:
            for pad1 in pad_net.pad_list:
                if pad1.position[2] == i or pad1.type == 'thru_hole':
                    if (pad1.type == 'thru_hole'):
                        color = 'black'
                    else:
                        color = 'red'
                    position_x = to_real_coord(pad1.position[0])
                    position_y = to_real_coord(pad1.position[1])
                    shape = pad1.shape
                    size0 = to_real_coord(pad1.hsize[0])
                    size1 = to_real_coord(pad1.hsize[1])
                    abj_list = []
                    # netID    = pad.pad_info.position
                    if shape == 'rect':
                        obj = VisualObject(position_x - size0, position_y - size1, 'rectangle', size0 * 2,
                                           size1 * 2, color)
                        abj_list.append(obj)
                        visualizer.add_object(abj_list)
                    if shape == 'circle' or shape == 'roundrect' or shape == 'oval':
                        obj = VisualObject(position_x, position_y, 'circle', size0, size1, color)
                        abj_list.append(obj)
                        visualizer.add_object(abj_list)
        hell_list = []
        cnt = 0
        for net in grid_env.netlist:
            for route in net.two_pin_net_route_list:
                past_pos = None
                x_set = []
                y_set = []
                for pos in route:
                    cur_pos = pos
                    if pos != route[0]:
                        if cur_pos[2] != past_pos[2]:
                            size_hell = to_real_coord(net.net_class.microvia_diameter)
                            obj = VisualObject(to_real_coord(pos[0]), to_real_coord(pos[1]), 'hole', size_hell,
                                               size_hell, 'green')
                            hell_list.append(obj)
                            visualizer.add_object(hell_list)
                    if pos[2] == i:
                        x = to_real_coord(pos[0])
                        x_set.append(x)
                        y = to_real_coord(pos[1])
                        y_set.append(y)
                    else:
                        width = net.net_class.track_width
                        visualizer.draw_path(x_set, y_set, width)
                        x_set = []
                        y_set = []
                    past_pos = cur_pos
                width = net.net_class.track_width
                visualizer.draw_path(x_set, y_set, width)
        name = []
        name = 'layer' + str(i + 1)
        visualizer.ax.set_title(name)
        visualizer.visualize_layout()


class VisualObject:
    def __init__(self, x, y, shape='rectangle', size1=None, size2=None, color='blue'):
        self.x = x
        self.y = y
        self.shape = shape
        self.size1 = size1
        self.size2 = size2
        self.color = color


class Visualizer:
    def __init__(self, canvas_width, canvas_height):
        self.objects = []
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.fig, self.ax = plt.subplots()
        self.current_frame = []

    def add_object(self, visual_object):
        self.objects.extend(visual_object)

    def frame_objects(self):
        # config the canvas
        self.ax.set_aspect('equal')
        self.ax.set_xlim(0, self.canvas_width)  # Set canvas limits
        self.ax.set_ylim(0, self.canvas_height)
        # place the objects
        for obj in self.objects:
            if obj.shape == 'rectangle':
                rect = patches.Rectangle((obj.x, obj.y), obj.size1, obj.size2, color=obj.color)
                self.current_frame.append(self.ax.add_patch(rect))
            elif obj.shape == 'circle':
                circle = patches.Circle((obj.x, obj.y), obj.size1, color=obj.color)
                self.current_frame.append(self.ax.add_patch(circle))
            elif obj.shape == 'hole':
                circle = patches.Circle((obj.x, obj.y), obj.size1, color=obj.color, fill=False)
                self.current_frame.append(self.ax.add_patch(circle))
            elif obj.shape == 'polygon':
                polygon = patches.Polygon(obj.size1)
                self.current_frame.append(self.ax.add_patch(polygon))

    def draw_path(self, x, y, width):
        self.ax.plot(x, y, 'green', lw=width)

    def visualize_layout(self, framing_on=False, frames=None, save_gif=True):
        if not framing_on:
            self.frame_objects()
            plt.show()
        else:
            if frames == None:
                raise Exception("framing is on, please input frames[]")
            else:
                import matplotlib.animation as animation
                ani = animation.ArtistAnimation(self.fig, frames, interval=200, blit=True)
                if save_gif:
                    ani.save('./logs/output.gif')
                plt.show()


# example usage
'''''
if __name__ == '__main__':

    def print_config(visualizer, framing_on):
        print("***************************************************")
        print("canvas_width = ", visualizer.canvas_width)
        print("canvas_height = ", visualizer.canvas_height)
        print("faming = ", framing_on)
        print("***************************************************")

#example objects
    obj1 = VisualObject(2, 3, 'rectangle', 2,2, 'red')
    obj2 = VisualObject(5, 5, 'circle', 1.5, 1, 'green')
    obj3 = VisualObject(7, 2, 'rectangle', 1,1, 'blue')
    obj_list = [obj1, obj2, obj3]

#visualize one frame
    visualizer1 = Visualizer(20,10)
    visualizer1.add_object(obj_list)

    print_config(visualizer1, False)
    visualizer1.visualize_layout()
'''''


class PointState(enum.Enum):
    # 障碍物
    BARRIER = 'black'
    # 未使用
    UNUSED = 'white'
    # 在open list的方格
    OPEN = 'gold'
    # 在close list的方格
    CLOSED = 'darkgray'
    # 路径
    PATH = 'orangered'


class MiniMap:
    class Point:
        def __init__(self, x, y, f, g, father, state, rectangle):
            # x坐标
            self.x = x
            # y坐标
            self.y = y
            # f = g + h, h为预估代价，这里使用曼哈顿距离
            self.f = f
            # 从寻路起点到这个点的代价
            self.g = g
            # 父节点
            self.father = father
            # 当前点状态
            self.state = state
            # 当前点对应画布上的矩形
            self.rectangle = rectangle

        # 重写比较，方便堆排序
        def __lt__(self, other):
            if self.f < other.f:
                return True
            else:
                return False

    def __init__(self, *args):
        # 高
        # self.height = args[0]
        # 宽
        # self.width = args[1]
        # 方格尺寸
        self.grid_env = args[0]
        self.size = args[1]
        # 起点
        # self.start = args[3]
        # 终点
        # self.end = args[4]
        # 每次绘制的延迟时间
        self.delay = args[2]

        self.root = Tk()
        self.root.title('navigation')
        self.canvas = Canvas(self.root, width=self.grid_env.grid_size[0] * self.size + 3,
                             height=self.grid_env.grid_size[1] * self.size + 3)
        # 生成方格集合
        self.points = self.generatePoints()
        # 生成网格
        self.generateMesh()
        self.canvas.pack(side=TOP, expand=YES, fill=BOTH)
        self.draw_abs()
        self.root.resizable(False, False)
        self.getter(self.canvas)
        self.root.mainloop()

    def generatePoints(self):
        """
        初始化绘制用的方格集合
        设置每个方格的状态和对应的矩形
        """
        points = [[self.Point(x, y, 0, 0, None, PointState.UNUSED.value,
                              self.canvas.create_rectangle((x * self.size + 3, y * self.size + 3),
                                                           ((x + 1) * self.size + 3, (y + 1) * self.size + 3),
                                                           fill=PointState.UNUSED.value)) for y in
                   range(self.grid_env.grid_size[1])]
                  for x in range(self.grid_env.grid_size[0])]
        return points

    def generateMesh(self):
        """
        绘制网格
        """
        for i in range(self.grid_env.grid_size[1] + 1):
            self.canvas.create_line((3, i * self.size + 3),
                                    (self.grid_env.grid_size[0] * self.size + 3, i * self.size + 3))
        for i in range(self.grid_env.grid_size[0] + 1):
            self.canvas.create_line((i * self.size + 3, 3),
                                    (i * self.size + 3, self.grid_env.grid_size[1] * self.size + 3))

    def draw_abs(self):
        for coord0 in self.grid_env.occupied_coord:
            coord_list = eval(coord0)
            print(coord_list)
            x = coord_list[0]
            y = coord_list[1]
            self.changeState(self.points[x][y], PointState.BARRIER.value)

    def changeState(self, point, state):
        """
        修改某一point的状态
        """
        point.state = state
        self.canvas.itemconfig(point.rectangle, fill=state)

    def getter(self, widget):
        widget.update()
        x = self.root.winfo_rootx() + widget.winfo_x()
        y = self.root.winfo_rooty() + widget.winfo_y()
        x1 = x + widget.winfo_width()
        y1 = y + widget.winfo_height()
        ImageGrab.grab().crop((x, y, x1, y1)).save("first.jpg")

