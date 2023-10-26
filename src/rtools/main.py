import argparse

from rtools.dataset import Dataset
from rtools.modeling import GridEnv
from rtools.router import AStarRouter


def routing_arguments():
    parser = argparse.ArgumentParser('AStarSolver')
    parser.add_argument('--episode', type=int, dest='episode', default=1)  # TODO
    parser.add_argument('--log', type=str, dest='log', default="log.txt")  # TODO
    parser.add_argument('--trace', type=bool, dest='trace', default=True)  # TODO
    parser.add_argument('--kicad_pcb', type=str, dest='kicad_pcb',
                        default="example/pcb/bench2/bm2.unrouted.kicad_pcb")
    parser.add_argument('--kicad_pro', type=str, dest='kicad_pro',
                        default="example/pcb/bench2/bm2.unrouted.kicad_pro")
    parser.add_argument('--save_file', type=str, dest='save_file',
                        default="logs/bm2.routed.kicad_pcb")

    return parser.parse_args()


if __name__ == '__main__':
    # get the routing arguments
    arg = routing_arguments()

    # get the file name
    pcb_file = arg.kicad_pcb
    pro_file = arg.kicad_pro
    save_file = arg.save_file

    # build the dateset from the file
    dataset = Dataset(pcb_file, pro_file, save_file)
    # load the routing data
    dataset.load()

    tmp = dataset.netList[1]
    dataset.netList[1] = dataset.netList[34]
    dataset.netList[34] = tmp

    # build the model from dataset
    model = GridEnv(dataset.board_area, dataset.layer_num,
                    dataset.netNum, dataset.netList, dataset.netClass, dataset.pad_obstacles)

    # build the router based on the model
    router = AStarRouter(model)
    # routing...
    router.run(1)  # arg.episode

    # write back the routing result
    dataset.store_route(model.merge_route())

    print('route time = {} s'.format(router.route_time))
