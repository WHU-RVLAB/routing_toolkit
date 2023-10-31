from argparse import ArgumentParser
from numpy import mean

from rtools.dataset import Dataset
from rtools.modeling.grid_env import GridEnv
from rtools.router import AStarRouter


def routing_arguments():
    parser = ArgumentParser('AStarSolver')
    parser.add_argument('--episode', type=int, dest='episode', default=1)  # TODO
    parser.add_argument('--log', type=str, dest='log', default="log.txt")  # TODO
    parser.add_argument('--trace', type=bool, dest='trace', default=True)  # TODO
    parser.add_argument('--kicad_dir', type=str, dest='kicad_dir', default="example/pcb/bench7_routed/")
    parser.add_argument('--kicad_pcb', type=str, dest='kicad_pcb', default="bm7.routed.kicad_pcb")
    parser.add_argument('--kicad_pro', type=str, dest='kicad_pro', default="bm7.routed.kicad_pro")
    parser.add_argument('--save_file', type=str, dest='save_file', default="bm7.rerouted.kicad_pcb")

    return parser.parse_args()


if __name__ == '__main__':
    # get the routing arguments
    arg = routing_arguments()

    # get the file name
    pcb_dir = arg.kicad_dir
    pcb_file = pcb_dir + arg.kicad_pcb
    pro_file = pcb_dir + arg.kicad_pro
    save_file = arg.save_file

    # build the dateset from the file
    dataset = Dataset(pcb_dir, pcb_file, pro_file, save_file)
    # load the routing data
    dataset.load()

    # tmp = dataset.netList[1]
    # dataset.netList[1] = dataset.netList[14]
    # dataset.netList[14] = tmp

    # tmp = dataset.netList[2]
    # dataset.netList[2] = dataset.netList[15]
    # dataset.netList[15] = tmp

    # build the model from dataset
    model = GridEnv(dataset.board_area, dataset.layers_,
                    dataset.netNum, dataset.netList, dataset.netClass, dataset.pad_obstacles)

    # build the router based on the model
    router = AStarRouter(model)
    # routing...
    router.run(1)  # arg.episode

    # write back the routing result
    route_combo = model.merge_route()
    # tmp = route_combo[0]
    # route_combo[0] = route_combo[13]
    # route_combo[13] = tmp

    # tmp = route_combo[1]
    # route_combo[1] = route_combo[14]
    # route_combo[14] = tmp
    dataset.store_route(route_combo)

    dataset.save()

    print('\nroute time = {} s'.format(router.route_time))
    # print('expend time (ns) : {}'.format(router.expend_time))
    print('avg expend time : {} ns'.format(mean(router.expend_time)))
    print('sum expend time : {} s'.format(router.sum_expend_time / 1000000000))
    print('get neighbors time : {} s'.format(router.get_neighbors_time / 1000000000))
    print('calculate space cost time : {} s'.format(router.calculate_space_cost_time / 1000000000))
    print('add neighbors time : {} s'.format(router.add_neighbors_time / 1000000000))
    print('routing cost : {}'.format(router.route_cost))
    print('episode cost : {}'.format(router.episode_cost))
