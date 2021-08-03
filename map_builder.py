from enum import Flag
from direction import Direction
import numpy as np

class BuilderState(Flag):
    EMPTY    = 0
    BUILDING = 1
    FINISHED = 2

class MapBuilder:
    def __init__(self):
        self._nodes_count = 0
        self._edges_count = 0

        self.state = BuilderState.EMPTY

        self.allowable_error = 0.8
        self.tile_size       = 0.5

        self._previous_node      = None
        self._previous_direction = None

        self._nodes_locations = {}
        self.road_map = {}


    def _get_node_by_pos(self, pos):
        if not isinstance(pos, np.ndarray):
            raise ValueError("pos meant to have a numpy.ndarray type")

        for node_pos in self._nodes_locations.keys():
            # if some node has same position
            if (abs(node_pos - pos) < np.ones(2) * self.tile_size * self.allowable_error).all():
                return self._nodes_locations[tuple(node_pos)]
        return None

    def _connect_existed(self, node, direction):
        if self._previous_node is None and self._previous_direction is None:
            return
        
        direction = direction.invert()
        self.road_map[node][direction]['edge'] = self.road_map[self._previous_node][self._previous_direction]['edge']
        self.road_map[node][direction]['vertex'] = self._previous_node
        self.road_map[self._previous_node][self._previous_direction]['vertex'] = node
        

    def _create_node(self, paths, node_pos, path):
        self._nodes_count += 1
        
        self.road_map[self._nodes_count] = {possible_path:{'edge':None, 'vertex':None} for possible_path in paths}
        # self.road_map[self._nodes_count][path]['edge'] = self._edges_count

        self._nodes_locations[tuple(node_pos)] = self._nodes_count
        
    def _choose_path(self, paths, node, heading):
        if node is None:
            return max([direction - heading + Direction.UP for direction in paths])
        return max([direction - heading + Direction.UP 
                    for direction in self.road_map[node].keys() if self.road_map[node][direction]['vertex'] is None])

    def update(self, paths, pos, direction):
        if not isinstance(pos, np.ndarray):
            raise ValueError("pos meant to have a numpy.ndarray type")
        if not isinstance(direction, Direction):
            raise ValueError("direction meant to have a Direction type")
        if self.state == BuilderState.FINISHED:
            return Direction.DOWN

        self.state = BuilderState.BUILDING

        pos += direction.get_normalized() * self.tile_size / 2
        node = self._get_node_by_pos(pos)

        path = self._choose_path(paths, node, direction)
        absolute_path = path + direction + Direction.UP

        self._edges_count += 1

        if node is None:
            self._create_node(paths, pos, absolute_path)
            node = self._nodes_count
        
        self.road_map[node][absolute_path]['edge'] = self._edges_count
        self._connect_existed(node, direction)

        self._previous_node      = node
        self._previous_direction = absolute_path

        if path == Direction.DOWN:
            self.state = BuilderState.FINISHED

        return path
    
    def print_map(self):
        d = self.road_map
        for node in d.keys():
            print(node, ':')
            for direction in d[node].keys():
                print('-', direction, ':', d[node][direction])

    def get_nodes_locations(self):
        return self._nodes_locations

    def get_map(self):
        if self.state == BuilderState.FINISHED:
            return self.road_map
        return None

if __name__ == "__main__":
    mp = MapBuilder()

    mp.tile_size = 3

    #======# 1 #======#
    data = (
        (Direction.LEFT, Direction.UP, Direction.RIGHT),
        np.array([6.0, 2.0]),
        Direction.RIGHT
    )
    mp.update(*data)
    #======# 2 #======#
    data = (
        (Direction.DOWN, Direction.LEFT, Direction.UP, Direction.RIGHT),
        np.array([10.0, 8.0]),
        Direction.LEFT
    )
    mp.update(*data)
    #======# 3 #======#
    data = (
        (Direction.DOWN, Direction.UP, Direction.RIGHT),
        np.array([2.0, 10.0]),
        Direction.DOWN
    )
    mp.update(*data)
    #======# 4 #======#
    data = (
        (Direction.LEFT, Direction.UP, Direction.RIGHT),
        np.array([6.0, 2.0]),
        Direction.RIGHT
    )
    mp.update(*data)
    #======# 5 #======#
    data = (
        (Direction.DOWN, Direction.LEFT, Direction.UP, Direction.RIGHT),
        np.array([8.0, 6.0]),
        Direction.UP
    )
    mp.update(*data)
    #======# 6 #======#
    data = (
        (Direction.DOWN, Direction.UP, Direction.RIGHT),
        np.array([5.0, 8.0]),
        Direction.LEFT
    )
    mp.update(*data)
    #======# 7* #======#
    data = (
        (Direction.DOWN, Direction.LEFT, Direction.UP, Direction.RIGHT),
        np.array([8.0, 6.0]),
        Direction.UP
    )
    print(mp.update(*data))
    
    mp.print_map()

    print(mp.get_nodes_locations())
