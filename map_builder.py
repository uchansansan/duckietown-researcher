from enum import FlagEnum
from BarcodeClassifier import Direction

class BuilderState(Flag):
    EMPTY    = 0
    BUILDING = 1
    FINISHED = 2

class MapBuilder:
    def __init__(self):
        self._nodes_count = 0
        self._edges_count = 1

        self.state = BuilderState.EMPTY

        self.allowable_error = 0.8
        self.tile_size       = 0.585

        self._previous_node      = None
        self._previous_direction = None

        self._nodes_locations = {}
        self.road_map = {}


    def _get_node_by_pos(self, pos):
        if not isinstance(pos, np.ndarray):
            raise ValueError("pos meant to have a numpy.ndarray type")

        for node_pos in _nodes_location.keys():
            # if some node has same position
            if (abs(node_pos - pos) < np.ones(2) * self.tile_size * self.allowable_error).all():
                return self._nodes_location[node_pos]
        return None

    def _connect_existed(self, node, direction):
        road_map[node][direction]['edge'] = self._edges_count
        road_map[node][direction]['vertex'] = self._previous_node
        road_map[self._previous_node][self._previous_direction]['vertex'] = node

        self._edges_count += 1

        self._previous_direction = direction
        self._previous_node      = node
        

    def _create_node(self, paths, node_pos, direction):
        self._nodes_count += 1
        
        road_map[self._nodes_count] = {path:{'edge':None, 'vertex':None} for path in paths}
        
        if self._previous_direction is not None and self._previous_node is not None:
            self._connect_existed(self._nodes_count, direction)
        road_map[self._nodes_count][direction]['edge'] = self._edges_count

        self._nodes_location[node_pos] = self._nodes_count
        
    def _choose_path(self, paths, node, heading):
        if node is None:
            return max([direction - heading + Direction.UP for direction in paths])
        return max([direction - heading + Direction.UP for direction in road_map[node].keys() if direction['vertex'] is None])

    def update(self, paths, pos, direction):
        if not isinstance(pos, np.ndarray):
            raise ValueError("pos meant to have a numpy.ndarray type")
        if not isinstance(direction, Direction):
            raise ValueError("direction meant to have a Direction type")
        if self.state == BuilderState.FINISHED:
            return

        self.state = BuilderState.BUILDING

        pos += direction.get_normalized() * self.tile_size
        node = self._get_node_by_pos(pos)

        path = self._choose_path(paths, node, direction)

        if node is None:
            self._create_node(paths, pos, path)
        else:
            self._connect_existed(node)

        return path
    
    def get_map(self):
        if self.state == BuilderState.FINISHED:
            return self.road_map
        return None

if __name__ == "__main__":
    mp = MapBuilder()

