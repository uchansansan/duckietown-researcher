from enum import FlagEnum
from BarcodeClassifier import Direction


class BuilderState(Flag):
    EMPTY    = 0
    BUILDING = 1
    FINISHED = 2

class MapBuilder:
    def __init__(self):
        self._nodes_count = 0

        self.state = BuilderState.EMPTY

        self._nodes_location = {}
        self.road_map = {}

    def _connect_existed(self, pos):
        node, direction = self._get_node_by_pos(pos)
        road_map[node][direction] = self._nodes_count
        road_map[self._nodes_count][direction] = node

    def _create_node(self, paths, node_pos):
        self._nodes_count += 1
        road_map[self._nodes_count] = {path:None for path in paths}
        self._nodes_location[self._nodes_count] = node_pos

    def update(self, paths):
        """
        connects new crossroad node to map
        updates MapBuilder's state if needed
        """
        if self.state == BuilderState.FINISHED:
            print("Map was built !!! Use get_map() to get map")
            return

        self.state = BuilderState.BUILDING
    
    def get_map(self):
        """
        get finished map
        """
        if self.state == BuilderState.EMPTY:
            return road_map
        print("Map is not finished !!")
        return None
