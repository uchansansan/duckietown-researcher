from enum import Enum

class BuilderState(Enum):
    EMPTY    = 0
    BUILDING = 1
    FINISHED = 2

class MapBuilder:
    state = BuilderState.EMPTY
    map = {}

    def update(self, paths):
        """
        connects new crossroad node to map
        updates MapBuilder's state if needed
        """
        if state == FINISHED:
            print("Map built !!! Use get_map() to get map")
            return

        state = BUILDING
    
    def get_map(self):
        """
        get finished map
        """
        if state == BuilderState.EMPTY:
            return map
        print("Map is not finished !!")
        return None