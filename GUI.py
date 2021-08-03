import turtle
import networkx as nx

class GUI:

    turtle.left(90)
    screen = turtle.Screen()
    screen.setup(800, 800)
    screen.title('Map')
    turtle.pensize(1)
    frame_skip = 1

    def draw_graph(graph):
        g = nx.Graph()
        


    def draw_path(action):
        """
        This function draw the path of a DuckieBot in a new screen
        :return:
        """
        from_wierd2turtle = 1.1784411471630984
        action *= GUI.frame_skip
        turtle.left(action[1] / from_wierd2turtle)
        turtle.forward(action[0])
        bot_pos = turtle.pos()
        bot_heading = turtle.heading()
        #return bot_pos, bot_heading




