#!/usr/bin/env python
# manual

"""
This script allows you to manually control the simulator or Duckiebot
using the keyboard arrows.
"""
from PIL import Image
import argparse
import sys
from researcher.direction import Direction
from researcher.duckiebot_control import DuckiebotControl
from researcher.barcode_classifier import BarcodeClassifier
from researcher.map_builder import MapBuilder
from researcher.gui import GUI


from cv2 import aruco
import turtle  # ya sosu bibu

import cv2

import gym
import numpy as np
import pyglet
from pyglet.window import key

from gym_duckietown.envs import DuckietownEnv

# from experiments.utils import save_img

parser = argparse.ArgumentParser()
parser.add_argument("--env-name", default=None)
parser.add_argument("--map-name", default="udem1")
parser.add_argument("--distortion", default=False, action="store_true")
parser.add_argument("--camera_rand", default=False, action="store_true")
parser.add_argument("--draw-curve", action="store_true", help="draw the lane following curve")
parser.add_argument("--draw-bbox", action="store_true", help="draw collision detection bounding boxes")
parser.add_argument("--domain-rand", action="store_true", help="enable domain randomization")
parser.add_argument("--dynamics_rand", action="store_true", help="enable dynamics randomization")
parser.add_argument("--frame-skip", default=1, type=int, help="number of frames to skip")
parser.add_argument("--seed", default=1, type=int, help="seed")
args = parser.parse_args()

if args.env_name and args.env_name.find("Duckietown") != -1:
    env = DuckietownEnv(
        seed=args.seed,
        map_name=args.map_name,
        draw_curve=args.draw_curve,
        draw_bbox=args.draw_bbox,
        domain_rand=args.domain_rand,
        frame_skip=args.frame_skip,
        distortion=args.distortion,
        camera_rand=args.camera_rand,
        dynamics_rand=args.dynamics_rand,
    )
else:
    env = gym.make(args.env_name)

env.reset()
env.render()


@env.unwrapped.window.event
def on_key_press(symbol, modifiers):
    """
    This handler processes keyboard commands that
    control the simulation
    """

    if symbol == key.BACKSPACE or symbol == key.SLASH:
        print("RESET")
        env.reset()
        env.render()
    elif symbol == key.PAGEUP:
        env.unwrapped.cam_angle[0] = 0
    elif symbol == key.ESCAPE:
        env.close()
        sys.exit(0)

    # Take a screenshot
    # UNCOMMENT IF NEEDED - Skimage dependency
    # elif symbol == key.RETURN:
    #     print('saving screenshot')
    #     img = env.render('rgb_array')
    #     save_img('screenshot.png', img)


# Register a keyboard handler
key_handler = key.KeyStateHandler()
env.unwrapped.window.push_handlers(key_handler)

duckiebot = DuckiebotControl(args.frame_skip)

barcode   = BarcodeClassifier()
barcode.show_markers = True
barcode.show_lines   = True

mapbuilder= MapBuilder()
mapbuilder.tile_size = 0.585
mapbuilder.allowable_error = 0.3
mapbuilder.hand_length = 1.8


# turtle.left(90)
# screen = turtle.Screen()
# screen.title('Map')
# turtle.pensize(1)
#
# start_angle = turtle.heading()
# from_wierd2turtle = 1.1784411471630984
# loop_count = 0

# def ka(action):
#     global from_wierd2turtle, bot_pos, bot_heading
#     action *= args.frame_skip
#     turtle.left(action[1]/from_wierd2turtle )
#     turtle.forward(action[0])
#     bot_pos = turtle.pos()
#     bot_heading = turtle.heading()

def tile_calculate(bot_pos):
    pass


GUI.frame_skip = args.frame_skip
turtle.penup()

def update(dt):
    """
    This function is called at every frame to handle
    movement/stepping and redrawing
    """
    wheel_distance = 0.102
    min_rad = 0.08

    action = np.array([0.0, 0.0])

    if key_handler[key.UP]:
        action += np.array([0.44, 0.0])
    if key_handler[key.DOWN]:
        action -= np.array([0.44, 0])
    if key_handler[key.LEFT]:
        action += np.array([0, 1])
    if key_handler[key.RIGHT]:
        action -= np.array([0, 1])
    if key_handler[key.SPACE]:
        action = np.array([0, 0])

    v1 = action[0]
    v2 = action[1]
    # Limit radius of curvature
    if v1 == 0 or abs(v2 / v1) > (min_rad + wheel_distance / 2.0) / (min_rad - wheel_distance / 2.0):
        # adjust velocities evenly such that condition is fulfilled
        delta_v = (v2 - v1) / 2 - wheel_distance / (4 * min_rad) * (v1 + v2)
        v1 += delta_v
        v2 -= delta_v

    action[0] = v1
    action[1] = v2

    # Speed boost
    if key_handler[key.LSHIFT]:
        action *= 1.5




    lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)
    distance_to_road_center = lane_pose.dist
    angle_from_straight_in_rads = lane_pose.angle_rad

    #print(f"{distance_to_road_center=}, {angle_from_straight_in_rads=}")

    global duckiebot, barcode, mapbuilder
    duckiebot.update(angle_from_straight_in_rads, distance_to_road_center)
    action = duckiebot.get_action()

    obs, reward, done, info = env.step(action)

    GUI.draw_path_debug(env.cur_angle, env.cur_pos)

    # print("step_count = %s, reward=%.3f" % (env.unwrapped.step_count, reward))

    barcode.update(cv2.cvtColor(obs, cv2.COLOR_RGB2BGR))
    barcode.show_observation()
    if mapbuilder.state.value == 2:
        GUI.draw_graph()
    cv2.waitKey(1)

    if barcode.red_line_detected:
        abs_paths = barcode.get_paths(env.cur_angle/np.pi*180)
        if len(abs_paths) > 0:
            y, _, x = env.cur_pos
            path = mapbuilder.update(abs_paths, np.array([x, y], dtype=np.float64), Direction.direction_from_radians(-env.cur_angle + np.pi))
            print(path)
            if mapbuilder.state.value == 2:
                GUI.update_graph(mapbuilder.road_map)
            # print(str(mapbuilder))
            # print(mapbuilder.get_nodes_locations())
            duckiebot.way(path)


    if key_handler[key.RETURN]:
        im = Image.fromarray(obs)

        im.save("screen.png")

    if done:
        print("done!")
        env.reset()
        env.render()

    env.render()


pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

# Enter main event loop
pyglet.app.run()

env.close()
