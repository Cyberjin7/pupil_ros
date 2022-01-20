#!/usr/bin/env python
import sys

import rospy
from pupil_msgs.msg import surface

import numpy as np
import pygame


def surface_callback(data):
    if len(data.gazes) != 0:
        gaze_position = [data.gazes[0].norm_pos.x, data.gazes[0].norm_pos.y]
        watching = data.gazes[0].on_surf
        screen.fill(white)
        if watching:
            # print(gaze_position[0], gaze_position[1])
            pygame.draw.circle(screen, circle_color, [gaze_position[0]*resolution[0], (1 - gaze_position[1])*resolution[1]], 50, 0)
            pygame.display.flip()
        else:
            pygame.display.flip()

def draw_target(target_screen, target_centers):


if __name__ == '__main__':
    pygame.init()
    screen = pygame.display.set_mode(size=[0, 0], flags=pygame.FULLSCREEN)
    resolution = pygame.display.get_window_size()
    white = 255, 255, 255
    screen.fill(white)
    circle_color = 0, 255, 0
    # pygame.draw.circle(screen, circle_color, [1000, 1000], 150.0, 0)
    pygame.display.flip()
    # print(resolution[0], resolution[1])


    try:
        rospy.init_node('gaze_tracker', anonymous=True)
        rate = rospy.Rate(60)

        surface_sub = rospy.Subscriber('pupil_surface', surface, surface_callback)

        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.display.quit()
                    pygame.quit()
                    rospy.signal_shutdown('Quitting')
                    sys.exit()
                if event.type == pygame.KEYDOWN:
                    pygame.quit()
                    rospy.signal_shutdown('Quitting')
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
