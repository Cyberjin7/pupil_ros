#!/usr/bin/env python
import sys

import rospy
from pupil_msgs.msg import surface

import numpy as np
import pygame


class AccuracyRecorder:
    def __init__(self, screen, cross_length):
        self.target_screen = screen
        self.cross_length = cross_length
        self.cross_color = (0, 0, 0)
        self.circle_color = 0, 255, 0
        self.erase = pygame.Surface(size=screen.get_size())  # surface used to erase objects
        self.erase.fill((255, 255, 255))
        self.current_cross = [pygame.Rect(0, 0, 0, 0), pygame.Rect(0, 0, 0, 0),  # Previous cross placement
                              pygame.Rect(0, 0, 0, 0), pygame.Rect(0, 0, 0, 0)]  # Current cross placement
        self.gaze = [pygame.Rect(0, 0, 0, 0),  # Previous gaze
                     pygame.Rect(0, 0, 0, 0)]  # Current gaze
        self.watch_switch = False

    def draw_target(self, target_center):
        self.current_cross[:2] = self.current_cross[2:]
        self.target_screen.blit(self.erase, self.current_cross[0], self.current_cross[0])
        self.target_screen.blit(self.erase, self.current_cross[1], self.current_cross[1])
        # vertical line
        vertical_cross = pygame.draw.line(self.target_screen,
                                          self.cross_color,
                                          [target_center[0], target_center[1] - self.cross_length / 2],
                                          [target_center[0], target_center[1] + self.cross_length / 2],
                                          5)
        # horizontal line
        horizontal_cross = pygame.draw.line(self.target_screen,
                                            self.cross_color,
                                            [target_center[0] - self.cross_length / 2, target_center[1]],
                                            [target_center[0] + self.cross_length / 2, target_center[1]],
                                            5)
        self.current_cross[2] = vertical_cross
        self.current_cross[3] = horizontal_cross
        pygame.display.update(self.current_cross)

    def surface_callback(self, data):
        if len(data.gazes) != 0:
            gaze_position = [data.gazes[0].norm_pos.x, data.gazes[0].norm_pos.y]
            watching = data.gazes[0].on_surf

            if watching:
                if not self.watch_switch:
                    self.watch_switch = True
                    self.gaze[1] = pygame.draw.circle(screen,
                                                      self.circle_color,
                                                      [gaze_position[0] * resolution[0],
                                                       (1 - gaze_position[1]) * resolution[1]],
                                                      50,
                                                      0)
                    pygame.display.update(self.gaze[1])
                else:
                    self.gaze[0] = self.gaze[1]
                    self.target_screen.blit(self.erase, self.gaze[0], self.gaze[0])

                    if self.gaze[0].colliderect(self.current_cross[2]):
                        pygame.draw.rect(self.target_screen, self.cross_color, self.current_cross[2], 0)
                    if self.gaze[0].colliderect(self.current_cross[3]):
                        pygame.draw.rect(self.target_screen, self.cross_color, self.current_cross[3], 0)

                    self.gaze[1] = pygame.draw.circle(screen,
                                                      self.circle_color,
                                                      [gaze_position[0] * resolution[0],
                                                       (1 - gaze_position[1]) * resolution[1]],
                                                      50,
                                                      0)
                    pygame.display.update(self.gaze + self.current_cross[2:])
            else:
                if self.watch_switch:
                    self.target_screen.blit(self.erase, self.gaze[1], self.gaze[1])
                    if self.gaze[1].colliderect(self.current_cross[2]):
                        pygame.draw.rect(self.target_screen, self.cross_color, self.current_cross[2], 0)
                    if self.gaze[1].colliderect(self.current_cross[3]):
                        pygame.draw.rect(self.target_screen, self.cross_color, self.current_cross[3], 0)
                    pygame.display.update(self.gaze[1:] + self.current_cross[2:])
                    self.watch_switch = False


if __name__ == '__main__':
    pygame.init()
    screen = pygame.display.set_mode(size=[0, 0], flags=pygame.FULLSCREEN)
    resolution = pygame.display.get_window_size()
    white = 255, 255, 255
    screen.fill(white)
    circle_color = 0, 255, 0
    pygame.display.flip()

    recorder = AccuracyRecorder(screen, 100)
    recorder.draw_target([50, 50])
    recorder.draw_target([500, 500])

    try:
        rospy.init_node('gaze_tracker', anonymous=True)
        rate = rospy.Rate(30)

        surface_sub = rospy.Subscriber('pupil_surface', surface, recorder.surface_callback)

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
                if event.type == pygame.MOUSEBUTTONDOWN:
                    recorder.draw_target(pygame.mouse.get_pos())
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
