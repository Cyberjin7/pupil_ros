#!/usr/bin/env python
import sys

import numpy.linalg
import rospy
from pupil_msgs.msg import surface

import numpy as np
import pygame


class AccuracyRecorder:
    def __init__(self, screen, cross_length, wait=1, record=2, frequency=60):
        self.target_screen = screen
        self.cross_length = cross_length
        self.cross_color = (0, 0, 0)
        self.wait_color = (255, 0, 0)
        self.record_color = (0, 255, 0)
        self.circle_color = (0, 0, 255, 50)
        self.erase = pygame.Surface(size=screen.get_size())  # surface used to erase objects
        self.gaze_surface = pygame.Surface(size=screen.get_size())
        self.gaze_surface.set_alpha(80)
        self.gaze_surface.fill((255, 255, 255))
        self.erase.fill((255, 255, 255))  # white color
        # List that stores horizontal and vertical Rects that together make a target cross.
        # Stores cross for current time step and (n-1)th time step.
        # List used for efficient screen updates
        self.current_cross = [pygame.Rect(0, 0, 0, 0), pygame.Rect(0, 0, 0, 0),  # Previous cross placement (placeholder)
                              pygame.Rect(0, 0, 0, 0), pygame.Rect(0, 0, 0, 0)]  # Current cross placement (placeholder)
        # List that stores gaze positions for current and (n-1)th time step
        # Used for efficient screen updates
        self.gaze = [pygame.Rect(0, 0, 0, 0),  # Previous gaze
                     pygame.Rect(0, 0, 0, 0)]  # Current gaze
        self.watch_switch = False

        self.accuracy_test = False
        self.test_in_progress = False
        self.accuracy_list = []
        self.target_list = []
        self.timer = 0
        self.wait_time = wait
        self.record_time = record
        self.record_switch = False
        self.frequency = frequency
        self.target_iterator = 0
        self.gaze_list = np.zeros(shape=(self.record_time*self.frequency + 1, 2), dtype=np.int64)
        self.average_accuracy = 0.

    def draw_target(self, target_center, record_color):
        self.current_cross[:2] = self.current_cross[2:]  # make current target previous target
        # erase previous target cross
        self.target_screen.blit(self.erase, self.current_cross[0], self.current_cross[0])
        self.target_screen.blit(self.erase, self.current_cross[1], self.current_cross[1])
        # Draw vertical line of target
        vertical_cross = pygame.draw.line(self.target_screen,
                                          record_color,
                                          [target_center[0], target_center[1] - self.cross_length / 2],
                                          [target_center[0], target_center[1] + self.cross_length / 2],
                                          5)
        # draw horizontal line of target
        horizontal_cross = pygame.draw.line(self.target_screen,
                                            record_color,
                                            [target_center[0] - self.cross_length / 2, target_center[1]],
                                            [target_center[0] + self.cross_length / 2, target_center[1]],
                                            5)
        # make new target current target
        self.current_cross[2] = vertical_cross
        self.current_cross[3] = horizontal_cross
        pygame.display.update(self.current_cross)  # Selectively update screen

    def surface_callback(self, data):
        if len(data.gazes) != 0:
            # Get gaze position from data
            gaze_position = [data.gazes[0].norm_pos.x, data.gazes[0].norm_pos.y]
            watching = data.gazes[0].on_surf

            # If gaze is within the defined surface: draw gaze on screen. Else erase gaze.
            if watching:
                # If gaze enters surface after leaving, no need to erase previous gaze in first loop
                if not self.watch_switch:
                    self.watch_switch = True
                    # Draw new gaze
                    self.gaze[1] = pygame.draw.circle(self.gaze_surface,
                                                      self.circle_color,
                                                      [gaze_position[0] * resolution[0],
                                                       (1 - gaze_position[1]) * resolution[1]],
                                                      50,
                                                      0)
                    self.target_screen.blit(self.gaze_surface, self.gaze[1], self.gaze[1])
                    pygame.display.update(self.gaze[1])  # Selectively update screen
                else:
                    # Set current gaze to previous gaze and erase
                    self.gaze[0] = self.gaze[1]
                    self.target_screen.blit(self.erase, self.gaze[0], self.gaze[0])
                    self.gaze_surface.blit(self.erase, self.gaze[0], self.gaze[0])

                    # Check if erased gaze overlaps with target cross. If it does, redraw target
                    if self.gaze[0].colliderect(self.current_cross[2]):
                        pygame.draw.rect(self.target_screen, self.cross_color, self.current_cross[2], 0)
                    if self.gaze[0].colliderect(self.current_cross[3]):
                        pygame.draw.rect(self.target_screen, self.cross_color, self.current_cross[3], 0)

                    # Draw new gaze
                    self.gaze[1] = pygame.draw.circle(self.gaze_surface,
                                                      self.circle_color,
                                                      [gaze_position[0] * resolution[0],
                                                       (1 - gaze_position[1]) * resolution[1]],
                                                      50,
                                                      0)
                    self.target_screen.blit(self.gaze_surface, self.gaze[1], self.gaze[1])
                    pygame.display.update(self.gaze + self.current_cross[2:])  # Selectively update screen
            else:
                # watch_switch used to erase gaze at first loop the gaze leaves surface.
                # Prevents continuously erasing the same spot if gaze leaves surface for prolonged period of time
                if self.watch_switch:
                    # Erase gaze
                    self.target_screen.blit(self.erase, self.gaze[1], self.gaze[1])
                    self.gaze_surface.blit(self.erase, self.gaze[1], self.gaze[1])
                    # If gaze overlaps target, redraw target
                    if self.gaze[1].colliderect(self.current_cross[2]):
                        pygame.draw.rect(self.target_screen, self.cross_color, self.current_cross[2], 0)
                    if self.gaze[1].colliderect(self.current_cross[3]):
                        pygame.draw.rect(self.target_screen, self.cross_color, self.current_cross[3], 0)

                    pygame.display.update(self.gaze[1:] + self.current_cross[2:])  # Selectively update screen
                    self.watch_switch = False

    def record_accuracy(self):
        # Wait period: Give experimenter time to adjust gaze
        if not self.record_switch:
            # If accuracy test just started, draw the first target.
            if not self.test_in_progress:
                self.cross_color = self.wait_color
                self.draw_target(self.target_list[self.target_iterator], self.cross_color)
                self.test_in_progress = True
                self.timer = 0
                self.accuracy_list.clear()
            # During wait period: do nothing
            if self.timer <= self.frequency*self.wait_time:
                self.timer = self.timer + 1
            else:
                # Once wait period is over, switch to record mode for current target.
                self.timer = 0
                self.cross_color = self.record_color
                self.draw_target(self.target_list[self.target_iterator], self.cross_color)
                self.record_switch = True
        # Record period: Experimenter should be looking at the target
        elif self.record_switch:
            # During record period: record gaze data
            if self.timer <= self.frequency*self.record_time:
                self.gaze_list[self.timer, :] = self.gaze[1].center
                self.timer = self.timer + 1
            else:
                # Once done recording: calculate accuracy. Reset timer
                distance = self.gaze_list - np.asarray(self.target_list[self.target_iterator])
                self.accuracy_list.append(np.average(numpy.linalg.norm(distance, ord=2, axis=1)))
                self.timer = 0
                # If there are targets left: draw the next target and switch to wait period
                if self.target_iterator < len(self.target_list)-1:
                    self.target_iterator = self.target_iterator + 1
                    self.cross_color = self.wait_color
                    self.draw_target(self.target_list[self.target_iterator], self.cross_color)
                    self.record_switch = False
                # If no targets left: reset all experiment parameters to init
                else:
                    self.average_accuracy = np.average(np.array(self.accuracy_list))
                    print("The average pixel error is: ", self.average_accuracy)
                    self.accuracy_test = False
                    self.test_in_progress = False
                    self.record_switch = False
                    self.target_iterator = 0
                    self.timer = 0
                    self.target_screen.blit(self.erase, self.current_cross[2], self.current_cross[2])
                    self.target_screen.blit(self.erase, self.current_cross[3], self.current_cross[3])
                    pygame.display.update(self.current_cross[2:])
                    self.current_cross = [pygame.Rect(0, 0, 0, 0), pygame.Rect(0, 0, 0, 0),
                                          pygame.Rect(0, 0, 0, 0), pygame.Rect(0, 0, 0, 0)]


if __name__ == '__main__':
    pygame.init()

    if not pygame.font:
        print("Pygame Font module not correctly imported!")

    screen = pygame.display.set_mode(size=[0, 0], flags=pygame.FULLSCREEN)
    resolution = pygame.display.get_window_size()
    white = 255, 255, 255
    screen.fill(white)

    font = pygame.font.Font(None, 60)
    button_text = font.render('Start', True, (0, 0, 0))
    button_pos = button_text.get_rect(center=(screen.get_width()/2, screen.get_height()/2))
    pygame.draw.rect(screen, (200, 200, 200), button_pos, 0)
    screen.blit(button_text, button_pos)

    pygame.display.flip()

    recorder = AccuracyRecorder(screen,
                                100,
                                wait=rospy.get_param('gaze_tracker/wait_period'),
                                record=rospy.get_param('gaze_tracker/record_period'),
                                frequency=rospy.get_param('gaze_tracker/ros_frequency'))

    targets = rospy.get_param("gaze_tracker/targets")

    for target in targets:
        recorder.target_list.append((resolution[0]*target[0], resolution[1]*target[1]))

    print(recorder.target_list)

    # recorder.target_list = [(resolution[0]*0.25, resolution[1]*0.25),
    #                         (resolution[0]*0.75, resolution[1]*0.25),
    #                         (resolution[0]*0.75, resolution[1]*0.75),
    #                         (resolution[0]*0.25, resolution[1]*0.75),
    #                         (resolution[0]*0.5, resolution[1]*0.5)]

    try:
        rospy.init_node('gaze_tracker', anonymous=True)
        rate = rospy.Rate(recorder.frequency)

        surface_sub = rospy.Subscriber('pupil_surface', surface, recorder.surface_callback)

        while not rospy.is_shutdown():

            # Change button color based on mouse position
            if not recorder.accuracy_test:
                if button_pos.collidepoint(pygame.mouse.get_pos()):
                    pygame.draw.rect(screen, (150, 150, 150), button_pos, 0)
                else:
                    pygame.draw.rect(screen, (200, 200, 200), button_pos, 0)
                screen.blit(button_text, button_pos)
                pygame.display.update(button_pos)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    rospy.signal_shutdown('Quitting')
                    sys.exit()

                if event.type == pygame.KEYDOWN:
                    pygame.quit()
                    rospy.signal_shutdown('Quitting')

                if event.type == pygame.MOUSEBUTTONUP:
                    if button_pos.collidepoint(pygame.mouse.get_pos()) and not recorder.accuracy_test:
                        screen.blit(recorder.erase, button_pos, button_pos)
                        pygame.display.update(button_pos)
                        recorder.accuracy_test = True

            if recorder.accuracy_test:
                recorder.record_accuracy()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
