#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from PIDController import PIDController
from threading import Thread

import time
import math
import pygame
import random
import os

try:
    os.chdir(os.path.dirname(__file__))
except:
    pass

################################################################################
# Setup
################################################################################

# Init Pygame
pygame.init()

# Init Display Parameters
infoObject = pygame.display.Info()
(display_width, display_height) = display_size = (infoObject.current_w, infoObject.current_h)
#(1600, 900)

# Open Display, set it to borderless windowed mode
gameDisplay = pygame.display.set_mode(display_size, pygame.NOFRAME)
pygame.display.set_caption('MOMO!')

# Init PID controllers
x_pid = PIDController(0.025, 0.005, 0.0)
y_pid = PIDController(0.025, 0.005, 0.0)

# Init colours
black = (0,0,0)
white = (255,255,255)

# Init clock
clock = pygame.time.Clock()
cycle_flag = True
crashed = False

# Init last command times
last_x_command_time = 0
last_y_command_time = 0
last_emotion_command_time = 0

# Load basic sprites
momo_imgs = [pygame.transform.scale(pygame.image.load('momo_1.png'), (display_width // 2, display_height // 2)),
             pygame.transform.scale(pygame.image.load('momo_2.png'), (display_width // 2, display_height // 2)),
             pygame.transform.scale(pygame.image.load('momo_3.png'), (display_width // 2, display_height // 2))]


momo_emotions = {'neutral': pygame.transform.scale(pygame.image.load('momo_neutral.png'), (display_width // 2, display_height // 2)),
                 'test': pygame.transform.scale(pygame.image.load('momo_XD.png'), (display_width // 2, display_height // 2))}

emotion_state = 'neutral'

# Create storage vars and backup list
momo_img = momo_emotions[emotion_state]

# The backups are for dealing with scale distortions, NOT for threading issues!
momo_img_bak = momo_emotions[emotion_state]
momo_imgs_bak = momo_imgs

# Init size vars
img_size_x, img_size_y = momo_emotions[emotion_state].get_rect().size

x_req = img_size_x / 2
pid_x = img_size_x / 2
x = img_size_x / 2

y_req = img_size_y / 2
pid_y = img_size_y / 2
y = img_size_y / 2

################################################################################
# Functions
################################################################################

def blink_cycler():
    global emoton_state
    global momo_img
    global momo_img_bak
    global cycle_flag

    # MOMO blink animation
    while True:
        if momo_blink:
            momo_img_bak = momo_emotions[emotion_state]
            time.sleep(5)
            time.sleep(random.uniform(0, 2))

            momo_img_bak = momo_imgs[0]
            cycle_flag = False
            while not cycle_flag:
                pass
            time.sleep(10/30)

            momo_img_bak = momo_imgs[1]
            cycle_flag = False
            while not cycle_flag:
                pass
            time.sleep(10/30)

            momo_img_bak = momo_imgs[2]
            cycle_flag = False
            while not cycle_flag:
                pass
            time.sleep(10/30)

            if random.randint(0, 1) == 1:
                momo_img_bak = momo_emotions[emotion_state]
                time.sleep(1)
                time.sleep(random.uniform(0, 3))

                momo_img_bak = momo_imgs[0]
                cycle_flag = False
                while not cycle_flag:
                    pass
                time.sleep(10/30)

                momo_img_bak = momo_imgs[1]
                cycle_flag = False
                while not cycle_flag:
                    pass
                time.sleep(10/30)

                momo_img_bak = momo_imgs[2]
                cycle_flag = False
                while not cycle_flag:
                    pass
                time.sleep(10/30)
        else:
            pass

def momo_img_translate(x,y):
    gameDisplay.blit(momo_img, (x,y))

# Init eye position
momo_img_translate(x, y)

momo_blink = True
Thread(target=blink_cycler, args=()).start()

################################################################################
# ROS Functions
################################################################################

def cmd_callback(msg):
    global x_req
    global last_x_command_time

    if abs(msg.angular.z) > 0.05:
        x_req += msg.angular.z * 75
        last_x_command_time = pygame.time.get_ticks()

def laser_callback(msg):
    global y_req
    global last_y_command_time

    min_range = min(msg.ranges)

    if min_range < 2.5:
        y_req = 1 * img_size_y * (1 - (min_range / 2.5))
        last_y_command_time = pygame.time.get_ticks()

def emotion_callback(msg):
    global emotion_state
    global last_emotion_command_time
    global momo_img_bak

    emotion_state = msg.data
    if momo_emotions.get(emotion_state):
        momo_img_bak = momo_emotions[emotion_state]
        last_emotion_command_time = pygame.time.get_ticks()
    else:
        emotion_state = "neutral"

################################################################################
# Main Loop
################################################################################

rospy.init_node('listener', anonymous = True)

# Callback function gets run each time a message is received
rospy.Subscriber("cmd_vel", Twist, cmd_callback)
rospy.Subscriber("scan_filtered", LaserScan, laser_callback)
rospy.Subscriber("momo_emotion", String, emotion_callback)

while not crashed:

    # If pygame crashes, quit
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            crashed = True

    # Grab key events
    keys = pygame.key.get_pressed()

    if keys[pygame.K_LEFT]:
        x_req -= 50

    elif keys[pygame.K_RIGHT]:
        x_req += 50

    elif keys[pygame.K_DOWN]:
        y_req += 50

    elif keys[pygame.K_UP]:
        y_req -= 50

    elif keys[pygame.K_k]:
        emotion_state = "test"
        momo_img_bak = momo_emotions[emotion_state]

        last_emotion_command_time = pygame.time.get_ticks()

    elif keys[pygame.K_ESCAPE] or keys[pygame.K_q]:
        if gameDisplay.get_flags() & pygame.NOFRAME:
            pygame.display.set_mode(display_size)
        else:
            pygame.display.set_mode(display_size, pygame.NOFRAME)

    else:
        if pygame.time.get_ticks() - last_x_command_time > 1000:
            x_req = img_size_x / 2

        if pygame.time.get_ticks() - last_y_command_time > 1000:
            y_req = img_size_y / 2

    if pygame.time.get_ticks() - last_emotion_command_time > 2000:
        if emotion_state != "neutral":
            emotion_state = "neutral"
            momo_img_bak = momo_emotions[emotion_state]

    # Limit the requests
    if x_req < 0:
        x_req = 0.05 * img_size_x
    if x_req > img_size_x:
        x_req = 0.95 * img_size_x
    if y_req > 1.1 * img_size_y:
        y_req = 1.1 * img_size_y
    if y_req < 0:
        y_req = 0

    # Compute PID control for eyes
    try:
        pid_x += x_pid.compute(x_req, x)
        pid_y += y_pid.compute(y_req, y)

        if pid_x != None:
            x = pid_x

        if pid_y != None:
            y = pid_y
    except:
        pass

    # Squash eyes if they're near the top of the screen
    if y > 0.9 * img_size_y:
        squashed_x = display_width // 2 * 1.1 - 1.5 * (img_size_y - y)
        squashed_y = display_height // 2 * 0.9 + (img_size_y - y)

        shf_x = 0.9 * x + 1.5 * (img_size_y - y) // 2
        shf_y = y

        momo_img = momo_img_bak
        momo_img = pygame.transform.scale(momo_img, (int(squashed_x), int(squashed_y))) #display_height // 2))

    # Squash eyes if they're near the bottom of the screen
    elif y < 0.1 * img_size_y:
        squashed_x = display_width // 2 * 1.1 - 1.5 * y
        squashed_y = display_height // 2 * 0.9 + y

        shf_x = 0.9 * x + 1.5 * y // 2
        shf_y = y

        momo_img = momo_img_bak
        momo_img = pygame.transform.scale(momo_img, (int(squashed_x), int(squashed_y))) #display_height // 2))

    else:
        momo_img = momo_img_bak
        shf_x = x
        shf_y = y

    # Fill the screen with white
    gameDisplay.fill(white)

    # Execute the eye translation
    momo_img_translate(shf_x, shf_y)

    # Update the frame and tick the clock (Best effort for 60FPS)
    pygame.display.update()
    clock.tick(60)

    cycle_flag = True

pygame.quit()
quit()
