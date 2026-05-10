# -*- coding: utf-8 -*-
"""
Created on Fri Apr 17 21:52:19 2026

@author: Aadi
"""

import pygame

# Initialize Pygame and the Joystick module
pygame.init()
pygame.joystick.init()

# Check for connected joysticks
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joystick detected. Connect one and restart.")
    exit()

# Initialize the first joystick
controller = pygame.joystick.Joystick(0)
controller.init()

print(f"Detected: {controller.get_name()}")
print(f"Buttons: {controller.get_numbuttons()} | Axes: {controller.get_numaxes()} | Hats: {controller.get_numhats()}")

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # 1. CHECK BUTTONS (Pressed/Released)
        if event.type == pygame.JOYBUTTONDOWN:
            print(f"Button {event.button} pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print(f"Button {event.button} released.")

        # 2. CHECK AXES (Analog sticks and Triggers)
        if event.type == pygame.JOYAXISMOTION:
            # We filter small movements (drift) by checking if absolute value > 0.1
            if abs(event.value) > 0.1:
                print(f"Axis {event.axis} moved to {event.value:.2f}")

        # 3. CHECK HATS (D-Pad)
        if event.type == pygame.JOYHATMOTION:
            # Returns a tuple like (x, y) e.g., (0, 1) for Up, (-1, 0) for Left
            print(f"Hat {event.hat} moved to {event.value}")

pygame.quit()