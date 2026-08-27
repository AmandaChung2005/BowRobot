import sys
import time
import winsound
from pynput.keyboard import Key, Listener

import config
from . import robot_interface_new as robot

halt = False
selection = None
using_input = False

def pressed(key):
    global halt, selection

    if key == Key.delete:
        halt = True
        print ("\nProgram Terminated")
        return False

    if using_input:
        return

    if key == Key.enter:
        selection = "enter"

listener = Listener(on_press = pressed)

def start_interface():
    listener.start()

def check_halt():
    if halt:
        robot.stop()
        listener.stop()
        sys.exit()

def wait_for_enter(message):
    global selection

    selection = None

    print()
    print(message)
    print("Press Enter to continue")
    print("Press Delete to stop")

    while selection != "enter":
        check_halt()
        time.sleep(0.01)

    selection = None

def safe_input(prompt):
    global using_input

    using_input = True

    try:
        answer = input(prompt)
    finally: 
        using_input = False

    check_halt()

    return answer

def select_string():
    while True:
        string = safe_input(
            "\nString:\n"
            " [G] G String\n"
            " [D] D String\n"
            " [A] A String\n"
            " [E] E String\n"
            " [X] Back\n"
            "> "
        ).strip().upper()

        if string in ("G", "D", "A", "E"):
            return string

        if string in ("X", "BACK"):
            return None
        
        print ("Invalid string, try again")

def select_start_position():
    while True:
        position = safe_input(
            "\nStarting Position:\n"
            " [F] Frog\n"
            " [M] Middle\n"
            " [T] Tip\n"
            " [X] Back\n"
            "> "
        ).strip().lower()

        if position in ("f", "frog"):
            return "frog"

        if position in ("m", "middle"):
            return "middle"

        if position in ("t", "tip"):
            return "tip"

        if position in ("x", "back"):
            return None

        print("Invalid position, try again")

def select_start_direction():
    while True:
        direction = safe_input(
            "\nSelect Start Direction:\n"
            " [U] Upbow\n"
            " [D] Downbow\n"
            " [X] Back\n"
            "> "
        ).strip().lower()

        if direction in ("u", "upbow"):
            return "upbow"

        if direction in ("d", "downbow"):
            return "downbow"

        if direction in ("x", "back"):
            return None

        print("Invalid Direction. Please Enter 'Upbow' or 'Downbow'")

def select_bowing_type():
    while True:
        bow = safe_input(
            "\nBowing Type:\n"
            " [R] Rosin\n"
            " [B] Basic\n"
            " [S] Spiccato\n"
            " [X] Back\n"
            "> "
        ).strip().lower()

        if bow in ("b", "basic"):
            return "basic"

        if bow in ("r", "rosin"):
            return "rosin"

        if bow in ("s", "spiccato"):
            return "spiccato"

        if bow in ("x", "back"):
            return None
        
        print("Invalid input, try again")

def yes_no(prompt):
    while True:
        answer = safe_input(prompt).strip().lower()

        if answer in ("y", "yes"):
            return True

        if answer in ("n", "no"):
            return False

        print("invalid input, try again")

def beep():
    if config.beep:
        winsound.Beep(500, 200)