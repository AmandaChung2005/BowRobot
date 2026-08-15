import sys
import time
from pynput.keyboard import Key, Listener

from . import robot_interface as robot

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
            "\nString (G, D, A, E): "
        ).strip().upper()

        if string in ("G", "D", "A", "E"):
            return string
        
        print ("Invalid string, try again")

def select_start_position():
    while True:
        position = safe_input(
            "\nStarting Position (frog, middle tip): "
        ).strip().lower()

        if position in ("f", "frog"):
            return "frog"

        if position in ("m", "middle"):
            return "middle"

        if position in ("t", "tip"):
            return "tip"

        print("Invalid position, try again")

def select_start_direction():
    while True:
        direction = input("Select Start Direction (Upbow/Downbow): ").strip().lower()

        if direction in ("u", "upbow"):
            return "upbow"

        if direction in ("d", "downbow"):
            return "downbow"

        print("Invalid Direction. Please Enter 'Upbow' or 'Downbow'")

def select_bowing_type():
    while True:
        bow = safe_input(
            "\nBowing Type (Rosin, Basic, Spiccato): "
        ).strip().lower()

        if bow in ("b", "basic"):
            return "basic"

        if bow in ("r", "rosin"):
            return "rosin"

        if bow in ("s", "spiccato"):
            return "spiccato"

        print("Invalid input, try again")

def yes_no(prompt):
    while True:
        answer = safe_input(prompt).strip().lower()

        if answer in ("y", "yes"):
            return True

        if answer in ("n", "no"):
            return False

        print("invalid input, try again")