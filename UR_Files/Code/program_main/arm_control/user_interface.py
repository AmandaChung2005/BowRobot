import sys
import time
from pynput.keyboard import Key, Listener

import config
from . import robot_interface_new as robot
from . import arm_config

import math
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import wave

try:
    import winsound
except ImportError:
    winsound = None

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
        menu = (
            "\nBowing Type:\n"
            " [R] Rosin\n"
            " [B] Basic\n"
            " [S] Spiccato\n"
        )

        if arm_config.bow_loaded and not config.simulation:
           menu += " [U] Unload Bow\n"

        menu += (
            " [X] Back\n"
            "> "
        )

        bow = safe_input(menu).strip().lower()

        if bow in ("b", "basic"):
            return "basic"

        if bow in ("r", "rosin"):
            return "rosin"

        if bow in ("s", "spiccato"):
            return "spiccato"

        if bow in ("u", "unload") and arm_config.bow_loaded:
            return "unload"

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

def play_tone(frequency, duration):
    sample_rate = 44100
    sample_count = int(sample_rate * duration / 1000)
    fade_samples = max(1, int(sample_rate * 0.005))

    with tempfile.NameTemporaryFile(
        suffix = ".wav",
        delete = False
    ) as temp_file:
        audio_path = temp_file.name

    try:
        samples = bytearray()

        for i in range(sample_count):
            envelope = min(
                1.0,
                i / fade_samples,
                (sample_count - 1 - i) / fade_samples
            )

            value = int(
                12000 * max(0.0, envelope) * math.sin(2* math.pi * frequency * i / sample_rate)
            )

            samples.extend(struct.pack("<h", value))

        with wave.open(audio_path, "wb") as audio_file:
            audio_file.setnchannels(1)
            audio_file.setsampwidth(2)
            audio_file.setframerate(sample_rate)
            audio_file.writeframes(samples)

        if sys.platform == "darwin":
            players = [
                ("afplay", [])
            ]
        elif sys.platform.startswith("linux"):
            players = [
                ("paplay", []),
                ("aplay", ["-q"]),
                ("ffplay", ["-nodisp", "--autoexit", "-loglevel", "quiet"]),
                ("play", ["-q"])
            ]
        else:
            players = []

        for player, options in players:
            executable = shutil.which(player)

            if executable is None:
                continue

            try:
                subprocess.run(
                    [executable, *options, audio_path],
                    check = True,
                    stdout = subprocess. DEVNULL,
                    stderr = subprocess.DEVNULL
                )
                return
            except (OSError, subprocess.CalledProcessError):
                continue

        print("'Beep' Functionality is Not Compatible with This Operating System")
        print("Turning 'Beep' Off")
        config.beep = False
    finally:
        if os.path.exists(audio_path):
            os.remove(audio_path)

def beep():
    if not config.beep:
        return
    
    frequency = 500
    duration = 200

    if winsound is not None:
        winsound.Beep(frequency, duration)

    else:
        play_tone(frequency, duration)