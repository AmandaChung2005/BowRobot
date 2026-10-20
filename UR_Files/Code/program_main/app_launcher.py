import json
import os
import queue
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk


PROJECT_PATH = Path(__file__).resolve().parent
RUN_SCRIPT = PROJECT_PATH / "run.py"
SETTINGS_ENV_VAR = "BOWROBOT_LAUNCH_SETTINGS"


class BowRobotLauncher:
    def __init__(self, root):
        self.root = root
        self.root.title("UR7e Violin Bowing Robot")
        self.root.geometry("760x850")
        self.root.minsize(680, 700)

        self.process = None
        self.output_queue = queue.Queue()
        self.close_when_finished = False

        self._build_variables()
        self._build_interface()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(50, self._process_output_queue)
        self.root.after(250, self._check_process)

    def _build_variables(self):
        # Existing run.py defaults. Force control is deliberately off until
        # the PID workflow has been integrated and tested in main.py.
        self.simulation_var = tk.BooleanVar(value=False)
        self.force_var = tk.BooleanVar(value=False)
        self.monitor_force_var = tk.BooleanVar(value=True)
        self.avoid_obstacles_var = tk.BooleanVar(value=False)
        self.beep_var = tk.BooleanVar(value=True)

        self.string_var = tk.StringVar(value="E")
        self.start_position_var = tk.StringVar(value="frog")
        self.start_direction_var = tk.StringVar(value="upbow")

        self.bowing_cycles_var = tk.StringVar(value="3")
        self.rosin_cycles_var = tk.StringVar(value="1")

        # Bow stroke speed is in m/s, matching arm_config.bow_speed.
        self.bow_speed_var = tk.StringVar(value="0.05")

        # Manual PID force-control settings. Units and defaults should be
        # verified against the corrected robot_interface_new.py.
        self.pid_speed_mm_s_var = tk.StringVar(value="1.0")
        self.pid_tolerance_var = tk.StringVar(value="0.1")
        self.pid_control_period_var = tk.StringVar(value="0.02")
        self.pid_max_adjustment_mm_var = tk.StringVar(value="3.0")
        self.pid_acceleration_var = tk.StringVar(value="0.05")

        self.pid_kp_var = tk.StringVar(value="0.001")
        self.pid_ki_var = tk.StringVar(value="0.00005")
        self.pid_kd_var = tk.StringVar(value="0.0001")

        self.pid_integral_limit_var = tk.StringVar(value="5.0")
        self.pid_derivative_alpha_var = tk.StringVar(value="0.2")
        self.pid_force_direction_var = tk.StringVar(value="-1.0")

        self.status_var = tk.StringVar(value="Ready")

    def _build_interface(self):
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)

        title = ttk.Label(
            outer,
            text="UR7e Violin Bowing Robot",
            font=("Segoe UI", 16, "bold")
        )
        title.pack(anchor=tk.W)

        subtitle = ttk.Label(
            outer,
            text=(
                "Configure the launch settings, then start the existing "
                "run.py workflow."
            ),
            wraplength=700
        )
        subtitle.pack(anchor=tk.W, pady=(2, 10))

        settings_frame = ttk.Frame(outer)
        settings_frame.pack(fill=tk.X)

        self._build_robot_settings(settings_frame)
        self._build_task_settings(settings_frame)
        self._build_pid_settings(settings_frame)

        controls = ttk.Frame(outer)
        controls.pack(fill=tk.X, pady=(10, 6))

        self.start_button = ttk.Button(
            controls,
            text="Start Program",
            command=self.start_program
        )
        self.start_button.pack(side=tk.LEFT)

        self.stop_button = ttk.Button(
            controls,
            text="Request Stop (Delete)",
            command=self.request_stop,
            state=tk.DISABLED
        )
        self.stop_button.pack(side=tk.LEFT, padx=(8, 0))

        ttk.Label(
            controls,
            textvariable=self.status_var
        ).pack(side=tk.RIGHT)

        console_frame = ttk.LabelFrame(
            outer,
            text="Program Console",
            padding=8
        )
        console_frame.pack(fill=tk.BOTH, expand=True, pady=(4, 0))

        self.console = tk.Text(
            console_frame,
            height=14,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.console.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        input_row = ttk.Frame(console_frame)
        input_row.pack(side=tk.BOTTOM, fill=tk.X, pady=(8, 0))

        self.console_input = ttk.Entry(input_row)
        self.console_input.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.console_input.bind("<Return>", self.send_console_input)

        self.send_button = ttk.Button(
            input_row,
            text="Send",
            command=self.send_console_input
        )
        self.send_button.pack(side=tk.LEFT, padx=(8, 0))

        note = ttk.Label(
            outer,
            text=(
                "Safety: the Stop button requests the program's Delete-key "
                "halt. Keep the robot's physical emergency stop accessible. "
                "Do not use this application as a safety-rated stop."
            ),
            wraplength=720,
            foreground="#8a2d2d"
        )
        note.pack(anchor=tk.W, pady=(8, 0))

    def _build_robot_settings(self, parent):
        frame = ttk.LabelFrame(parent, text="Robot and Runtime", padding=8)
        frame.pack(fill=tk.X, pady=(0, 8))

        checks = ttk.Frame(frame)
        checks.pack(fill=tk.X)

        ttk.Checkbutton(
            checks,
            text="RoboDK simulation",
            variable=self.simulation_var
        ).pack(side=tk.LEFT, padx=(0, 14))

        ttk.Checkbutton(
            checks,
            text="Enable force control",
            variable=self.force_var
        ).pack(side=tk.LEFT, padx=(0, 14))

        ttk.Checkbutton(
            checks,
            text="Monitor force",
            variable=self.monitor_force_var
        ).pack(side=tk.LEFT, padx=(0, 14))

        ttk.Checkbutton(
            checks,
            text="Avoid obstacles",
            variable=self.avoid_obstacles_var
        ).pack(side=tk.LEFT, padx=(0, 14))

        ttk.Checkbutton(
            checks,
            text="Beep",
            variable=self.beep_var
        ).pack(side=tk.LEFT)

        ttk.Label(
            frame,
            text=(
                "Force control is off by default. Enable it only after the "
                "PID bowing workflow is integrated and tested."
            ),
            wraplength=700,
            foreground="#8a2d2d"
        ).pack(anchor=tk.W, pady=(6, 0))

    def _build_task_settings(self, parent):
        frame = ttk.LabelFrame(parent, text="Bowing Task", padding=8)
        frame.pack(fill=tk.X, pady=(0, 8))

        row1 = ttk.Frame(frame)
        row1.pack(fill=tk.X)

        ttk.Label(row1, text="String").pack(side=tk.LEFT)
        ttk.Combobox(
            row1,
            textvariable=self.string_var,
            values=("G", "D", "A", "E"),
            state="readonly",
            width=7
        ).pack(side=tk.LEFT, padx=(6, 16))

        ttk.Label(row1, text="Starting position").pack(side=tk.LEFT)
        ttk.Combobox(
            row1,
            textvariable=self.start_position_var,
            values=("frog", "middle", "tip"),
            state="readonly",
            width=10
        ).pack(side=tk.LEFT, padx=(6, 16))

        ttk.Label(row1, text="Direction").pack(side=tk.LEFT)
        ttk.Combobox(
            row1,
            textvariable=self.start_direction_var,
            values=("upbow", "downbow"),
            state="readonly",
            width=10
        ).pack(side=tk.LEFT, padx=(6, 0))

        row2 = ttk.Frame(frame)
        row2.pack(fill=tk.X, pady=(8, 0))

        self._add_entry(
            row2,
            "Basic cycles",
            self.bowing_cycles_var,
            width=8
        )
        self._add_entry(
            row2,
            "Rosin cycles",
            self.rosin_cycles_var,
            width=8
        )
        self._add_entry(
            row2,
            "Bow speed (m/s)",
            self.bow_speed_var,
            width=10
        )

        ttk.Label(
            frame,
            text=(
                "Your current main.py may still prompt for string and "
                "starting position; these fields set run.py configuration "
                "values but do not remove existing prompts."
            ),
            wraplength=700
        ).pack(anchor=tk.W, pady=(6, 0))

    def _build_pid_settings(self, parent):
        frame = ttk.LabelFrame(
            parent,
            text="Manual PID Force Control",
            padding=8
        )
        frame.pack(fill=tk.X)

        row1 = ttk.Frame(frame)
        row1.pack(fill=tk.X)

        self._add_entry(
            row1,
            "Target force (N)",
            self.pid_target_force_var(),
            width=9
        )
        self._add_entry(
            row1,
            "Max Z speed (mm/s)",
            self.pid_speed_mm_s_var,
            width=10
        )
        self._add_entry(
            row1,
            "Tolerance (N)",
            self.pid_tolerance_var,
            width=9
        )
        self._add_entry(
            row1,
            "Period (s)",
            self.pid_control_period_var,
            width=9
        )

        row2 = ttk.Frame(frame)
        row2.pack(fill=tk.X, pady=(8, 0))

        self._add_entry(
            row2,
            "Max adjustment (mm)",
            self.pid_max_adjustment_mm_var,
            width=10
        )
        self._add_entry(
            row2,
            "Acceleration",
            self.pid_acceleration_var,
            width=9
        )
        self._add_entry(
            row2,
            "Force direction (+/-1)",
            self.pid_force_direction_var,
            width=10
        )

        row3 = ttk.Frame(frame)
        row3.pack(fill=tk.X, pady=(8, 0))

        self._add_entry(row3, "Kp", self.pid_kp_var, width=10)
        self._add_entry(row3, "Ki", self.pid_ki_var, width=10)
        self._add_entry(row3, "Kd", self.pid_kd_var, width=10)
        self._add_entry(
            row3,
            "Integral limit",
            self.pid_integral_limit_var,
            width=10
        )
        self._add_entry(
            row3,
            "Derivative alpha",
            self.pid_derivative_alpha_var,
            width=10
        )

        ttk.Label(
            frame,
            text=(
                "PID values are passed to run.py as arm_config settings. "
                "The current attached main.py/robot_interface_new.py must "
                "call the PID bowing workflow for these settings to affect "
                "bow strokes."
            ),
            wraplength=700
        ).pack(anchor=tk.W, pady=(6, 0))

    def _add_entry(self, parent, label, variable, width=10):
        block = ttk.Frame(parent)
        block.pack(side=tk.LEFT, padx=(0, 12))

        ttk.Label(block, text=label).pack(anchor=tk.W)
        ttk.Entry(
            block,
            textvariable=variable,
            width=width
        ).pack(anchor=tk.W)

    def pid_target_force_var(self):
        # Keep the UI variable stable for the lifetime of the app.
        if not hasattr(self, "_pid_target_force"):
            self._pid_target_force = tk.StringVar(value="1.0")
        return self._pid_target_force

    def _float_value(self, variable, label):
        try:
            value = float(variable.get())
        except ValueError as exc:
            raise ValueError(f"{label} must be a number") from exc

        if value != value or abs(value) == float("inf"):
            raise ValueError(f"{label} must be finite")

        return value

    def _int_value(self, variable, label):
        try:
            value = int(variable.get())
        except ValueError as exc:
            raise ValueError(f"{label} must be an integer") from exc

        if value <= 0:
            raise ValueError(f"{label} must be greater than zero")

        return value

    def _build_settings(self):
        pid = {
            "speed_mm_s": self._float_value(
                self.pid_speed_mm_s_var,
                "Max Z speed"
            ),
            "tolerance": self._float_value(
                self.pid_tolerance_var,
                "Force tolerance"
            ),
            "control_period": self._float_value(
                self.pid_control_period_var,
                "Control period"
            ),
            "max_adjustment_mm": self._float_value(
                self.pid_max_adjustment_mm_var,
                "Maximum Z adjustment"
            ),
            "acceleration": self._float_value(
                self.pid_acceleration_var,
                "Acceleration"
            ),
            "kp": self._float_value(self.pid_kp_var, "Kp"),
            "ki": self._float_value(self.pid_ki_var, "Ki"),
            "kd": self._float_value(self.pid_kd_var, "Kd"),
            "integral_limit": self._float_value(
                self.pid_integral_limit_var,
                "Integral limit"
            ),
            "derivative_filter_alpha": self._float_value(
                self.pid_derivative_alpha_var,
                "Derivative alpha"
            ),
            "force_direction": self._float_value(
                self.pid_force_direction_var,
                "Force direction"
            ),
        }

        settings = {
            "simulation": bool(self.simulation_var.get()),
            "use_force": bool(self.force_var.get()),
            "monitor_force": bool(self.monitor_force_var.get()),
            "avoid_obstacles": bool(self.avoid_obstacles_var.get()),
            "beep": bool(self.beep_var.get()),

            "current_string": self.string_var.get(),
            "start_pos": self.start_position_var.get(),
            "start_dir": self.start_direction_var.get(),

            "bowing_cycles": self._int_value(
                self.bowing_cycles_var,
                "Basic cycles"
            ),
            "rosin_cycles": self._int_value(
                self.rosin_cycles_var,
                "Rosin cycles"
            ),
            "bow_speed": self._float_value(
                self.bow_speed_var,
                "Bow speed"
            ),

            "bow_force": self._float_value(
                self.pid_target_force_var(),
                "Target force"
            ),
            "manual_force_pid": pid,
        }

        if settings["bow_speed"] <= 0:
            raise ValueError("Bow speed must be greater than zero")

        if pid["speed_mm_s"] <= 0:
            raise ValueError("Max Z speed must be greater than zero")
        if pid["control_period"] <= 0:
            raise ValueError("Control period must be greater than zero")
        if pid["max_adjustment_mm"] <= 0:
            raise ValueError("Maximum Z adjustment must be greater than zero")
        if pid["acceleration"] <= 0:
            raise ValueError("Acceleration must be greater than zero")
        if pid["tolerance"] < 0:
            raise ValueError("Force tolerance cannot be negative")
        if pid["integral_limit"] < 0:
            raise ValueError("Integral limit cannot be negative")
        if not 0.0 <= pid["derivative_filter_alpha"] <= 1.0:
            raise ValueError("Derivative alpha must be between 0 and 1")
        if pid["force_direction"] not in (-1.0, 1.0):
            raise ValueError("Force direction must be -1 or +1")

        return settings

    def _append_console(self, text):
        self.console.configure(state=tk.NORMAL)
        self.console.insert(tk.END, text)
        self.console.see(tk.END)
        self.console.configure(state=tk.DISABLED)

    def start_program(self):
        if self.process is not None and self.process.poll() is None:
            messagebox.showwarning(
                "Already running",
                "The robot program is already running."
            )
            return

        if not RUN_SCRIPT.exists():
            messagebox.showerror(
                "Missing run.py",
                f"Could not find:\n{RUN_SCRIPT}"
            )
            return

        try:
            settings = self._build_settings()
        except ValueError as exc:
            messagebox.showerror("Invalid setting", str(exc))
            return

        environment = os.environ.copy()
        environment[SETTINGS_ENV_VAR] = json.dumps(settings)

        self.console.configure(state=tk.NORMAL)
        self.console.delete("1.0", tk.END)
        self.console.configure(state=tk.DISABLED)

        command = [
            sys.executable,
            "-u",
            str(RUN_SCRIPT)
        ]

        try:
            self.process = subprocess.Popen(
                command,
                cwd=str(PROJECT_PATH),
                env=environment,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                encoding="utf-8",
                errors="replace"
            )
        except OSError as exc:
            messagebox.showerror(
                "Could not start program",
                str(exc)
            )
            self.process = None
            return

        self.start_button.configure(state=tk.DISABLED)
        self.stop_button.configure(state=tk.NORMAL)
        self.status_var.set("Running")
        self._append_console(
            "Launching run.py with the settings shown above...\n\n"
        )

        threading.Thread(
            target=self._read_child_output,
            daemon=True
        ).start()

    def _read_child_output(self):
        process = self.process

        if process is None or process.stdout is None:
            return

        try:
            # Read character-by-character so input() prompts without a
            # newline appear in the app console promptly.
            while True:
                character = process.stdout.read(1)

                if character == "":
                    break

                self.output_queue.put(character)
        except Exception as exc:
            self.output_queue.put(
                f"\n[Console reader error: {exc}]\n"
            )
        finally:
            self.output_queue.put(None)

    def _process_output_queue(self):
        finished_reading = False

        while True:
            try:
                item = self.output_queue.get_nowait()
            except queue.Empty:
                break

            if item is None:
                finished_reading = True
                continue

            self._append_console(item)

        if finished_reading:
            self._check_process()

        self.root.after(50, self._process_output_queue)

    def send_console_input(self, _event=None):
        if self.process is None or self.process.poll() is not None:
            return "break"

        text = self.console_input.get()
        self.console_input.delete(0, tk.END)

        try:
            self.process.stdin.write(text + "\n")
            self.process.stdin.flush()
            self._append_console(f"\n> {text}\n")
        except (BrokenPipeError, OSError) as exc:
            self._append_console(
                f"\n[Could not send input: {exc}]\n"
            )

        return "break"

    def request_stop(self):
        if self.process is None or self.process.poll() is not None:
            return

        # Existing user_interface.py listens for the Delete key and calls
        # robot.stop(). Send that key to the child program.
        try:
            from pynput.keyboard import Controller, Key

            keyboard = Controller()
            keyboard.press(Key.delete)
            keyboard.release(Key.delete)

            self._append_console(
                "\n[Delete-key stop requested. Waiting for the program "
                "to stop safely...]\n"
            )
            self.status_var.set("Stop requested")
        except Exception as exc:
            messagebox.showwarning(
                "Stop key unavailable",
                "Could not send Delete to the program. Press Delete on "
                "the keyboard or use the robot's physical stop controls.\n\n"
                f"Details: {exc}"
            )

    def _check_process(self):
        if self.process is not None:
            return_code = self.process.poll()

            if return_code is not None:
                self.start_button.configure(state=tk.NORMAL)
                self.stop_button.configure(state=tk.DISABLED)
                self.status_var.set(
                    f"Finished (exit code {return_code})"
                )

                self._append_console(
                    f"\n\nProgram ended with exit code {return_code}.\n"
                )

                self.process = None

                if self.close_when_finished:
                    self.root.destroy()
                    return

        self.root.after(250, self._check_process)

    def on_close(self):
        if self.process is not None and self.process.poll() is None:
            should_stop = messagebox.askyesno(
                "Program running",
                "The robot program is still running. Request a Delete-key "
                "stop before closing?"
            )

            if should_stop:
                self.close_when_finished = True
                self.request_stop()
                return

            return

        self.root.destroy()


def main():
    root = tk.Tk()
    BowRobotLauncher(root)
    root.mainloop()


if __name__ == "__main__":
    main()