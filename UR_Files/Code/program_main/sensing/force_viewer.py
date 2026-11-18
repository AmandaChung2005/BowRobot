"""Fresh, live plot of TCP-frame forces and TCP linear speed."""

import csv
import json
import queue
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class ForceViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("Force and TCP Speed - Current Run")
        self.root.geometry("1000x700")
        self.root.minsize(720, 500)

        self.messages = queue.Queue()
        self.times = []
        self.data = {
            "Fx": [],
            "Fy": [],
            "Fz": [],
            "TCP speed": [],
        }
        self.finished = False
        self.closed = False

        self.visible = {
            name: tk.BooleanVar(value=True)
            for name in self.data
        }

        self._build_ui()

        threading.Thread(
            target=self._read_input,
            daemon=True
        ).start()

        self.root.protocol("WM_DELETE_WINDOW", self._discard)
        self.root.after(50, self._poll_messages)

    def _build_ui(self):
        toggle_frame = ttk.LabelFrame(
            self.root,
            text="Visible lines",
            padding=8
        )
        toggle_frame.pack(fill=tk.X, padx=8, pady=(8, 4))

        for name in self.data:
            ttk.Checkbutton(
                toggle_frame,
                text=name,
                variable=self.visible[name],
                command=self._redraw
            ).pack(side=tk.LEFT, padx=(0, 16))

        self.figure = Figure(figsize=(9, 6), dpi=100)
        self.axes = self.figure.add_subplot(111)

        self.axes.set_title(
            "Measured force and TCP linear speed"
        )
        self.axes.set_xlabel("Time (s)")
        self.axes.set_ylabel("Force (N) / speed (m/s)")
        self.axes.grid(True, alpha=0.3)

        colors = {
            "Fx": "tab:blue",
            "Fy": "tab:orange",
            "Fz": "tab:green",
            "TCP speed": "tab:red",
        }

        self.lines = {
            name: self.axes.plot(
                [],
                [],
                label=name,
                color=colors[name],
                linestyle="--" if name == "TCP speed" else "-"
            )[0]
            for name in self.data
        }

        self.axes.legend(loc="upper right")

        self.canvas = FigureCanvasTkAgg(
            self.figure,
            master=self.root
        )
        self.canvas.get_tk_widget().pack(
            fill=tk.BOTH,
            expand=True,
            padx=8,
            pady=4
        )

        self.readout = ttk.Label(
            self.root,
            text=(
                "Fx: -- N    Fy: -- N    Fz: -- N    "
                "TCP speed: -- m/s"
            ),
            font=("Segoe UI", 12, "bold")
        )
        self.readout.pack(anchor=tk.W, padx=10, pady=4)

        self.status = ttk.Label(
            self.root,
            text="Recording this run..."
        )
        self.status.pack(anchor=tk.W, padx=10, pady=(0, 4))

        buttons = ttk.Frame(self.root, padding=8)
        buttons.pack(fill=tk.X)

        self.save_button = ttk.Button(
            buttons,
            text="Save Graph and CSV",
            command=self._save,
            state=tk.DISABLED
        )
        self.save_button.pack(side=tk.RIGHT, padx=(8, 0))

        self.discard_button = ttk.Button(
            buttons,
            text="Don't Save",
            command=self._discard,
            state=tk.DISABLED
        )
        self.discard_button.pack(side=tk.RIGHT)

    def _read_input(self):
        try:
            for line in sys.stdin:
                try:
                    message = json.loads(line)
                except json.JSONDecodeError:
                    continue

                self.messages.put(message)

                if message.get("type") == "finish":
                    return
        finally:
            self.messages.put({"type": "finished"})

    def _poll_messages(self):
        if self.closed:
            return

        got_samples = False

        while True:
            try:
                message = self.messages.get_nowait()
            except queue.Empty:
                break

            if message.get("type") == "sample":
                try:
                    values = {
                        "time": float(message["time"]),
                        "Fx": float(message["fx"]),
                        "Fy": float(message["fy"]),
                        "Fz": float(message["fz"]),
                        "TCP speed": float(message["tcp_speed_m_s"]),
                    }
                except (KeyError, TypeError, ValueError):
                    continue

                if not np.all(np.isfinite(list(values.values()))):
                    continue

                self.times.append(values["time"])

                for name in self.data:
                    self.data[name].append(values[name])

                got_samples = True

            elif message.get("type") == "finished":
                self._finish()

        if got_samples:
            self._redraw()

        self.root.after(50, self._poll_messages)

    @staticmethod
    def _limits(values):
        values = np.asarray(values, dtype=float)

        if values.size == 0:
            return -1.0, 1.0

        low = float(np.min(values))
        high = float(np.max(values))

        if high == low:
            padding = max(0.1, abs(low) * 0.1)
        else:
            padding = max(0.1, (high - low) * 0.08)

        return low - padding, high + padding

    def _redraw(self):
        if not self.times:
            return

        times = np.asarray(self.times, dtype=float)

        for name in self.data:
            self.lines[name].set_data(times, self.data[name])
            self.lines[name].set_visible(self.visible[name].get())

        # Show every sample from this run; no rolling window or trimming.
        x_start = float(times[0])
        x_end = float(times[-1])

        if x_end <= x_start:
            x_end = x_start + 1.0

        self.axes.set_xlim(x_start, x_end)

        visible_values = []

        for name in self.data:
            if self.visible[name].get():
                visible_values.extend(self.data[name])

        if visible_values:
            self.axes.set_ylim(self._limits(visible_values))

        self.readout.configure(
            text=(
                f"Latest | "
                f"Fx: {self.data['Fx'][-1]:+.3f} N    "
                f"Fy: {self.data['Fy'][-1]:+.3f} N    "
                f"Fz: {self.data['Fz'][-1]:+.3f} N    "
                f"TCP speed: {self.data['TCP speed'][-1]:.4f} m/s"
            )
        )

        self.canvas.draw_idle()

    def _finish(self):
        if self.finished:
            return

        self.finished = True
        self.status.configure(
            text=f"Run complete: {len(self.times)} samples collected"
        )
        self.save_button.configure(state=tk.NORMAL)
        self.discard_button.configure(state=tk.NORMAL)
        self._redraw()

    def _save(self):
        if not self.times:
            messagebox.showinfo(
                "No data",
                "No samples were received."
            )
            self._discard()
            return

        filename = filedialog.asksaveasfilename(
            parent=self.root,
            title="Save force and speed graph",
            defaultextension=".png",
            filetypes=[("PNG image", "*.png")]
        )

        if not filename:
            return

        png_path = Path(filename)
        csv_path = png_path.with_suffix(".csv")

        try:
            self.figure.savefig(
                png_path,
                dpi=200,
                bbox_inches="tight"
            )

            with csv_path.open(
                "w",
                newline="",
                encoding="utf-8"
            ) as output_file:
                writer = csv.writer(output_file)
                writer.writerow([
                    "time_s",
                    "Fx_N",
                    "Fy_N",
                    "Fz_N",
                    "tcp_speed_m_s",
                ])

                writer.writerows(zip(
                    self.times,
                    self.data["Fx"],
                    self.data["Fy"],
                    self.data["Fz"],
                    self.data["TCP speed"]
                ))

        except OSError as exc:
            messagebox.showerror(
                "Save failed",
                f"Could not save graph or data:\n{exc}"
            )
            return

        messagebox.showinfo(
            "Saved",
            f"Graph saved to:\n{png_path}\n\n"
            f"All samples saved to:\n{csv_path}"
        )
        self._discard()

    def _discard(self):
        if self.closed:
            return

        self.closed = True
        self.root.destroy()


def main():
    root = tk.Tk()
    ForceViewer(root)
    root.mainloop()


if __name__ == "__main__":
    main()