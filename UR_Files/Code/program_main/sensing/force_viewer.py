import csv
import json
import queue
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from pathlib import Path
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

window_seconds = 30.0

class ForcePlotViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("Live End-Effector Force")
        self.root.geometry("950x620")
        self.root.minsize(700, 450)

        self.messages = queue. Queue()
        self.times = []
        self.fx = []
        self.fy = []
        self.fz = []
        self.finished = False
        self.closed = False

        self._build_ui()

        self.reader_thread = threading.Thread(
            target = self._read_stdin,
            daemon = True
        )

        self.read_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self._discard_and_close)
        self.root.after(50, self._poll_messages)
    def _build_ui(self):
        plot_frame = ttk.Frame(self.root, padding = 8)
        plot_frame.pack(fill = tk.BOTH, expand = True)

        self.figure = Figure(figsize = (9, 5), dpi = 100)
        self.axes = self.figure.add_subplot(111)

        self.axes.set_title("Force Measured at End Effector")
        self.axes.set_xlabel("Time (s)")
        self.axes.set_ylable("Force (N)")
        self.axes.grid(True, alpha = 0.3)

        self.line_x = self.axes.plot([], [], label = "Fx")
        self.line_y = self.axes.plot([], [], label = "Fy")
        self.line_z = self.axes.plot([], [], label = "Fz")
        self.axes.legend(loc = "upper right")

        self.canvas = FigureCanvasTkAgg(self.figure, master = plot_frame)
        self.canvas.get_tk_eidget().pack(fill = tk.BOTH, expand = True)

        controls = ttk.Frame(self.root, padding = (8, 0, 8, 8))
        controls.pack(fill = tk.X)

        self.status_label = ttk.Label(
            controls,
            text = "Recording force data..."
        )
        self.status_label.pack(side = tk.LEFT)

        self.save_button = ttk.Button(
            controls,
            text = "Save Graph",
            command = self._save,
            state = tk.DISABLED
        )

        self.save_button.pack(side = tk.RIGHT, padx = (8, 0))

        self.discard_button = ttk.Button(
            controls,
            text = "Don't Save",
            command = self._discard_and_close,
            state = tk.DISABLED
        )
        self.discard_button.pack(side = tk.RIGHT)

        self.readout = ttk.Label(
            plot_frame,
            text="Fx: -- N    Fy: -- N    Fz: -- N",
            font=("Segoe UI", 12, "bold")
        )
        self.readout.pack(anchor=tk.W, pady=(6, 0))

    def _read_stdin(self):
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
            self.messages.put({"type": "eof"})

    def _poll_messages(self):
        if self.closed:
            return

        got_samples = False

        while True:
            try:
                message = self.messages.get_nowait()
            except queue.Empty:
                break

            message_type = message.get("type")

            if message_type == "sample":
                try:
                    sample_time = float(message["time"])
                    fx = float(message["fx"])
                    fy = float(message["fy"])
                    fz = float(message["fz"])
                except (KeyError, TypeError, ValueError):
                    continue

                if not np.all(np.isfinite([sample_time, fx, fy, fz])):
                    continue

                self.times.append(sample_time)
                self.fx.append(fx)
                self.fy.append(fy)
                self.fz.append(fz)
                got_samples = True

            elif message_type in ("finish", "eof"):
                self._mark_finished()

        if got_samples:
            self._redraw()

        self.root.after(50, self._poll_messages)

    def _redraw(self):
        if not self.times:
            return

        times = np.asarray(self.times)
        fx = np.asarray(self.fx)
        fy = np.asarray(self.fy)
        fz = np.asarray(self.fz)

        latest_time = times[-1]
        visible_start = max(0.0, latest_time - window_seconds)
        visible = times >= visible_start

        visible_times = times[visible]
        visible_fx = fx[visible]
        visible_fy = fy[visible]
        visible_fz = fz[visible]

        self.line_x.set_data(visible_times, visible_fx)
        self.line_y.set_data(visible_times, visible_fy)
        self.line_z.set_data(visible_times, visible_fz)

        right = max(window_seconds, latest_time)
        left = max(0.0, right - window_seconds)
        self.axes.set_xlim(left, right)

        visible_forces = np.concatenate(
            [visible_fx, visible_fy, visible_fz]
        )

        if visible_forces.size:
            minimum = float(np.min(visible_forces))
            maximum = float(np.max(visible_forces))
            padding = max(0.25, 0.1 * (maximum - minimum))

            if maximum - minimum < 1e-9:
                minimum -= 0.5
                maximum += 0.5
            else:
                minimum -= padding
                maximum += padding

            self.axes.set_ylim(minimum, maximum)

        self.canvas.draw_idle()

        self.readout.configure(
            text=(
                f"Latest TCP-frame force  |  "
                f"Fx: {self.fx[-1]:+.3f} N    "
                f"Fy: {self.fy[-1]:+.3f} N    "
                f"Fz: {self.fz[-1]:+.3f} N"
            )
        )

    def _mark_finished(self):
        if self.finished:
            return

        self.finished = True
        self.status_label.configure(
            text=f"Recording complete: {len(self.times)} samples"
        )
        self.save_button.configure(state=tk.NORMAL)
        self.discard_button.configure(state=tk.NORMAL)

    def _save(self):
        if not self.times:
            messagebox.showinfo(
                "No data",
                "No force samples were recorded."
            )
            self._discard_and_close()
            return

        filename = filedialog.asksaveasfilename(
            parent=self.root,
            title="Save force graph",
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
            ) as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(
                    ["time_s", "Fx_N", "Fy_N", "Fz_N"]
                )
                writer.writerows(
                    zip(self.times, self.fx, self.fy, self.fz)
                )

        except OSError as exc:
            messagebox.showerror(
                "Save failed",
                f"Could not save the graph or data:\n{exc}"
            )
            return

        messagebox.showinfo(
            "Saved",
            f"Graph saved to:\n{png_path}\n\n"
            f"Force data saved to:\n{csv_path}"
        )
        self._discard_and_close()

    def _discard_and_close(self):
        if self.closed:
            return

        self.closed = True
        self.root.destroy()


def main():
    root = tk.Tk()
    ForcePlotViewer(root)
    root.mainloop()


if __name__ == "__main__":
    main()