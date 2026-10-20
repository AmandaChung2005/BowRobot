import matplotlib.pyplot as plt
import sensing.sensing_config as sensing_config

class Plotter:
    def __init__(self):
        pass

    def plot(self, time, data):
        fig = plt.figure(figsize=(10, 6))

        if data.ndim == 1:
            plt.plot(time, data)

        else:
            labels = []

            if "TriAccelerometer" in sensing_config.daq_channels:
                for axis in sensing_config.daq_channels["TriAccelerometer"]:
                    labels.append(f"Accel {axis.upper()}")

            for sensor in sensing_config.daq_channels:
                if sensor != "TriAccelerometer":
                    labels.append(sensor)

            for i, channel in enumerate(data):
                if i < len(labels):
                    plt.plot(time, channel, label = labels[i])
                else:
                    plt.plot(time, channel, label = f"Channel {i}")

            plt.legend()

        plt.xlabel("Time (s)")
        plt.ylabel("Voltage (V)")
        plt.title("Sensor Data")
        plt.grid(True)
        plt.tight_layout()

        plt.show(block = False)
        plt.pause(0.1)