import nidaqmx
import numpy as np
import matplotlib.pyplot as plt
from nidaqmx.constants import AcquisitionType

class SensorDAQ:
    def __init__(
            self,
            device="Dev1",
            channels=("ai0",),
            sample_rate=10000
    ):
        self.device = device
        self.channels = channels
        self.sample_rate = sample_rate
        self.task = None

    def connect(self):
        self.task = nidaqmx.Task()

        for channel in self.channels:
            self.task.ai_channels.add_ai_voltage_chan(
                f"{self.device}/{channel}"
            )

    def read(self, samples=10000):
        self.task.timing.cfg_samp_clk_timing(
            rate=self.sample_rate,
            sample_mode=AcquisitionType.FINITE,
            samps_per_chan=samples
        )

        data = self.task.read(
            number_of_samples_per_channel=samples
        )

        return np.asarray(data)

    def plot(self,data):
        plt.figure()

        if data.ndim ==1:
            plt.plot(data)

        else:
            for i, channel in enumerate(data):
                plt.plot(channel, label=f"Channel {i}")

            plt.legend()

        plt.xlabel("Sample")
        plt.ylabel("Voltage (V)")
        plt.title("Sensor Data")
        plt.gird(True)

        plt.show()

    def disconnect(self):
        if self.task is not None:
            self.task.close()
            self.task = None