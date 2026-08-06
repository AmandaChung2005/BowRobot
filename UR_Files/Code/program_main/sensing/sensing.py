import nidaqmx
import numpy as np
import matplotlib.pyplot as plt
from nidaqmx.constants import AcquisitionType, Coupling, ExcitationSource
from nidaqmx.system import System
import os

import config
import plotting as plot

class SensorDAQ:
    def __init__(self):
            self.task = None

    def connect(self):
        self.task = nidaqmx.Task()

        channels = []

        accel = config.sensors["TriAccelerometer"]

        for axis, physical_channel in config.daq_channels["TriAccelerometer"].items():
            channels.append({
                "physical_channel": physical_channel,
                "name": f"Accel_{axis.upper()}",
                "sensor": accel
            })

        for sensor_name, physical_channel in config.daq_channels.items():
            if sensor_name == "TriAccelerometer":
                continue
            channels.append({
                "physical_channel": physical_channel,
                "name": sensor_name,
                "sensor": config.sensors[sensor_name]
            })

        channels.sort(
            key=lambda c: int(c["physical_channel"].replace("ai", ""))
        )

        for ch in channels:
            channel = self.task.ai_channels.add_ai_voltage_chan(
                f"{config.device}/{ch['physical_channel']}",
                name_to_assign_to_channel = ch["name"]
            )

            if ch["sensor"]["sensor_type"] == "IEPE":
                channel.ai_coupling = Coupling.AC
                channel.ai_excit_src = ExcitationSource.INTERNAL
                channel.ai_excit_val = ch["sensor"]["excitation_current"]

            else:
                channel.ai_coupling = Coupling.DC

    @staticmethod
    def test_connection():
        print("Searching for NI devices...\n")

        system = System.local()
        devices = list(system.devices)

        if len(devices) == 0:
            print("No NI devices found")
            return False

        print("Detected Devices: ")
        for device in devices:
            print(f" {device.name}")

        print()

        if config.device in [device.name for device in devices]:
            print(f"Successfully connected to {config.device}")
            return True

        print(f"{config.device} was not found")
        return False

    def read(self, seconds = config.default_duration):
        samples = int(seconds * config.sample_rate)

        self.task.timing.cfg_samp_clk_timing(
            rate = config.sample_rate,
            sample_mode = AcquisitionType.FINITE,
            samps_per_chan = samples
        )

        data = np.asarray(
            self.task.read(
                number_of_samples_per_channel=samples
            )
        )

        time = np.arange(samples)/config.sample_rate

        return time, data       

    def get_save_name(self):
        os.makedirs(config.save_folder, exist_ok = True)

        index = 0

        while True:
            if index == 0:
                suggested = config.save_name
            else:
                suggested = f"{config.save_name}_{index:03d}"

            filepath = os.path.join(config.save_folder, suggested + config.file_extension)

            if not os.path.exists(filepath):
                break

            index += 1

        while True:
            response = input(
                f"Suggested File Name: {suggested}\n"
                "Press ENTER to accept, or type a new name: "
            ).strip()

            if response == "":
                return suggested

            filepath = os.path.join(config.save_folder, response + config.file_extension)

            if os.path.exists(filepath):
                overwrite = input(
                    f"{response}.npz already exists. Overwrite?"
                ).lower()

                if overwrite in {"y", "yes"}:
                    return response

                elif overwrite in {"n", "no"}:
                    continue

                else:
                    print("Invalid Input. Try Again")

            else:
                return response

    def save(self, filename, time, data):
        filename = self.get_save_name()

        filepath = os.path.join(config.save_folder, filename + config.file_extension)

        np.savez(
            filepath,
            time = time,
            data = data,
            sample_rate = config.sample_rate,
            channel_names = [channel.name for channel in self.task.ai_channels]
        )

    def disconnect(self):
        if self.task is not None:
            self.task.close()
            self.task = None

if __name__ == "__main__":

    SensorDAQ.test_connection()

    daq = SensorDAQ()

    print("Connecting...")
    daq.connect()

    print("\nChannel Configuration:")
    for channel in daq.task.ai_channels:
        print(
            channel.name,
            "| Coupling:",
            channel.ai_coupling,
            "| Excitation:",
            channel.ai_excit_val
        )

    print("\nConfigured Channels:")
    for channel in daq.task.ai_channels:
        print(f"  {channel.name}")

    print("Reading data...")
    time, data = daq.read()

    print(f"\nData shape: {data.shape}")

    print("Plotting...")
    plotter = plot.Plotter()
    plotter.plot(time, data)

    print("Saving...")
    daq.save(time, data)

    print("Disconnecting...")
    daq.disconnect()

    print("Finshed")

    plt.show()