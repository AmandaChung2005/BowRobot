import nidaqmx
import numpy as np
import matplotlib.pyplot as plt
from nidaqmx.constants import AcquisitionType, Coupling, ExcitationSource
from nidaqmx.system import System

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
                # channel.ai_excit_src = ExcitationSource.INTERNAL
                # channel.ai_excit_val = ch["sensor"]["excitation_current"]

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


    def save(self, filename, time, data):
  
        np.savez(
            filename,
            time = time,
            data = data,
            sample_rate = config.sample_rate,
            channel_names = [channel.name for channel in self.task.ai_channels]
        )

        print(f"Saved {filename}.npz")      

    def disconnect(self):
        if self.task is not None:
            self.task.close()
            self.task = None

if __name__ == "__main__":

    SensorDAQ.test_connection()

    daq = SensorDAQ()

    print("Connecting...")
    daq.connect()

    print("\nConfigured Channels:")

    for channel in daq.task.ai_channels:
        print(f"  {channel.name}")

    print("Reading data...")
    time, data = daq.read()

    print(f"\nData shape: {data.shape}")

    print("Plotting...")
    plot.plot(time, data)

    print("Saving...")
    daq.save("trial_001", time, data)

    print("Disconnecting...")
    daq.disconnect()

    print("Finshed")