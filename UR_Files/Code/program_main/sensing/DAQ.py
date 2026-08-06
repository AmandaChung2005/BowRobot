import nidaqmx
from nidaqmx.constants import AcquisitionType

with nidaqmx.Task() as task:

    for channel in ["ai0", "ai1", "ai2", "ai3"]:
        task.ai_channels.add_ai_voltage_chan(
            f"Dev1/{channel}"
        )

    task.timing.cfg_samp_clk_timing(
        rate=4800,
        sample_mode=AcquisitionType.FINITE,
        samps_per_chan=100
    )

    data = task.read(
        number_of_samples_per_channel=100
    )

    print("Shape:")
    print(len(data), len(data[0]))

    for i, channel in enumerate(data):
        print(f"Channel {i}:")
        print(" Mean:", sum(channel)/len(channel))
        print(" Min:", min(channel))
        print(" Max:", max(channel))
        print()