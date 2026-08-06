import numpy as np
import matplotlib.pyplot as plt
import os

import config
import plotting as plot

while True:
    filename = input(
        "Enter File Name (or Type 'list' to View Saved Filed): ")

    if filename.lower() == "list":
        files = sorted(
            f[:-4] for f in os.listdir(config.save_folder)
            if f.endswith(config.file_extension)
        )

        if len(files) == 0:
            print("No Saved Files Found")

        else:
            print("\nSaved Files:")
            for file in files:
                print(f" {file}")
            print()

        continue

    filepath = os.path.joint(config.save_folder, filename = config.file_extension)

    if os.path.exists(filepath):
        break

    print("File Not Found. Type 'list' to See Available Files")
    
    trial = np.load("data/{filename}.npz", allow_pickle = True)

    time = trial["time"]
    data = trial["data"]
    sample_rate = trial["sample_rate"]
    channel_names = trial["channel_names"]

    print(f"Sample Rate: {sample_rate} Hz")
    print(f"Channels: {list(channel_names)}")
    print(f"Data Shape: {data.shape}")

    plotter = plot.Plotter()
    plotter.plot(time, data)

    plt.show()