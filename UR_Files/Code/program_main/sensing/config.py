# DAQ Settings

device = "Dev1"
sample_rate = 4800
default_duration = 0.1

save_name = "test"
save_folder = "data"
file_extension = ".npz"

daq_channels = {
    "Microphone": "ai0",
    "TriAccelerometer": {
        "x": "ai1",
        "y": "ai2",
        "z": "ai3"
    }
}

sensors = {
    "TriAccelerometer": {
        "manufacturer": "PCB Piezotronics",
        "model": "356A03",
        "sensor_type": "IEPE",
        "excitation_current": 2.1e-3,
        "axes": {
            "x": {
                "units": "m/s^2",
                "sensitivity": 0.1004
            },
            "y": {
                "units": "m/s^2",
                "sensitivity": 0.1004
            },
            "z": {
                "units": "m/s^2",
                "sensitivity": 0.1004
            }
        }
    },

    "Microphone": {
        "manufacturer": "PCB Piezotronics",
        "model": "378B02",
        "sensor_type": "IEPE",
        "excitation_current": 2.1e-3,
        "units": "Pa",
        "sensitivity": 0.0500
    },

    "Piezo": {
        "manufacturer": "Generic",
        "model": "Piezo Disc",
        "sensor_type": "Voltage",
        "units": "V",
        "sensitivity": None
    }
}