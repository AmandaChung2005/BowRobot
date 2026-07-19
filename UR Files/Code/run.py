import config

if config.setup:
    import calibration
else:
    import main