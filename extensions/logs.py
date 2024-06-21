import logging
import json
params = None


def load_json():  # expects the params file to be in root project directory
    global params
    if params is None:
        try:
            with open("./params.json", "r") as f:
                params = json.load(f)
        except FileNotFoundError:
            with open("../params.json", "r") as f:
                params = json.load(f)
    return params


class StreamToLogger:
    """
    Custom stream object that redirects writes to a logger instance.
    """

    def __init__(self, logger, log_level=logging.ERROR):
        self.logger = logger
        self.log_level = log_level
        self.buffer = ''

    def write(self, buf):
        self.buffer += buf
        if '\n' in self.buffer:
            self.flush()

    def flush(self):
        if self.buffer:
            self.logger.log(self.log_level, self.buffer.strip())
            self.buffer = ''


# 10: Debug | 20: INFO | 30: WARNING | 40: ERROR | 50: CRITICAL
if not logging.getLogger().hasHandlers():
    # Suppress numba warnings.
    from numba.core.errors import NumbaDeprecationWarning, NumbaPendingDeprecationWarning
    import warnings

    nb_log = logging.getLogger('numba')
    nb_log.setLevel(logging.ERROR)
    warnings.simplefilter('ignore', category=NumbaDeprecationWarning)
    warnings.simplefilter('ignore', category=NumbaPendingDeprecationWarning)

    import datetime
    import os
    import sys
    if not os.path.exists('logs'):
        os.makedirs('logs')
    ct = datetime.datetime.now().strftime('%H:%M:%S')
    log_file = f"logs/{datetime.date.today()}@{ct}.log"
    log_level = int(load_json().get("level", logging.INFO))

    # Create handlers
    file_handler = logging.FileHandler(log_file)
    console_handler = logging.StreamHandler()

    # Set level and format for handlers
    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s', datefmt='%H:%M')
    file_handler.setLevel(log_level)
    file_handler.setFormatter(formatter)
    console_handler.setLevel(log_level)
    console_handler.setFormatter(formatter)

    # Get root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)

    # Add handlers to the root logger
    root_logger.addHandler(file_handler)
    root_logger.addHandler(console_handler)

    sys.stderr = StreamToLogger(logging.getLogger(), logging.ERROR)

    logging.info(f"\n{'-' * 20}\nStarted logging at {ct}\n{'-' * 20}\n")

    with open("/proc/mounts", "r") as f:
        mounts = filter(lambda x: x[0].startswith("/dev/"), [line.split()[:2] for line in f.readlines()])
    for mount_point in mounts:
        if os.path.exists(os.path.join(mount_point[1], "killswitch.ks")):
            logging.info("[Killswitch] Killswitch activated.")
            exit(64)


