import logging
import json
params = None


def load_json():  # expects the params file to be in root project directory
    global params
    if params is None:
        try:
            with open("./params.json", "r") as f:
                params = json.load(f)
        except OSError:
            with open("../params.json", "r") as f:
                params = json.load(f)
    return params


# 10: Debug | 20: INFO | 30: WARNING | 40: ERROR | 50: CRITICAL
if not logging.getLogger().hasHandlers():
    # Suppress numba warnings.
    from numba.core.errors import NumbaDeprecationWarning, NumbaPendingDeprecationWarning
    import warnings

    warnings.simplefilter('ignore', category=NumbaDeprecationWarning)
    warnings.simplefilter('ignore', category=NumbaPendingDeprecationWarning)

    import datetime
    import os
    if not os.path.exists('logs'):
        os.makedirs('logs')
    ct = datetime.datetime.now().strftime('%H:%M:%S')
    logging.basicConfig(filename=f"logs/{datetime.date.today()}@{ct}", filemode='a', format=f'%(asctime)s [%(levelname)s]%(message)s', datefmt='%H:%M', level=load_json().get("log_level", logging.INFO))
    logging.info(f"\n{'-'*20}\nStarted logging at {ct}\n{'-'*20}\n")

    with open("/proc/mounts", "r") as f:
        mounts = filter(lambda x: x[0].startswith("/dev/"), [line.split()[:2] for line in f.readlines()])
    for mount_point in mounts:
        if os.path.exists(os.path.join(mount_point[1], "killswitch.ks")):
            logging.info("[Killswitch] Killswitch activated.")
            exit(64)

