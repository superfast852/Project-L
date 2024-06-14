from os import path
from extensions.logs import logging
logger = logging.getLogger(__name__)

with open("/proc/mounts", "r") as f:
    mounts = filter(lambda x: x[0].startswith("/dev/"), [line.split()[:2] for line in f.readlines()])
for mount_point in mounts:
    if path.exists(path.join(mount_point[1], "killswitch.ks")):
        logger.info("[Killswitch] Killswitch activated.")
        exit(64)