import logging
import datetime


if not logging.getLogger().hasHandlers():
    logging.basicConfig(filename="log", filemode='a', format=f'%(asctime)s [%(levelname)s]%(message)s', datefmt='%H:%M', level=logging.DEBUG)
    logging.info(f"\n{'-'*20}\nStarted logging at {datetime.datetime.now()}\n{'-'*20}\n")

