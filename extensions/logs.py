import logging
import datetime


if not logging.getLogger().hasHandlers():
    ct = datetime.datetime.now().strftime('%H:%M:%S')
    logging.basicConfig(filename=f"logs/{datetime.date.today()}@{ct}", filemode='a', format=f'%(asctime)s [%(levelname)s]%(message)s', datefmt='%H:%M', level=logging.DEBUG)
    logging.info(f"\n{'-'*20}\nStarted logging at {ct}\n{'-'*20}\n")

