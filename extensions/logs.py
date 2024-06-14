import logging

if not logging.getLogger().hasHandlers():
    logging.basicConfig("log", 'a', format=f'%(asctime)s [%(levelname)s]%(message)s', datefmt='%H:%M', level=logging.DEBUG)
    logging.info(f"\n{'-'*20}Started logging.")

