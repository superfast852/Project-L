import logging

if not logging.getLogger().hasHandlers():
    import datetime
    import os
    if not os.path.exists('logs'):
        os.makedirs('logs')
    ct = datetime.datetime.now().strftime('%H:%M:%S')
    logging.basicConfig(filename=f"logs/{datetime.date.today()}@{ct}", filemode='a', format=f'%(asctime)s [%(levelname)s]%(message)s', datefmt='%H:%M', level=logging.DEBUG)
    logging.info(f"\n{'-'*20}\nStarted logging at {ct}\n{'-'*20}\n")

