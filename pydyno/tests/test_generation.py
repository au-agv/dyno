import logging
import uuid

import coloredlogs

from multiprocessing import Pool

from pydyno.generate import Generator


# Get the logger
logger = logging.getLogger("pydyno")
coloredlogs.install(level=logger.level, logger=logger)

# Generate a unique ID for this dataset generation run
uid = uuid.uuid4().hex[0:4].upper()

def main():

    logger.info("The unique ID for this dataset is '%s'", uid)

    generator = Generator(uid)

    executables = ["dyno_olav_dlc"]
    templates = ["dlc139"]

    with Pool(processes=2) as pool:
        pool.starmap(generator.run, zip(executables, templates))


if __name__ == '__main__':
    main()
