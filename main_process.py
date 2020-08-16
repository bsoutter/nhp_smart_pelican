#!/usr/bin/python3
# Written by Ben Soutter

from rpi_serial_handler import UARTHandler
from rpi_neo_handler import NeoHandler
from rpi_modbus_handler import ModbusHandler

import logging, signal
from systemd.journal import JournaldLogHandler

# sudo pip3 install pymodbus twisted service_identity adafruit-circuitpython-neopixel systemd

# get an instance of the logger object this module will use
logger = logging.getLogger(__name__)

# instantiate the JournaldLogHandler to hook into systemd
journald_handler = JournaldLogHandler()

# set a formatter to include the level name
journald_handler.setFormatter(logging.Formatter(
    '[%(levelname)s] %(message)s'
))

# add the journald handler to the current logger
logger.addHandler(journald_handler)

# optionally set the logging level
logger.setLevel(logging.DEBUG)

if __name__ == "__main__":
    neo = NeoHandler()
    ser = UARTHandler()
    mb = ModbusHandler(neo_handler=neo, serial_handler=ser, logger=logger)

    def exit_gracefully():
        neo.stop_thread()
        mb.stop_server()

    signal.signal(signal.SIGINT, exit_gracefully)
    signal.signal(signal.SIGTERM, exit_gracefully)

    neo.start()
    mb.start()

    try:
        logger.info("Waiting for threads to finish")
        mb.join()
        neo.join()
    except:
        logger.info("Stopping Threads")
        exit_gracefully()
