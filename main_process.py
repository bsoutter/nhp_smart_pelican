# Written by Ben Soutter

from rpi_serial_handler import UARTHandler
from rpi_neo_handler import NeoHandler
from rpi_modbus_handler import ModbusHandler

if __name__ == "__main__":
    neo = NeoHandler()
    ser = UARTHandler()
    mb = ModbusHandler(neo_handler=neo, serial_handler=ser)
    
    neo.start()
    mb.start()
    try:
        mb.join()
        neo.join()
    except:
        neo.stop_thread()
        mb.stop_server()