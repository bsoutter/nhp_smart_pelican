# Written by Ben Soutter

from pymodbus.server.asynchronous import StartTcpServer, StopServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder , Endian
from threading import Thread
from twisted.internet.task import LoopingCall
import os

initial_volume = 35 # Initial volume register
ocr_default_file = "/home/pi/Desktop/MainProcess/ocr_default_values.txt"
neo_default_file = "/home/pi/Desktop/MainProcess/neo_default_values.txt"
modbus_map_size = 100 # How many registers in the Modbus map (starting at add 0)

NEO_DEFAULTS = {
    'function':     0,
    'frequency':    10,
    'brightness':   100,
    'red':          0,
    'green':        255,
    'blue' :        0
}

# sudo pip3 install pymodbus twisted service_identity adafruit-circuitpython-neopixel

class ModbusHandler(Thread):
    def __init__(self, neo_handler, serial_handler, logger=None):
        self._logger = logger
        self._map_data = [0] * modbus_map_size
        self._neo_handler = neo_handler
        self._serial_handler = serial_handler
        self._esp_board_error = False
        self._neo_initialised = False

        self._logger.info("Modbus Thread Started")
        Thread.__init__(self)

    def loop_call(self, context):
        # Check that we're connected
        try:
            for i in range(3):
                self._serial_handler.get_values(i+1)

            if self._esp_board_error == True:
                # Board reconnected - reset all modbus registers to last good value, ensure boards are re-written over this scan
                context[0][0].setValues(3,0,self._map_data)
                self._map_data[0:13] = [pow(2,15)*13]
            self._esp_board_error = False
        except:
            self._esp_board_error = True

        if self._esp_board_error:
            context[0][0].setValues(3,0,[pow(2,15)]*13)

        else:
            # Check for data change
            map_data = context[0][0].getValues(3, 0, count=modbus_map_size)

            # Check current map against last read
            if map_data != self._map_data:
                print("different!")
                # Find the register that was updated
                system_v_amp_updated = False
                system_i_amp_updated = False
                system_i_shift_updated = False
                for reg, (new, old) in enumerate(zip(map_data, self._map_data)):
                    # Check each register for a difference
                    if new != old:
                        new = self.decode_16bit_int(new)
                        old = self.decode_16bit_int(old)
                        # System Change
                        if reg >= 0 and reg <= 2:
                            if reg == 0:
                                system_v_amp_updated = True
                                if new < 0 or new > 255:
                                    context[0][0].setValues(3, 0, [self.encode_16bit_int(old)])
                                else:
                                    for i in range(3):
                                        try:
                                            self._serial_handler.set_values(board_no=i+1, v_amp=new)
                                        except Exception as e:
                                            print(1, e)
                                        context[0][0].setValues(3, 3+i, [self.encode_16bit_int(new)])
                            elif reg == 1:
                                system_i_amp_updated = True
                                if new < -255 or new > 255:
                                    context[0][0].setValues(3, 1, [self.encode_16bit_int(old)])
                                else:
                                    for i in range(3):
                                        try:
                                            self._serial_handler.set_values(board_no=i+1, i_amp=new)
                                        except Exception as e:
                                            print(2,e)
                                        context[0][0].setValues(3, 6+i, [self.encode_16bit_int(new)])
                            else:
                                system_i_shift_updated = True
                                if new < -90 or new > 90:
                                    context[0][0].setValues(3, 2, [self.encode_16bit_int(old)])
                                else:
                                    for i in range(3):
                                        try:
                                            self._serial_handler.set_values(board_no=i+1, i_shift=new)
                                        except Exception as e:
                                            print(3, e)
                                        context[0][0].setValues(3, 9+i, [self.encode_16bit_int(new)])

                        # Individual Line Voltage Updated
                        if reg >= 3 and reg <= 5 and not system_v_amp_updated:
                            if new < 0 or new > 255:
                                context[0][0].setValues(3, reg, [self.encode_16bit_int(old)])
                            else:
                                context[0][0].setValues(3, 0, [300])
                                try:
                                    self._serial_handler.set_values(board_no=reg-2, v_amp=new)
                                except Exception as e:
                                    print(4,e)

                        # Individual Line Current Updated
                        if reg >= 6 and reg <= 8 and not system_i_amp_updated:
                            if new < -255 or new > 255:
                                context[0][0].setValues(3, reg, [self.encode_16bit_int(old)])
                            else:
                                context[0][0].setValues(3, 1, [300])
                                try:
                                    self._serial_handler.set_values(board_no=reg-5, i_amp=new)
                                except Exception as e:
                                    print(5,e)

                        # Individual Line Current Shift Updated
                        if reg >= 9 and reg <= 11 and not system_i_shift_updated:
                            if new < -90 or new > 90:
                                context[0][0].setValues(3, reg, [self.encode_16bit_int(old)])
                            else:
                                context[0][0].setValues(3, 2, [300])
                                try:
                                    self._serial_handler.set_values(board_no=reg-8, i_shift=new)
                                except Exception as e:
                                    print(6,e)
                        
                        # Defaults Requested
                        if reg == 12 and new == 1:
                            self.ocr_write_defaults()
                            context[0][0].setValues(3, 12, [0])

                        # NEO Leds
                        # Function
                        if reg == 50:
                            if new < 0 or new >= self._neo_handler.neo_state_no:
                                context[0][0].setValues(3, 50, [self.encode_16bit_int(old)])
                            else:
                                self._neo_handler.set_function(new)
                        # Frequency
                        if reg == 51:
                            if new < 0 or new > 255:
                                context[0][0].setValues(3, 51, [self.encode_16bit_int(old)])
                            else:
                                self._neo_handler.update_frequency(float(new) / 10.0)
                        # Brightness
                        if reg == 52:
                            if new < 0 or new > 100:
                                context[0][0].setValues(3, 52, [self.encode_16bit_int(old)])
                            else:
                                self._neo_handler.brightness = new
                        # Red Register
                        if reg == 53:
                            if new < 0 or new > 255:
                                context[0][0].setValues(3, 53, [self.encode_16bit_int(old)])
                            else:
                                self._neo_handler.set_colour({'red':new})
                        # Green Register
                        if reg == 54:
                            if new < 0 or new > 255:
                                context[0][0].setValues(3, 54, [self.encode_16bit_int(old)])
                            else:
                                self._neo_handler.set_colour({'green':new})
                        # Blue Register
                        if reg == 55:
                            if new < 0 or new > 255:
                                context[0][0].setValues(3, 55, [self.encode_16bit_int(old)])
                            else:
                                self._neo_handler.set_colour({'blue':new})
                        # Default
                        if reg == 56 and new != 0:
                            self.neo_write_defaults(context[0])
                            context[0][0].setValues(3, 56, [0])
                            

                self._map_data = context[0][0].getValues(3, 0, count=modbus_map_size)

    def stop_server(self):
        # If process is stopped, stop outputting on ESPs
        for i in range(3):
            self._logger.info("Stopping Board No {}".format(i+1))
            self._serial_handler.set_values(board_no=i+1, v_amp=0,i_amp=0)
        StopServer()

    def encode_16bit_int(self,val):
        pl = BinaryPayloadBuilder(byteorder=Endian.Big)
        pl.add_16bit_int(val)
        return pl.to_registers()[0]

    def decode_16bit_int(self,val):
        pl = BinaryPayloadDecoder(bytearray([val >> 8 & 0xff, val & 0xff]), byteorder=Endian.Big)
        return pl.decode_16bit_int()

    # funtion to write the current default sine wave values to the default on next power up
    def ocr_write_defaults(self):
        defaults = []
        system_defaults = dict()
        for i in range(3):
            try:
                try:
                    defaults.append(self._serial_handler.get_values(i+1))
                except Exception as e:
                    print(7,e)
            except Exception as e:
                print(str(e))
        
        if defaults[0]['v_amp'] == defaults[1]['v_amp'] == defaults[2]['v_amp']:
            system_defaults['sys_v_amp'] = defaults[0]['v_amp']
        else:
            system_defaults['sys_v_amp'] = 300

        if defaults[0]['i_amp'] == defaults[1]['i_amp'] == defaults[2]['i_amp']:
            system_defaults['sys_i_amp'] = defaults[0]['i_amp']
        else:
            system_defaults['sys_i_amp'] = 300

        if defaults[0]['i_shift'] == defaults[1]['i_shift'] == defaults[2]['i_shift']:
            system_defaults['sys_i_shift'] = defaults[0]['i_shift']
        else:
            system_defaults['sys_i_shift'] = 300
        
        defaults.append(system_defaults)

        with open(ocr_default_file, 'w') as f:
            f.write(str(defaults))
            return defaults

    def ocr_read_defaults(self):
        if os.path.isfile(ocr_default_file):
            # File exists
            with open(ocr_default_file, 'r') as f:
                data = eval(f.read())
                for i in range(0,3):
                    try:
                        self._serial_handler.set_values(i+1, v_amp=data[i]['v_amp'], i_amp=data[i]['i_amp'], i_shift=data[i]['i_shift'])
                    except Exception as e:
                        print(8,e)
                return data
        else:
            return self.ocr_write_defaults()

    def neo_write_defaults(self, context=None):
        defaults = {}
        if self._neo_initialised:
            if context is not None:
                defaults = {
                    'function':     context[0].getValues(3,50,count=1)[0],
                    'frequency':    context[0].getValues(3,51,count=1)[0],
                    'brightness':   context[0].getValues(3,52,count=1)[0],
                    'red':          context[0].getValues(3,53,count=1)[0],
                    'green':        context[0].getValues(3,54,count=1)[0],
                    'blue' :        context[0].getValues(3,55,count=1)[0]
                }
        else:
            defaults = NEO_DEFAULTS
        with open(neo_default_file, 'w') as f:
            f.write(str(defaults))
            return defaults

    def neo_read_defaults(self, context):
        if os.path.isfile(neo_default_file):
            # File exists
            with open(neo_default_file, 'r') as f:
                data = eval(f.read())
                return data
        else:
            return self.neo_write_defaults(context)

    def run(self):
        store = ModbusSlaveContext(
            hr=ModbusSequentialDataBlock(0, [0]*(modbus_map_size + 1)),
        )
        context = ModbusServerContext(slaves=store, single=True)

        # Read Default Values and write to ESPs, prefil Modbus Map
        ocr_defaults = self.ocr_read_defaults()
        # System values
        context[0].setValues(3,0,[ocr_defaults[3]['sys_v_amp'],ocr_defaults[3]['sys_i_amp'],ocr_defaults[3]['sys_i_shift']])
        # Voltages
        context[0].setValues(3,3,[ocr_defaults[i]['v_amp'] for i in range(3)])
        # Current
        context[0].setValues(3,6,[ocr_defaults[i]['i_amp'] for i in range(3)])
        # Shift
        context[0].setValues(3,9,[ocr_defaults[i]['i_shift'] for i in range(3)])

        # Read NEO Pixel Defaults and fill modbus map
        neo_defaults = self.neo_read_defaults(context)
        context[0].setValues(3,50,[neo_defaults['function']])
        context[0].setValues(3,51,[neo_defaults['frequency']])
        context[0].setValues(3,52,[neo_defaults['brightness']])
        context[0].setValues(3,53,[neo_defaults['red']])
        context[0].setValues(3,54,[neo_defaults['green']])
        context[0].setValues(3,55,[neo_defaults['blue']])

        # Pass neo values to neo thread
        self._neo_handler.set_colour(neo_defaults)
        self._neo_handler.set_function(neo_defaults['function'])
        self._neo_handler.brightness = neo_defaults['brightness']
        self._neo_initialised = True

        # Synchronise maps on startup
        self._map_data = context[0].getValues(3, 0, count=modbus_map_size)

        

        # # Initial Values for Neo
        # context[0].setValues(3,11,[10]) # Frequency Register
        # context[0].setValues(3,12,[100]) # Brightness Register

        # # Initial Colour to Red
        # context[0].setValues(3,13,[255])
        # context[0].setValues(3,14,[0])
        # context[0].setValues(3,15,[0])

        identity = ModbusDeviceIdentification()
        identity.VendorName = 'NHP'
        # identity.ProductCode = 'PM'
        # identity.VendorUrl = 'http://github.com/bashwork/pymodbus/'
        identity.ProductName = 'OCR Current Transducer'
        # identity.ModelName = 'Pymodbus Server'
        identity.MajorMinorRevision = '1.0'

        # # TCP Server
        loop = LoopingCall(f=self.loop_call, context=(context,))
        loop.start(0.5, now=False) # initially delay by time

        print("Server Running!")
        StartTcpServer(context, identity=identity, address=("", 502))
        print("Exiting Modbus Server")