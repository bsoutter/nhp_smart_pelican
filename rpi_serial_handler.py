import serial, time
# Serial data structure
# 10 Byte Packet:
#     B[0] Start Of Packet (0x7E)
#     B[1] Address of ESP to read/write to
#     B[2] nRead/Write Bit
#     B[3] V_Amplitude (0 to 255)
#     B[4,5] Signed I_Amplitude
#     B[6,7] Signed I_Phase_Shift
#     B[8] Reserved
#     B[9] End of packet (0xff)

class UARTHandler():
    def __init__(self, port='/dev/serial0', baudrate=115200):
        self._PACKET_START_BYTE = 0x7e
        self._PACKET_END_BYTE = 0xff
        self._PACKET_READ_ACK = 0x77
        self._PACKET_LENGTH = 10
        self._PACKET_ADDRESS_BYTE = 1
        self._PACKET_READ_BYTE = 2
        self._RPI_ADDRESS = 0x00
        self._UART_TIMEOUT = 100 # 100ms


        self._packet_read = [0] * self._PACKET_LENGTH
        self._packet_write = [0] * self._PACKET_LENGTH

        # Configure Serial Port, timeout should be handled already
        self._port = serial.Serial(port, baudrate=baudrate, timeout=1)
        self._port.flushInput()
        self._port.flushOutput()
    
    def get_values(self, board_no):
        self._port.flushInput()
        v_amp = 0
        i_amp = 0
        i_shift = 0
        if board_no != 1 and board_no != 2 and board_no != 3:
            raise ValueError('Error: get_values() Invalid Board Number (1,2 or 3)')
            # return 0 # Invalid Board Number

        # Create Read Packet
        self._packet_read = [self._PACKET_START_BYTE, board_no, 0] + [0] * 6 + [self._PACKET_END_BYTE]
        self._port.write(self._packet_read)

        timeout = 0
        while (self._port.inWaiting() != self._PACKET_LENGTH):
            time.sleep(0.001)
            timeout += 1
            if timeout > self._UART_TIMEOUT:
                raise ValueError('Error: get_values() Message timed out, check RPi connections to the ESP proto board')
                # return -1 # timeout
        
        # Read data from the serial port
        return_data = list(bytearray(self._port.read(self._port.inWaiting())))

        # Process packet - check start and stop bytes
        if(return_data[0] != self._PACKET_START_BYTE or return_data[self._PACKET_LENGTH-1] != self._PACKET_END_BYTE):
            raise ValueError('Error: get_values() Bad data received (incorrect SOF or EOF)')
        else:
            # Packet is good, check if it's for rpi
            if return_data[1] != self._RPI_ADDRESS:
                raise ValueError('Error: get_values() Return Address not ours, something went wrong on the ESP proto board')
            else:
                # Check that returned data is read type
                if return_data[2] != 0:
                    raise ValueError('Error: get_values() Received a write packet, expected a read')
                else:
                    # Read the data
                    v_amp = return_data[3]

                    i_amp = (return_data[4]<<8 & 0xff00) | (return_data[5] & 0x00ff)
                    if i_amp & 0x8000: i_amp -= 65536

                    i_shift = (return_data[6]<<8 & 0xff00) | (return_data[7] & 0x00ff)
                    if i_shift & 0x8000: i_shift -= 65536
                    
        return {'v_amp': v_amp, 'i_amp': i_amp, 'i_shift': i_shift}

    def set_values(self, board_no, v_amp = None, i_amp = None, i_shift = None):
        self._port.flushInput()
        if board_no != 1 and board_no != 2 and board_no != 3:
            raise ValueError('Error: set_values() Invalid Board Number (1,2 or 3)')
        current_board_values = self.get_values(board_no)

        # Setup write packet
        self._packet_write = [self._PACKET_START_BYTE, board_no, 1] + [0] * 6 + [self._PACKET_END_BYTE]

        # Valid data check - cast all value to int
        if v_amp is not None:
            v_amp = int(v_amp)
            if v_amp < 0 or v_amp > 255:
                raise ValueError('Error: set_values() v_amp out of range (0 to 255)')
            self._packet_write[3] = v_amp
        else:
            self._packet_write[3] = current_board_values['v_amp']

        if i_amp is not None:
            i_amp = int(i_amp)
            if i_amp < -255 or i_amp > 255:
                raise ValueError('Error: set_values() v_amp out of range (-255 to 255)')
            self._packet_write[4] = i_amp >> 8 & 0xff
            self._packet_write[5] = i_amp & 0xff
        else:
            self._packet_write[4] = current_board_values['i_amp'] >> 8 & 0xff
            self._packet_write[5] = current_board_values['i_amp'] & 0xff

        if i_shift is not None:
            i_shift = int(i_shift)
            if i_shift < -90 or i_shift > 90:
                raise ValueError('Error: set_values() i_shift out of range (-90 to 90)')
            self._packet_write[6] = i_shift >> 8 & 0xff
            self._packet_write[7] = i_shift & 0xff
        else:
            self._packet_write[6] = current_board_values['i_shift'] >> 8 & 0xff
            self._packet_write[7] = current_board_values['i_shift'] & 0xff

        self._port.write(bytearray(self._packet_write))

        timeout = 0
        while (self._port.inWaiting() != self._PACKET_LENGTH):
            time.sleep(0.001)
            timeout += 1
            if timeout > self._UART_TIMEOUT:
                raise ValueError('Error: set_values() Message timed out, check RPi connections to the ESP proto board')
        
                # Read data from the serial port
        return_data = list(bytearray(self._port.read(self._port.inWaiting())))

        # Process packet - check start and stop bytes
        if(return_data[0] != self._PACKET_START_BYTE or return_data[self._PACKET_LENGTH-1] != self._PACKET_END_BYTE):
            raise ValueError('Error: set_values() Bad data received (incorrect SOF or EOF)')
        else:
            # Packet is good, check if it's for rpi
            if return_data[1] != self._RPI_ADDRESS:
                raise ValueError('Error: set_values() Return Address not ours, something went wrong on the ESP proto board')
            # Check that returned data is read type
            if return_data[2] != 1:
                raise ValueError('Error: set_values() Received a read packet, expected a write')
            # Check for Ack
            if return_data[3] != self._PACKET_READ_ACK:
                raise ValueError('Error: set_values() ESP proto board didn\'t acknowledge read')
        return 1