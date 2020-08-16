# Written by Ben Soutter

from threading import Thread
from neopixel import NeoPixel
import board, time, math

# pip3 install adafruit-circuitpython-neopixel

# number of data points for pulse output, more = smoother pulse slower max speed, less = better speed but more jerky
pulse_data_points = 40

# Main Neo Pixel Thread
class NeoHandler(Thread):
    def __init__(self, number_of_leds=300):
        self.stop = False
        self.leds = number_of_leds
        self.pixels = NeoPixel(board.D18, 300, auto_write=False) # Hard coded 300 for timing issues TODO (should re-look at this)

        # Define Neo States
        self.neo_state_off = 0
        self.neo_state_solid = 1
        self.neo_state_flashing = 2
        self.neo_state_alternate = 3
        self.neo_state_pulse = 4
        self.neo_state_chase = 5
        self.neo_state_bounce = 6
        self.neo_state_no = 7

        # Initial Colour
        self.colour = (29,60,125)
        self.period_delay = 0.5
        self.brightness = 0.5

        # Current State
        self.neo_state = self.neo_state_off

        # Chaser Variables
        self.chaser_leds_on = 3
        self.chaser_leds_off = 7
        self.chaser_reverse = False

        Thread.__init__(self)

    # Function to fill only selected LEDS (needed because of hard coded led number)
    def fill(self, col):
        for i in range(self.leds):
            self.pixels[i] = col

    # Function to stop thread
    def stop_thread(self):
        self.stop = True

    # Update colour with any function (obsolete)
    def _update_colour(self,col):
        self.colour = col

    # For writing preset functions, i.e. red pulsing
    def set_function(self, state, col = -1, freq = -1):
        if col != -1:
            self.colour = col
        if freq != -1:
            self.period_delay = 1.0 / float(freq) / 2.0
        self.neo_state = state

    # Update Frequency 
    def update_frequency(self, freq):
        self.period_delay = 1.0 / float(freq) / 2.0

    # Thread to exit gracefully
    def exit(self):
        self.fill((0,0,0))
        self.pixels.show()
        print("Exiting Neo Thread")

    # Thread to update colour from Modbus
    def set_colour(self,colour_dict):
        col = list(self.colour)
        if 'red' in colour_dict:
            col[0] = colour_dict['red']
        if 'green' in colour_dict:
            col[1] = colour_dict['green']
        if 'blue' in colour_dict:
            col[2] = colour_dict['blue']
        self.colour = tuple(col)

    # Main thread
    def run(self):
        period = False

        # Initialise Chaser Values
        reg_on = 0
        reg_leds = 0
        leds_on_off = self.chaser_leds_on + self.chaser_leds_off
        led_mask = pow(2, self.leds)-1
        for j in range(self.chaser_leds_on):
            reg_on |= 1 << j

        # Initial Pulse Values
        pulse_x = range(pulse_data_points)
        pulse_vals = []
        for i in pulse_x:
            pulse_vals.append(pow(1.15,i))
        pulse_vals = [i/max(pulse_vals) for i in pulse_vals]
        tmp = pulse_vals.copy()
        tmp.sort(reverse=True)
        pulse_vals = pulse_vals + tmp
        
        # Bounce variables
        bounce_led = 0
        bounce_dir = False

        # main loop
        while not self.stop:
            # Update colour/delay time at start of each cycle
            colour = self.colour
            period_delay = self.period_delay
            
            # Reset brightness if state isn't pulsing
            if self.neo_state != self.neo_state_pulse:
                self.pixels.brightness = self.brightness

            # Leds off
            if self.neo_state == self.neo_state_off:
                self.fill((0,0,0))
                self.pixels.show()

            # Leds on
            elif self.neo_state == self.neo_state_solid:
                self.fill(colour)
                self.pixels.show()

            # Flash On/Off
            elif self.neo_state == self.neo_state_flashing:
                if not period:
                    self.fill(colour)
                    self.pixels.show()
                else:
                    self.fill((0,0,0))
                    self.pixels.show()
                time.sleep(period_delay)

            # Pulsing on/off
            elif self.neo_state == self.neo_state_pulse:
                self.fill(colour)
                for i in pulse_vals:
                    self.pixels.brightness = i
                    self.pixels.show()

            # Bounce 1 led swings from end to end
            elif self.neo_state == self.neo_state_bounce:
                if not bounce_dir:
                    self.fill((0,0,0))
                    self.pixels[bounce_led] = colour
                    if bounce_led >= self.leds - 1: 
                        bounce_dir = True
                    else: 
                        bounce_led += 1
                else:
                    self.fill((0,0,0))
                    self.pixels[bounce_led] = colour
                    if bounce_led <= 0: 
                        bounce_dir = False
                    else: 
                        bounce_led -= 1
                self.pixels.show()

            # Adjustable chaser (every few leds on or off and shifts along the strip)
            elif self.neo_state == self.neo_state_chase:
                # cycle through each section of LEDs in strip
                for i in range(leds_on_off):
                    for j in range(int(math.ceil(float(self.leds)/float(leds_on_off)))):
                        reg_leds |= (reg_on << (j*leds_on_off)) & led_mask
                    
                    # Reverse binary string if Leds to chase backwards
                    if self.chaser_reverse:
                        reg_leds = int(format(reg_leds, "0{}b".format(self.leds))[::-1], 2)
                    
                    # Shift left and mask with No of LEDs
                    reg_on = reg_on << 1
                    reg_on &= led_mask

                    # Wrap around
                    if i >= leds_on_off - self.chaser_leds_on: reg_on |= 1

                    # Write to the LEDs
                    for i in range(self.leds):
                        if (1 << i) & reg_leds:
                            self.pixels[i] = colour
                        else:
                            self.pixels[i] = (0,0,0)
                    self.pixels.show()
                    time.sleep(period_delay)

                    # Reset test reg
                    reg_leds = 0

            # Every second led alternates from on to off
            elif self.neo_state == self.neo_state_alternate:
                if not period:
                    for i in range(self.leds):
                        if i % 2:
                            self.pixels[i] = colour
                        else:
                            self.pixels[i] = (0,0,0)
                    self.pixels.show()
                else:
                    for i in range(self.leds):
                        if i % 2:
                            self.pixels[i] = (0,0,0)
                        else:
                            self.pixels[i] = colour
                    self.pixels.show()
                time.sleep(period_delay)

            # Invalid amount of states
            else:   
                self.neo_state = self.neo_state_off
            period = not period
        self.exit()