################################ pwm_for_servos_example.py ###################################
# Peter Halverson Nov. 1, 2014
# The LabJack U3 can output two pulse-width modulated signals.  You could use it to control 
# two servo motors.
#
# PWM signal 0 comes out of the FI04 screw terminal.
# PWM signal 1 comes out of the FI05 screw terminal.  (Strange, but that's the way it is.)
#
# Documentation that allowed me to write this code is at 
# http://labjack.com/support/u6/users-guide/2.9.1.1
# and in u3.py
import u3

if __name__=="__main__":

    d = u3.U3() # Opens first found U3 over USB
    d.getFeedback(u3.BitDirWrite(5, 1)) 
    d.getFeedback(u3.BitStateWrite(5, 0))

    d.configIO(FIOAnalog = 15)
    i = 0

    while (i>-1):
        i = int(raw_input('Choose a number: '))
        d.getFeedback(u3.BitStateWrite(5, i))