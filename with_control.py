import copy
"""
Requirements:
    python (version > 2.7)
    re
    pyusb: $ pip install pyusb

    libusb-compat (USB backend): $ brew install libusb-compat

References:
    [1] pyusb tutorial, https://github.com/walac/pyusb/blob/master/docs/tutorial.rst
        5/13/2015
    [2] user manual, https://www.newport.com/mam/celum/celum_assets/np/resources/8742_User_Manual.pdf?0

Further Information:

Notes:
    1) Only tested with 1 8742 Model Open Loop Picomotor Controller.
    2) Current code will not support multiple controllers

TODO:
    1) Add in multi controller functionality
    2) Block illegal commands, not just commands with an invalid format
    3) Develop GUI
    4) Add in connection check.
        Execute the following commands ever .5 s
            1>1 MD?\r
            1>1 TP?\r
            1>2 TP?\r
            1>3 TP?\r
            1>4 TP?\r
            1>ERRSTR?\r
        Use http://stackoverflow.com/questions/6357850/how-do-i-run-some-python-code-in-another-process
"""
import os
import numpy as np
import serial
import matplotlib.pyplot as plt
os.chdir('C:\\Chicago RA\\CeNTREX\\auto alignment\\picomotor python code')
import usb.core
import time as time
import usb.util
import re
from labjack import ljm                       # labjack
import LabJackAnalog as LJA


NEWFOCUS_COMMAND_REGEX = re.compile("([0-9]{0,1})([a-zA-Z?]{2,})([0-9+-]*)")
MOTOR_TYPE = {
        "0":"No motor connected",
        "1":"Motor Unknown",
        "2":"'Tiny' Motor",
        "3":"'Standard' Motor"
        }

class Controller(object):
    """Picomotor Controller

    Example:

        >>> controller = Controller(idProduct=0x4000, idVendor=0x104d)
        >>> controller.command('VE?')

        >>> controller.start_console()
    """


    def __init__(self, idProduct, idVendor):
        """Initialize the Picomotor class with the spec's of the attached device

        Call self._connect to set up communication with usb device and endpoints

        Args:
            idProduct (hex): Product ID of picomotor controller
            idVendor (hex): Vendor ID of picomotor controller
        """
        self.idProduct = idProduct
        self.idVendor = idVendor
        self._connect()

    def _connect(self):
        """Connect class to USB device

        Find device from Vendor ID and Product ID
        Setup taken from [1]

        Raises:
            ValueError: if the device cannot be found by the Vendor ID and Product
                ID
            Assert False: if the input and outgoing endpoints can't be established
        """
        # find the device
        self.dev = usb.core.find(
                        idProduct=self.idProduct,
                        idVendor=self.idVendor
                        )

        if self.dev is None:
            raise ValueError('Device not found')

        # set the active configuration. With no arguments, the first
        # configuration will be the active one
        self.dev.set_configuration()

        # get an endpoint instance
        cfg = self.dev.get_active_configuration()
        intf = cfg[(0,0)]

        self.ep_out = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match = \
            lambda e: \
                usb.util.endpoint_direction(e.bEndpointAddress) == \
                usb.util.ENDPOINT_OUT)

        self.ep_in = usb.util.find_descriptor(
            intf,
            # match the first IN endpoint
            custom_match = \
            lambda e: \
                usb.util.endpoint_direction(e.bEndpointAddress) == \
                usb.util.ENDPOINT_IN)

        assert (self.ep_out and self.ep_in) is not None

        # Confirm connection to user
        resp = self.command('VE?')
        print("Connected to Motor Controller Model {}. Firmware {} {} {}\n".format(
                                                    *resp.split(' ')
                                                    ))
        for m in range(1,5):
            resp = self.command("{}QM?".format(m))
            print("Motor #{motor_number}: {status}".format(
                                                    motor_number=m,
                                                    status=MOTOR_TYPE[resp[-1]]
                                                    ))



    def send_command(self, usb_command, get_reply=False):
        """Send command to USB device endpoint

        Args:
            usb_command (str): Correctly formated command for USB driver
            get_reply (bool): query the IN endpoint after sending command, to
                get controller's reply

        Returns:
            Character representation of returned hex values if a reply is
                requested
        """
        self.ep_out.write(usb_command)
        if get_reply:
            return self.ep_in.read(100)


    def parse_command(self, newfocus_command):
        """Convert a NewFocus style command into a USB command

        Args:
            newfocus_command (str): of the form xxAAnn
                > The general format of a command is a two character mnemonic (AA).
                Both upper and lower case are accepted. Depending on the command,
                it could also have optional or required preceding (xx) and/or
                following (nn) parameters.
                cite [2 - 6.1.2]
        """
        m = NEWFOCUS_COMMAND_REGEX.match(newfocus_command)

        # Check to see if a regex match was found in the user submitted command
        if m:

            # Extract matched components of the command
            driver_number, command, parameter = m.groups()


            usb_command = command

            # Construct USB safe command
            if driver_number:
                usb_command = '1>{driver_number} {command}'.format(
                                                    driver_number=driver_number,
                                                    command=usb_command
                                                    )
            if parameter:
                usb_command = '{command} {parameter}'.format(
                                                    command=usb_command,
                                                    parameter=parameter
                                                    )

            usb_command += '\r'

            return usb_command
        else:
            print("ERROR! Command {} was not a valid format".format(
                                                            newfocus_command
                                                            ))


    def parse_reply(self, reply):
        """Take controller's reply and make human readable

        Args:
            reply (list): list of bytes returns from controller in hex format

        Returns:
            reply (str): Cleaned string of controller reply
        """

        # convert hex to characters
        reply = ''.join([chr(x) for x in reply])
        return reply.rstrip()


    def command(self, newfocus_command):
        """Send NewFocus formated command

        Args:
            newfocus_command (str): Legal command listed in usermanual [2 - 6.2]

        Returns:
            reply (str): Human readable reply from controller
        """
        usb_command = self.parse_command(newfocus_command)

        # if there is a '?' in the command, the user expects a response from
        # the driver
        if '?' in newfocus_command:
            get_reply = True
        else:
            get_reply = False

        reply = self.send_command(usb_command, get_reply)

        # if a reply is expected, parse it
        if get_reply:
            return self.parse_reply(reply)

    def show_command(self):
        print('''
        Picomotor Command Line
        ---------------------------
        Common Commands:
        RCL.0/1: Restore the default/last saved setting
        AB: Abort motion. Instantaneously stop any motion
        xACnn: Set x axis' acceleration to nn(1~200000, default 100000)
        xVAnn: Set x axis' velocity to nn (1~2000, default 2000)
        SM: Save setting
        xMV[+-]: .....Indefinitely move motor 'x' in + or - direction
        xPAnn: move x axis to target position nn
        xPRnn: .....Move motor 'x' 'nn' steps
        ST: .....Stop all motor movement
        RS: restart device
        \n
        ''')




if __name__ == "__main__":
    # test
    import math
    import numpy as np
    import matplotlib.pyplot as plt
    print('\n\n')
    print('#'*80)
    print('#\tPython controller for NewFocus Picomotor Controller')
    print('#'*80)
    print('\n')

    idProduct = '0x4000'
    idVendor = '0x104d'

    if not (idProduct or idVendor):
        print('Run the following command in a new terminal window:')
        print('\t$ system_profiler SPUSBDataType\n')
        print('Enter Product ID:')
        idProduct = input('> ')
        print('Enter Vendor ID:')
        idVendor = input('> ')
        print('\n')

    # convert hex value in string to hex value
    idProduct = int(idProduct, 16)
    idVendor = int(idVendor, 16)

## Initialize the controller
controller = Controller(idProduct=idProduct, idVendor=idVendor)
controller.show_command()

## Initialize the LabJack
#first two lines open and close the LJ, to get rid of errors.
lj = LJA.LabJackAnalog(identifier="ANY")
lj.close()
lj = LJA.LabJackAnalog(identifier="ANY")
handle = ljm.openS("ANY","ANY","ANY")

print('684_labjack loaded')

input_channel = 2                          # Use AIN2 channel to read the voltage from photodetector

## Set initial coupling power and the threshold for re-coupling power
P_max = 3.14                            # initial coupling power(in watt), if use photo detector this should be voltage
P_thr = P_max*0.85                      # Threshold for start re-coupling
time_start = time.time()                # record the starting time

##  Set Nelder-Mead function parameters
step=50                               # initial step
V_int = P_max
no_improve_thr= 0.9
V_goal = V_int * no_improve_thr
no_improv_break=3
max_iter=100
alpha=1.                               #Reflection param
gamma=2.                               #Expansion param
rho=-0.5                               #Contraction param
sigma=0.5                              #Reduction param
'''
    @param V_int(float): initial voltage read from photo detector
    @param step (float): look-around radius in initial step
    @no_improv_thr,  no_improv_break (float, int): break after no_improv_break iterations with
            an improvement lower than no_improv_thr
    @max_iter (int): always break after this number of iterations.
            Set it to 0 to loop indefinitely.
    @alpha, gamma, rho, sigma (floats): parameters of the algorithm
            (see Wikipedia page for reference)
    return: tuple (best parameter array, best score)
'''

##Start Nelder-Mead loop
controller.command("1DH")                          # Set the current position as home position for all axis
controller.command("2DH")
controller.command("3DH")
controller.command("4DH")

x_start = np.array( [0.,0.,0.,0.])                                # Start all four axis at home position
time_start = time.time()                                                  # record the starting time
# init
dim = len(x_start)
##
prev_best = -1*lj.analog_in(input_channel)             # read the inverse first power
no_improv = 0
res = [[x_start, prev_best]]
best_his=[prev_best]
print("the initial coupling efficiency is: "+str(-prev_best / V_int))
x_his = []

for i in range(dim):                                   # get 4 test points, with the (0,0,0,0) one we get 5 initial test points
    x = copy.copy(x_start)
    x[i] = x[i] + step
    for j in range(1,dim+1):
        controller.command(str(j)+"PA"+str(x[j-1]))      # move the current axis to test position
        time.sleep(0.5)
    score = -1*lj.analog_in(input_channel)                   #read the inverse testing power
    res.append([x, score])

# simplex iter
iters = 0
while True:
    # order
    res.sort(key=lambda x: x[1])                             # sort by power from minimum to maximum
    best = res[0][1]                                         # the mimimum power within 5 testing points
    x_best = res[0][0]                                       # position of the miminum point
    best_his.append(best)
    x_his.append(x_best)
    print('...best so far:', -res[0][1]/ V_int)

    # break after max_iter
    if max_iter and iters >= max_iter:
        break
    iters += 1

    # break if all no_improv_break iterations(10) satisfy the goal efficiency
    if best > -1 * V_int * no_improve_thr:                   # goal efficiency: 0.9 V_int
        no_improv = 0
        prev_best = best
    else:
        no_improv += 1
    if no_improv >= no_improv_break:
        break

    # centroid                                            #calculate the center of all points except the maximum one
    x0 = [0.] * dim
    for tup in res[:-1]:
        for i, c in enumerate(tup[0]):
            x0[i] += c / (len(res)-1)

    # reflection
    xr = x0 + alpha*(x0 - res[-1][0])
    for j in range(1,dim+1):
        controller.command(str(j)+"PA"+str(xr[j-1]))       # move the current axis to reflection position
        time.sleep(0.5)
    rscore = -1*lj.analog_in(input_channel)              # read the inverse power at reflection position
    if res[0][1] <= rscore < res[-2][1]:                 # if new power lays between min and max, keep the new point
        del res[-1]
        res.append([xr, rscore])
        continue

    # expansion
    if rscore < res[0][1]:
        xe = x0 + gamma*(x0 - res[-1][0])
        for j in range(1,dim+1):
            controller.command(str(j)+"PA"+str(xe[j-1]))       # move the current axis to expansion position
            time.sleep(0.5)
        escore =  -1*lj.analog_in(input_channel)             # read the inverse power at expansion position
        if escore < rscore:                                  # if the new point is better, keep it
            del res[-1]
            res.append([xe, escore])
            continue
        else:                                                # if the new point is worse, keep the original(reflection) one
            del res[-1]
            res.append([xr, rscore])
            continue

    # contraction
    xc = x0 + rho*(x0 - res[-1][0])
    for j in range(1,dim+1):
        controller.command(str(j)+"PA"+str(xc[j-1]))           # move the current axis to contraction position
        time.sleep(0.5)
    cscore = -1*lj.analog_in(input_channel)                  # read the inverse power at contraction position
    if cscore < res[-1][1]:                                  # if the new point is better, keep it
        del res[-1]
        res.append([xc, cscore])
        continue

    # reduction
    x1 = res[0][0]
    nres = []
    for tup in res:
        redx = x1 + sigma*(tup[0] - x1)
        for j in range(1,dim+1):
            controller.command(str(j)+"PA"+str(redx[j-1]))         # move the current axis to reduction position to every point
            time.sleep(0.5)
        score = -1*lj.analog_in(input_channel)                   # read the inverse power at reduction position to every point
        nres.append([redx, score])
    res = nres

time_end = time.time()                                         # record the ending time
res.sort(key=lambda x: x[1])                                   # sort by power from minimum to maximum
print(res)
x_best_final = res[0][0]                                       # position of the miminum point
for j in range(1,dim+1):
    controller.command(str(j)+"PA"+str(x_best_final[j-1]))               # move the current axis to final position to every point
    time.sleep(0.5)

score_final = -1*lj.analog_in(input_channel)
print("the final efficiency is:", -score_final/V_int)
controller.command("1DH")                          # Set the current position as home position for all axis
controller.command("2DH")
controller.command("3DH")
controller.command("4DH")
##plot
time_run = np.linspace(0,time_end - time_start,len(best_his))
N_C = -1*np.array(best_his)/V_int
plt.plot(time_run,N_C,'b.-')
plt.axhline(y=no_improve_thr, color='r', linestyle='-')
plt.ylabel("normalized coupling efficiency")
plt.xlabel("time/s")
plt.show()

##save scan data

full_folder = 'C:\Chicago RA\\CeNTREX\\auto alignment\\Nelder-Mead method'
#os.mkdir(full_folder)
filename_data = full_folder +"\\data10.txt"
#np.savetxt(filename, np.transpose([x_forward, B_forward]), header = "x \t B_for[V]")
np.savetxt(filename_data, N_C)
filename_time = full_folder +"\\time10.txt"
np.savetxt(filename_time, time_run)