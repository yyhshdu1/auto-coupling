"""
Picomotor Controller connect & control code

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

import copy
import usb.core
import usb.util
import re

NEWFOCUS_COMMAND_REGEX = re.compile("([0-9]{0,1})([a-zA-Z?]{2,})([0-9+-]*)")
MOTOR_TYPE = {
    "0": "No motor connected",
    "1": "Motor Unknown",
    "2": "'Tiny' Motor",
    "3": "'Standard' Motor"
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
        intf = cfg[(0, 0)]

        self.ep_out = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match=lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_OUT)

        self.ep_in = usb.util.find_descriptor(
            intf,
            # match the first IN endpoint
            custom_match=lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_IN)

        assert (self.ep_out and self.ep_in) is not None

        # Confirm connection to user
        resp = self.command('VE?')
        print("Connected to Motor Controller Model {}. Firmware {} {} {}\n".format(
            *resp.split(' ')
        ))
        for m in range(1, 5):
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
