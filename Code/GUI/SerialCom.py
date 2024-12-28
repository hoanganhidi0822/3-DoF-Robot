# serial_comm.py

import serial
import time

# Set up serial communication (adjust port and baud rate as needed)
def initialize_serial_connection(port='COM9', baudrate=115200):
    """
    Initialize and return the serial connection.
    :param port: The serial port to connect to (default is 'COM6').
    :param baudrate: The baud rate for serial communication (default is 115200).
    :return: The serial object for communication.
    """
    ser = serial.Serial(port, baudrate)
    time.sleep(2)  # Wait for the connection to establish
    return ser

def format_angle(angle):
    """
    Format the angle into a sign and formatted number.
    :param angle: The angle value to format.
    :return: A tuple (sign, formatted_number).
    """
    sign = '1' if angle >= 0 else '0'  # Determine the sign (1 for positive, 0 for negative)
    whole, fractional = divmod(abs(angle), 1)
    formatted_number = f"{int(whole):03d}{int(fractional * 1000):03d}"  # Format the number
    return sign, formatted_number
