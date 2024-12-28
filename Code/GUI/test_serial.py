# serial_comm.py

import serial
import time

# Set up serial communication (adjust port and baud rate as needed)
def initialize_serial_connection(port='COM11', baudrate=115200):
    """
    Initialize and return the serial connection.
    :param port: The serial port to connect to (default is 'COM9').
    :param baudrate: The baud rate for serial communication (default is 115200).
    :return: The serial object for communication.
    """
    ser = serial.Serial(port, baudrate)
    # ser = 1
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

def format_angles_for_robot(angles):
    """
    Format a list of angles for sending to the robot.
    :param angles: A list of tuples [(angle1, angle2, angle3), ...].
    :return: A formatted string to send to the robot.
    """
    formatted_data = []
    for angle_set in angles:
        formatted_angles = []
        for angle in angle_set:
            sign, formatted_number = format_angle(angle)
            formatted_angles.append(f"{sign}{formatted_number}")
        formatted_data.append("".join(formatted_angles))
    return "".join(formatted_data)

def send_angles_to_robot(serial_conn, angles):
    """
    Send formatted angles to the robot over serial communication.
    :param serial_conn: The initialized serial connection.
    :param angles: A list of tuples [(angle1, angle2, angle3), ...].
    """
    formatted_message = format_angles_for_robot(angles)
    serial_conn.write((formatted_message + "\n").encode('utf-8'))
    print(f"Sent to robot: {len(formatted_message)}")

# # Example usage:
# if __name__ == "__main__":
#     # Initialize serial connection
#     ser = initialize_serial_connection(port='COM11', baudrate=115200)

#     # Example angles to send [(angle1, angle2, angle3), ...]
#     example_angles = [(45.123, -30.456, 10.789), (90.000, 0.123, -45.678)]

#     # Send angles to robot
#     send_angles_to_robot(ser,example_angles)

#     # Close the serial connection
#     ser.close()
