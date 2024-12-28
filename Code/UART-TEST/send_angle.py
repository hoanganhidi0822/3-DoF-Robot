import serial
import time

# Set up serial communication (adjust port and baud rate as needed)
ser = serial.Serial('COM6', 115200)  # Change 'COM6' to your serial port
time.sleep(2)  # Wait for connection to establish

def format_angle(angle):
    # Determine the sign (1 for positive, 0 for negative)
    sign = '1' if angle >= 0 else '0'
    # Convert angle to absolute value and split into whole and fractional parts
    whole, fractional = divmod(abs(angle), 1)
    # Format the whole part with zero-padding for 3 digits and fractional for 3 digits
    formatted_number = f"{int(whole):03d}{int(fractional * 1000):03d}"
    return sign, formatted_number

try:
    while True:  # Continuous loop for sending data
        # Predefined angle values
        angle1 = 90.00
        angle2 = 90
        angle3 = -145.00

        # Convert each angle
        sign1, data1 = format_angle(angle1)
        sign2, data2 = format_angle(angle2)
        sign3, data3 = format_angle(angle3)

        # Combine all into a single string with spaces for clarity
        data_to_send = f"{sign1}{data1}{sign2}{data2}{sign3}{data3}"
        
        # Send the formatted string
        ser.write(data_to_send.encode())  # Encode the string to bytes before sending
        print(f"Sent: {data_to_send}")  # Confirm what was sent
        
        time.sleep(1)  # Delay between sends

except KeyboardInterrupt:
    print("\nProgram terminated by user.")

finally:
    ser.close()  # Close the serial connection
