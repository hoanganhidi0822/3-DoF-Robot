import serial
import time

# Set up serial communication (adjust port and baudrate as needed)
ser = serial.Serial('COM6', 115200)  # Change 'COM3' to your serial port
time.sleep(2)  # Wait for connection to establish

try:
    while True:  # Continuous loop for sending data
        # Get angles from user input
        angle1 = 10.0
        angle2 = 10.0
        angle3 = 10.0

        # Format the angles into a single string, separated by commas
        data_to_send = "{:.2f},{:.2f},{:.2f}\n".format(angle1, angle2, angle3)
        
        # Send the formatted string
        ser.write(data_to_send.encode())
        print(f"Sent: {data_to_send.strip()}")  # Confirm what was sent
        
        time.sleep(1)  # Optional delay between sends

except KeyboardInterrupt:
    print("\nProgram terminated by user.")

finally:
    ser.close()  # Close the serial port safely
