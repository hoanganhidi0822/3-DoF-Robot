import serial
import time

# Set up serial communication (adjust port and baud rate as needed)
ser = serial.Serial('COM6', 115200, timeout=1)  # Change 'COM3' to your port

try:
    print("Waiting for data...")
    while True:  # Continuous loop to read data
        if ser.in_waiting > 0:  # Check if data is available to read
            received_data = ser.readline()# Read and decode the data
            print(f"Received: {received_data}")  # Print the received data
        
        time.sleep(1)  # Optional small delay to reduce CPU usage

except KeyboardInterrupt:
    print("\nProgram terminated by user.")

finally:
    ser.close()  # Close the serial port safely
