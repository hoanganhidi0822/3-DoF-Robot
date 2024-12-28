import serial

def read_uart(port, baudrate, timeout=1):
    try:
        # Mở kết nối UART
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"Connected to {port} at {baudrate} baud.")
        
        while True:
            # Đọc dữ liệu từ UART
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                print(f"Received: {data}")
    
    except serial.SerialException as e:
        print(f"Error: {e}")
    
    except KeyboardInterrupt:
        print("Exiting...")
    
    finally:
        # Đóng kết nối khi hoàn thành
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")

# Cấu hình UART
if __name__ == "__main__":
    port = "COM10"  # Thay bằng cổng UART của bạn (ví dụ: "/dev/ttyUSB0" trên Linux)
    baudrate = 115200
    read_uart(port, baudrate)
