import cv2
import numpy as np
from Kinematic import IK_T6
from test_serial import initialize_serial_connection, send_angles_to_robot

def pixel_to_real_coordinates(detected_bolts):
    # Conversion factor
    pixel_to_cm = 0.045  # 1 pixel = 0.04 cm

    # Real-life origin offset in cm
    origin_offset_x = -13.7  # cm
    origin_offset_y = 0      # cm

    # List to store real-world coordinates
    real_coordinates = []

    # Convert each detected bolt's coordinates
    for (x, y) in detected_bolts:
        x_real = x * pixel_to_cm + origin_offset_x
        y_real = y * pixel_to_cm + origin_offset_y
        real_coordinates.append((x_real, y_real))

    return real_coordinates

def process_image(frame):
    # Chuyển đổi sang ảnh xám
    frame_ = frame[0:480, 60:620]
    gray = cv2.cvtColor(frame_, cv2.COLOR_BGR2GRAY)

    # Áp dụng threshold để tách nền trắng
    _, thresh = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY_INV)

    # Tìm contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected_bolts = []
    drop_targets = []

    # Sắp xếp các contours từ trái qua phải, từ dưới lên trên
    contours = sorted(contours, key=lambda c: (cv2.boundingRect(c)[1], cv2.boundingRect(c)[0]))

    for contour in contours:
        # Lọc contours theo kích thước (diện tích)
        area = cv2.contourArea(contour)
        # print(area)
        if 200 < area < 400:  # Giới hạn diện tích cho ốc vít
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / float(h)
            if 0.5 < aspect_ratio < 2:  # Hình dạng hợp lệ
                cv2.rectangle(frame_, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame_, (x + w // 2, y + h // 2), 3, (0, 0, 255), -1)
                detected_bolts.append((x + w, y + h))
        elif 19000 < area < 25000:  # Giới hạn diện tích cho mục tiêu thả
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame_, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Vẽ bounding box màu xanh dương
            drop_targets.append((x + w // 2, y + h // 2))

    # Chuyển đổi tọa độ pixel sang tọa độ thực tế
    real_coordinates = pixel_to_real_coordinates(detected_bolts)
    drop_coordinates = pixel_to_real_coordinates(drop_targets)

    # Vẽ tọa độ thực tế lên ảnh
    for i, coord in enumerate(real_coordinates):
        x, y = detected_bolts[i]
        x_real, y_real = coord
        cv2.putText(
            frame_,
            f"({x_real:.1f}, {y_real:.1f}) mm",
            (x + 5, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            1
        )

    return frame_, detected_bolts, real_coordinates, drop_coordinates, thresh


def main():
    # Mở camera
    cap = cv2.VideoCapture(1)  # Sử dụng camera mặc định (thay 0 bằng ID camera nếu cần)

    if not cap.isOpened():
        print("Không thể mở camera")
        return

    pickup_and_drop_coordinates = []  # Mảng lưu tọa độ gắp và thả
    detected_bolt_count = 0  # Đếm số ốc đã phát hiện
    ser = initialize_serial_connection(port='COM26', baudrate=115200)

    pickup_z = 0.25  # Tọa độ Z của điểm gắp (có thể tùy chỉnh)
    drop_z = 3   # Tọa độ Z của điểm thả (có thể tùy chỉnh)

    while True:
        # Đọc khung hình từ camera
        ret, frame = cap.read()
        if not ret:
            print("Không thể đọc khung hình từ camera")
            break

        # Xử lý ảnh
        processed_frame, bolts, real_coordinates, drop_coordinates, thresh = process_image(frame)

        # Cập nhật tọa độ thực tế
        if real_coordinates:
            pickup_and_drop_coordinates.extend(real_coordinates)
            detected_bolt_count += len(real_coordinates)

        # Nếu đã phát hiện đủ 5 ốc, thêm tọa độ điểm thả
        if detected_bolt_count >= 4 and len(pickup_and_drop_coordinates) == 5 and drop_coordinates:
            drop_point = (drop_coordinates[0][0], drop_coordinates[0][1], drop_z)  # Tọa độ điểm thả với Z tùy chỉnh
            pickup_and_drop_coordinates.append(drop_point)

            # Chuyển đổi tọa độ thành góc
            angles_list = []
            for coord in pickup_and_drop_coordinates:
                x, y = coord[1], coord[0]
                z = pickup_z if coord != drop_point else drop_z  # Gán Z tùy chỉnh
                angle1, angle2, angle3 = IK_T6(x, y, z, 2)
                angles_list.append((angle1, angle2, angle3))

            print("Converted Angles (IK):", angles_list)
            send_angles_to_robot(ser, angles_list)
            break

        # Hiển thị ảnh kết quả
        cv2.imshow("Thresh", thresh)
        cv2.imshow("Detected Bolts", processed_frame)

        # Nhấn phím 'D' để thực hiện lại
        key = cv2.waitKey(1)
        if key == ord('d') or key == ord('D'):
            continue
        elif key == 27:  # Nhấn ESC để thoát
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()