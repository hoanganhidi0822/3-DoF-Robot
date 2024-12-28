import cv2
import numpy as np

def pixel_to_real_coordinates(detected_bolts):
    # Conversion factor
    pixel_to_cm = 0.045  # 1 pixel = 0.04 cm

    # Real-life origin offset in cm
    origin_offset_x = -13.7 # cm
    origin_offset_y = 0     # cm

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

    for contour in contours:
        # Lọc contours theo kích thước (diện tích)
        area = cv2.contourArea(contour)
        if 120 < area < 350:  # Giới hạn diện tích
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / float(h)
            if 0.8 < aspect_ratio < 2:  # Hình dạng hợp lệ
                cv2.rectangle(frame_, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame_, (x + w // 2, y + h // 2), 3, (0, 0, 255), -1)
                detected_bolts.append((x + w, y + h))

    # Chuyển đổi tọa độ pixel sang tọa độ thực tế
    real_coordinates = pixel_to_real_coordinates(detected_bolts)

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

    return frame_, detected_bolts, real_coordinates, thresh

def main():
    # Mở camera
    cap = cv2.VideoCapture(1)  # Sử dụng camera mặc định (thay 0 bằng ID camera nếu cần)

    if not cap.isOpened():
        print("Không thể mở camera")
        return
    real_coordinates_list = []  # Mảng lưu tọa độ thực tế
    while True:
        # Đọc khung hình từ camera
        ret, frame = cap.read()
        if not ret:
            print("Không thể đọc khung hình từ camera")
            break

        # Xử lý ảnh
        processed_frame, bolts, real_coordinates, thresh = process_image(frame)

        # Lưu tọa độ thực tế vào mảng
        if real_coordinates:
            real_coordinates_list.extend(real_coordinates)

        # Hiển thị ảnh kết quả
        cv2.imshow("Thresh", thresh)
        cv2.imshow("Detected Bolts", processed_frame)

        # In giá trị bounding boxes và tọa độ thực tế
        # print("Detected bolts (bounding boxes):", bolts)
        print("Real-world coordinates:", real_coordinates)

        # Nhấn phím 'D' để thực hiện lại
        key = cv2.waitKey(1)
        if key == ord('d') or key == ord('D'):
            continue
        elif key == 27:  # Nhấn ESC để thoát
            break

    # In toàn bộ tọa độ thực tế đã lưu
    # print("All detected real-world coordinates:", real_coordinates_list)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()