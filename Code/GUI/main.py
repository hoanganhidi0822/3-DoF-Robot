from PyQt5 import QtWidgets, QtGui, QtCore
import sys
import pyqtgraph as pg
import numpy as np
from PyQt5.QtGui import QImage, QPixmap
import cv2
# Cửa sổ đăng nhập
class LoginWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        # Thiết lập cửa sổ chính với kích thước 1280x720
        self.setWindowTitle("LOGIN")
        self.resize(1280, 720)

        # Thiết lập ảnh nền cho toàn bộ cửa sổ
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setBrush(self.backgroundRole(), QtGui.QBrush(QtGui.QPixmap("background.png")))
        self.setPalette(palette)

        self.initUI()

    def initUI(self):
        # Tạo layout chính giữa màn hình
        main_layout = QtWidgets.QVBoxLayout(self)
        
        # Tạo khung chứa form đăng nhập, có nền trong suốt
        login_frame = QtWidgets.QFrame(self)
        login_frame.setFixedSize(400, 250)  # Kích thước của khung đăng nhập
        login_frame.setStyleSheet("""
            QFrame {
                background-color: rgba(255, 255, 255, 150);  /* Nền trong suốt nhẹ */
                border-radius: 15px;  /* Bo góc */
                border: 1px solid #ccc;
            }
            QLineEdit, QPushButton {
                border-radius: 10px;
                padding: 10px;
                font-size: 16px;
            }
            QPushButton {
                background-color: #5B9BD5;
                color: white;
            }
            QPushButton:hover {
                background-color: #4A89C5;
            }
        """)

        # Layout cho khung đăng nhập
        layout = QtWidgets.QVBoxLayout()

        # Tạo các widget và thêm vào layout
        self.username = QtWidgets.QLineEdit(self)
        self.username.setPlaceholderText("User name")
        
        self.password = QtWidgets.QLineEdit(self)
        self.password.setPlaceholderText("Password")
        self.password.setEchoMode(QtWidgets.QLineEdit.Password)
        
        self.login_button = QtWidgets.QPushButton("Login", self)
        self.login_button.clicked.connect(self.check_credentials)
        
        # Thêm các widget vào layout
        layout.addWidget(self.username)
        layout.addWidget(self.password)
        layout.addWidget(self.login_button)
        
        login_frame.setLayout(layout)
        
        # Thêm khung đăng nhập vào layout chính giữa màn hình
        main_layout.addStretch()
        main_layout.addWidget(login_frame, alignment=QtCore.Qt.AlignCenter)
        main_layout.addStretch()

    def check_credentials(self):
        if self.username.text() == "1" and self.password.text() == "1":
            self.switch_window()
        else:
            QtWidgets.QMessageBox.warning(self, "Error", "Wrong user name or password!")
            
    def switch_window(self):
        self.mode_window = ModeWindow()
        self.mode_window.show()
        self.close()    
           
# Cửa sổ chọn chế độ
class ModeWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Chọn chế độ robot")
        self.resize(1280, 720)
        
        # Ảnh nền
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setBrush(self.backgroundRole(), QtGui.QBrush(QtGui.QPixmap("image.png")))
        self.setPalette(palette)

        self.initUI()

    def initUI(self):
        # Layout dọc chính
        main_layout = QtWidgets.QVBoxLayout(self)
        
        # Tạo khoảng trống phía trên
        main_layout.addStretch()

        # Nhãn tiêu đề
        label = QtWidgets.QLabel("Robot Mode", self)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("font-size: 32px; font-weight: bold;")
        main_layout.addWidget(label)
        
        # Layout ngang cho các nút
        button_layout = QtWidgets.QHBoxLayout()
        
        # Tạo nút và bo tròn các cạnh
        self.manual_button = QtWidgets.QPushButton("Manual", self)
        self.auto_button = QtWidgets.QPushButton("Auto", self)
        self.style_buttons([self.manual_button, self.auto_button])
        self.manual_button.clicked.connect(self.open_manual_mode)
        self.auto_button.clicked.connect(self.open_auto_mode)
        
        button_layout.addStretch()  # Tạo khoảng trống bên trái
        button_layout.addWidget(self.manual_button)
        button_layout.addStretch()  # Khoảng trống giữa 2 nút
        button_layout.addWidget(self.auto_button)
        button_layout.addStretch()  # Tạo khoảng trống bên phải
        
        # Thêm layout ngang vào layout chính
        main_layout.addLayout(button_layout)
        
        # Tạo khoảng trống phía dưới
        main_layout.addStretch()

    def style_buttons(self, buttons):
        for button in buttons:
            button.setFixedSize(320, 100)
            button.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border-radius: 20px;
                    font-size: 24px;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
            """)

    def open_manual_mode(self):
        self.manual_window = ManualWindow()
        self.manual_window.show()
        self.close()

    def open_auto_mode(self):
        self.auto_window = AutoWindow()
        self.auto_window.show()
        self.close()

class ManualWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Manual Mode")
        self.resize(1280, 720)
        self.theta1 = self.theta2 = self.theta3 = 90
        self.x = self.y = self.z = 0
        self.initUI()

    def initUI(self):
        main_layout = QtWidgets.QVBoxLayout(self)

        # Layout cho nút Back ở góc trên bên trái
        back_button_layout = QtWidgets.QHBoxLayout()
        self.back_button = QtWidgets.QPushButton("Back", self)
        self.back_button.setFixedSize(100, 40)
        self.back_button.setStyleSheet("""
            QPushButton {
                background-color: #FF5722;
                color: white;
                border-radius: 10px;
                font-size: 18px;
            }
            QPushButton:hover {
                background-color: #E64A19;
            }
        """)
        self.back_button.clicked.connect(self.go_back)

        back_button_layout.addWidget(self.back_button)
        back_button_layout.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)

        # Khu vực điều khiển góc và vị trí (2/3 trên)
        control_layout = QtWidgets.QVBoxLayout()
        self.create_theta_controls(control_layout)
        self.create_position_controls(control_layout)
        self.create_buttons(control_layout)

        main_layout.addLayout(back_button_layout)  # Thêm layout nút Back
        main_layout.addLayout(control_layout, stretch=2)

        # Khu vực hiển thị đồ thị (1/3 dưới)
        self.create_scope_display(main_layout)

        main_layout.addStretch()
    
    # --- Phần điều khiển góc và vị trí ---
    def create_theta_controls(self, layout):
        theta_labels = ["Theta 1", "Theta 2", "Theta 3"]
        self.sliders = []
        self.inputs = []

        # Tạo khung bao quanh các điều khiển góc
        frame = QtWidgets.QFrame(self)
        frame.setStyleSheet("""
            QFrame {
                background-color: #f0f0f0;
                border: 2px solid #4CAF50;
                border-radius: 20px;
            }
        """)
        frame.setFixedHeight(250)
        frame_layout = QtWidgets.QVBoxLayout(frame)

        for i in range(3):
            h_layout = QtWidgets.QHBoxLayout()

            # Nhãn Theta
            theta_label = QtWidgets.QLabel(theta_labels[i], self)
            theta_label.setFixedWidth(80)
            theta_label.setStyleSheet("font-size: 18px; font-weight: bold;")
            
            # Slider
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
            slider.setMinimum(0)
            slider.setMaximum(180)
            slider.setValue(90)
            slider.setTickInterval(10)
            slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
            slider.setFixedWidth(400)
            slider.setStyleSheet("""
                QSlider::groove:horizontal {
                    background: #ccc;
                    border-radius: 5px;
                    height: 10px;
                }
                QSlider::handle:horizontal {
                    background: #4CAF50;
                    border: 1px solid #45a049;
                    width: 20px;
                    margin: -5px 0;
                    border-radius: 10px;
                }
            """)
            self.sliders.append(slider)
            
            # Ô nhập liệu
            input_box = QtWidgets.QLineEdit(self)
            input_box.setFixedWidth(100)
            input_box.setText("90")
            input_box.setAlignment(QtCore.Qt.AlignCenter)
            input_box.setStyleSheet("""
                QLineEdit {
                    border: 2px solid #4CAF50;
                    border-radius: 10px;
                    padding: 5px;
                    font-size: 18px;
                }
            """)
            self.inputs.append(input_box)

            slider.valueChanged.connect(lambda value, index=i: self.update_input(value, index))
            input_box.editingFinished.connect(lambda index=i: self.update_slider(index))

            h_layout.addWidget(theta_label)
            h_layout.addWidget(slider)
            h_layout.addWidget(input_box)
            frame_layout.addLayout(h_layout)

        layout.addWidget(frame)

    def create_position_controls(self, layout):
        position_labels = ["X", "Y", "Z"]
        self.position_inputs = []

        # Tạo khung bao quanh các điều khiển vị trí
        frame = QtWidgets.QFrame(self)
        frame.setStyleSheet("""
            QFrame {
                background-color: #f0f0f0;
                border: 2px solid #2196F3;
                border-radius: 20px;
            }
        """)
        frame.setFixedHeight(150)
        frame_layout = QtWidgets.QVBoxLayout(frame)

        for label_text in position_labels:
            h_layout = QtWidgets.QHBoxLayout()
            label = QtWidgets.QLabel(f"{label_text}:", self)
            label.setFixedWidth(50)
            label.setStyleSheet("font-size: 18px; font-weight: bold;")
            
            input_box = QtWidgets.QLineEdit(self)
            input_box.setFixedWidth(150)
            input_box.setText("0")
            input_box.setAlignment(QtCore.Qt.AlignCenter)
            input_box.setStyleSheet("""
                QLineEdit {
                    border: 2px solid #2196F3;
                    border-radius: 10px;
                    padding: 5px;
                    font-size: 18px;
                }
            """)
            self.position_inputs.append(input_box)

            h_layout.addWidget(label)
            h_layout.addWidget(input_box)
            frame_layout.addLayout(h_layout)

        layout.addWidget(frame)

    def create_buttons(self, layout):
        # Tạo khung bao quanh các nút
        button_frame = QtWidgets.QFrame(self)
        button_frame.setStyleSheet("""
            QFrame {
                background-color: #f0f0f0;
                border: 2px solid #2196F3;
                border-radius: 20px;
            }
        """)
        button_layout = QtWidgets.QHBoxLayout(button_frame)

        forward_frame = self.create_button_frame("Forward", self.save_theta_values, "#2196F3", "#1976D2")
        inverse_frame = self.create_button_frame("Inverse", self.save_position_values, "#FF9800", "#FB8C00")
        autohome_frame = self.create_button_frame("Auto Home", self.auto_home_function, "#9C27B0", "#7B1FA2")

        button_layout.addWidget(forward_frame)
        button_layout.addWidget(inverse_frame)
        button_layout.addWidget(autohome_frame)
        
        layout.addWidget(button_frame)
    def create_button_frame(self, button_text, function, color, hover_color):
        frame = QtWidgets.QFrame(self)
        frame.setStyleSheet(f"border: 2px solid {color}; border-radius: 20px; padding: 10px;")
        layout = QtWidgets.QVBoxLayout(frame)
        button = QtWidgets.QPushButton(button_text, self)
        button.setFixedSize(200, 60)
        button.setStyleSheet(f"background-color: {color}; color: white; border-radius: 20px; font-size: 20px;")
        button.clicked.connect(function)
        layout.addWidget(button, alignment=QtCore.Qt.AlignCenter)
        return frame

    # --- Phần đồ thị ---
    
    ###
    def update_sine_wave(self):
        self.theta += 0.1  # Tăng góc theta
        x_data = np.linspace(self.theta, self.theta + 2 * np.pi, 100)
        y_data = np.sin(x_data)

        for scope in self.scopes:
            scope.clear()
            scope.plot(x_data, y_data, pen=pg.mkPen('b', width=2))
    ###
    def create_scope_display(self, layout):
        scope_frame = QtWidgets.QFrame(self)
        scope_frame.setFixedHeight(240)
        scope_frame.setStyleSheet("""
            QFrame {
                border: 2px solid #424949;
                border-radius: 2px;
                background-color: #f9f9f9;
            }
        """)
        scope_layout = QtWidgets.QHBoxLayout(scope_frame)
        
        self.scopes = []
        for i in range(3):
            plot_widget = pg.PlotWidget()
            plot_widget.setYRange(-90, 90)
            plot_widget.setBackground('white')
            plot_widget.showGrid(x=True, y=True)
            plot_widget.setLabel('left', f'Theta {i+1} (deg)')
            plot_widget.setLabel('bottom', 'Time')
            self.scopes.append(plot_widget)
            scope_layout.addWidget(plot_widget)
        
        layout.addWidget(scope_frame)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_scopes)
        self.timer.start(100)  # Cập nhật mỗi 100ms
        
        self.time_data = np.linspace(0, 4*np.pi, 100)
        self.index = 0

    def update_scopes(self):
        for i in range(3):
            theta_value = np.sin(self.time_data + self.index) * 90  # Giới hạn từ -90 đến 90
            self.scopes[i].plot(self.time_data, theta_value, clear=True, pen=pg.mkPen('#e74c3c', width=3))
        self.index += 0.1

    # Các hàm xử lý tương tự
    def update_input(self, value, index):
        self.inputs[index].setText(str(value))
    
    def update_slider(self, index):
        try:
            value = int(self.inputs[index].text())
            self.sliders[index].setValue(value)
        except ValueError:
            pass
    
    def auto_home_function(self):
        # Example action: Reset sliders and input boxes to home positions (90 for angles, 0 for positions)
        for slider, input_box in zip(self.sliders, self.inputs):
            slider.setValue(90)
            input_box.setText("90")
        for input_box in self.position_inputs:
            input_box.setText("0")
        print("Auto Home activated: All values reset.")

    def go_back(self):
        # Example action: Return to the previous mode selection window
        self.close()
        self.mode_window = ModeWindow()  # Assuming ModeWindow is the previous window
        self.mode_window.show()
    
    def save_theta_values(self):
        self.theta1 = self.sliders[0].value()
        self.theta2 = self.sliders[1].value()
        self.theta3 = self.sliders[2].value()
        print(f"Theta values saved: Theta1={self.theta1}, Theta2={self.theta2}, Theta3={self.theta3}")

    def save_position_values(self):
        self.x = int(self.position_inputs[0].text())
        self.y = int(self.position_inputs[1].text())
        self.z = int(self.position_inputs[2].text())
        print(f"Position values saved: X={self.x}, Y={self.y}, Z={self.z}")


        

class AutoWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Auto Mode")
        self.resize(1280, 720)
        self.time = 0  # Biến thời gian ban đầu

        layout = QtWidgets.QVBoxLayout(self)

        # Nút quay về
        back_button = QtWidgets.QPushButton("Back", self)
        back_button.setFixedSize(120, 50)
        # Nút quay về (Back)
        back_button.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                border-radius: 15px;  /* Bo tròn góc */
                font-size: 18px;
            }
            QPushButton:hover {
                background-color: #d32f2f;
            }
        """)
        back_button.clicked.connect(self.go_back)

        back_layout = QtWidgets.QHBoxLayout()
        back_layout.addWidget(back_button)
        back_layout.addStretch()

        # Tiêu đề
        label = QtWidgets.QLabel("Chế độ Auto", self)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("font-size: 32px; font-weight: bold;")

        # Camera view (960x720)
        self.camera_label = QtWidgets.QLabel(self)
        self.camera_label.setFixedSize(640, 480)
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setStyleSheet(" background-color: #f0f0f0;")

        # Nút Run
        self.run_button = QtWidgets.QPushButton("Run", self)
        self.run_button.setFixedSize(200, 80)
        # Nút Run
        self.run_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border-radius: 15px;  /* Bo tròn góc */
                font-size: 24px;
            }
            QPushButton:hover {
                background-color: #388E3C;
            }
        """)
        self.run_button.clicked.connect(self.start_run)

        camera_button_layout = QtWidgets.QHBoxLayout()
        camera_button_layout.addWidget(self.camera_label)
        camera_button_layout.addWidget(self.run_button)

        

        # Thêm layout chứa các scope
        scope_layout = QtWidgets.QHBoxLayout()
        self.scopes = []

        # Frame chứa camera và nút Run
        # Frame chứa camera và nút Run (loại bỏ khung)
        frame = QtWidgets.QFrame(self)
        frame.setStyleSheet("""
            QFrame {
                border: none;  /* Loại bỏ viền */
                padding: 5px;  /* Giữ padding nếu cần thiết */
            }
        """)
        frame.setLayout(camera_button_layout)

        # Tạo khung bao quanh các scope
        scope_frame = QtWidgets.QFrame(self)
        scope_frame.setStyleSheet("""
            QFrame {
                border: 3px solid #000000;  /* Khung ngoài */
                border-radius: 10px;
                padding: 5px;
                background-color: #f0f0f0;
            }
        """)
        scope_frame.setLayout(scope_layout)

        # Loại bỏ viền của các scope (PlotWidget)
        for i in range(3):
            plot_widget = pg.PlotWidget(title=f"Scope {i+1}")
            plot_widget.setFixedSize(300, 200)
            plot_widget.setBackground('w')
            plot_widget.showGrid(x=True, y=True)
            plot_widget.plotItem.setLabel('left', 'Amplitude')
            plot_widget.plotItem.setLabel('bottom', 'Theta')
            plot_widget.setStyleSheet("border: none;")  # Loại bỏ viền của scope
            self.scopes.append(plot_widget)
            scope_layout.addWidget(plot_widget)

        
        # Layout chứa camera và nút Run
        camera_button_layout = QtWidgets.QHBoxLayout()
        camera_button_layout.addWidget(self.camera_label)
        camera_button_layout.addWidget(self.run_button)

        # Frame chứa camera và nút Run (đã loại bỏ viền)
        frame = QtWidgets.QFrame(self)
        frame.setStyleSheet("""
            QFrame {
                border: none;
                padding: 5px;
            }
        """)
        frame.setLayout(camera_button_layout)

        # Tạo khung chứa cả frame và nút Run
        camera_container_frame = QtWidgets.QFrame(self)
        camera_container_frame.setStyleSheet("""
            QFrame {
                border: 3px solid #000000;  /* Tạo khung bao ngoài */
                border-radius: 10px;
                padding: 10px;
                background-color: #f0f0f0;  /* Màu nền */
            }
        """)
        camera_container_layout = QtWidgets.QVBoxLayout(camera_container_frame)
        camera_container_layout.addWidget(frame)  # Thêm frame vào container

        # Thêm frame chứa vào layout chính
        layout.addLayout(back_layout)
        layout.addWidget(label)
        layout.addWidget(camera_container_frame)  # Thêm container frame vào layout chính
        layout.addWidget(scope_frame)
        layout.addStretch()


        # Cài đặt camera
        self.capture = cv2.VideoCapture(0)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(30)

        # Cài đặt theta cho sine wave
        self.theta = 0
        self.sin_timer = QtCore.QTimer(self)
        self.sin_timer.timeout.connect(self.update_sine_wave)
        self.sin_timer.start(100)  # Cập nhật mỗi 100ms

    def update_image(self):
        ret, frame = self.capture.read()
        if ret:
            height, width, channels = frame.shape
            bytes_per_line = channels * width
            q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(q_img)
            self.camera_label.setPixmap(pixmap.scaled(640, 480, QtCore.Qt.KeepAspectRatio))

    def update_sine_wave(self):
        # Tăng giá trị thời gian (time)
        self.time += 0.1  # Tăng 0.1 giây mỗi lần cập nhật
        x_data = np.linspace(self.time, self.time + 10, 100)  # Khoảng time hiện tại và 10 đơn vị tiếp theo
        
        # Tạo dữ liệu y (biên độ từ -90 đến 90)
        y_data = 90 * np.sin(x_data)  # Nhân với 90 để điều chỉnh biên độ
        
        # Cập nhật các scope
        for scope in self.scopes:
            scope.clear()
            scope.plot(x_data, y_data, pen=pg.mkPen('b', width=2))
            scope.getAxis('left').setTicks([[(x, str(round(x, 2))) for x in np.arange(-90, 90, 20)]])
            scope.plotItem.setLabel('left', 'Amplitude')
            scope.plotItem.setLabel('bottom', 'Time (s)')

    def start_run(self):
        print("Run button clicked")

    def go_back(self):
        self.mode_window = ModeWindow()
        self.mode_window.show()
        self.close()

        
# Chạy ứng dụng
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = LoginWindow()
    window.show()
    sys.exit(app.exec_())
