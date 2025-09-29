import socket
import time
import path
from support_main.lib_main import edit_csv_tab
import numpy as np
import pyperclip
import struct
import os
import threading
from PyQt6 import QtCore, QtGui, QtWidgets


# Cấu hình mặc định (sẽ được override bằng file setting)
UDP_PORT_1 = 2368
UDP_PORT_2 = 2369

path_phan_mem = path.path_phan_mem
path_admin = path_phan_mem + "/setting/admin_window.csv"
if os.name == "nt":
    print("Hệ điều hành là Windows")
    path_admin = path_phan_mem + "/setting/admin_window.csv"
elif os.name == "posix":
    print("Hệ điều hành là Ubuntu (Linux)")
    path_admin = path_phan_mem + "/setting/admin_ubuntu.csv"

# Đọc cài đặt từ file CSV
data_admin = edit_csv_tab.load_all_stt(path_admin)
host_lidar_1, port_lidar_1 = "192.168.1.10", 2368
host_lidar_2, port_lidar_2 = "192.168.1.11", 2369

for row in data_admin:
    if len(row) > 1:
        if row[0] == "host_lidar_1":
            host_lidar_1 = row[1]
        if row[0] == "port_lidar_1":
            port_lidar_1 = int(float(row[1]))
        if row[0] == "host_lidar_2":
            host_lidar_2 = row[1]
        if row[0] == "port_lidar_2":
            port_lidar_2 = int(float(row[1]))

print("Config:", host_lidar_1, port_lidar_1, host_lidar_2, port_lidar_2)


class LidarP:
    def __init__(self):
        self.sock_1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_1.bind(("0.0.0.0", port_lidar_1))
        self.sock_1.settimeout(1.0)

        self.sock_2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_2.bind(("0.0.0.0", port_lidar_2))
        self.sock_2.settimeout(1.0)

        self.timer_1 = QtCore.QTimer()
        self.final_data_1 = np.array([[0, 0, 0]])
        self.final_data_old_1 = np.array([[0, 0, 0]])
        self.final_data_new_1 = []
        self.data_ok_1 = 0

        self.timer_2 = QtCore.QTimer()
        self.final_data_2 = np.array([[0, 0, 0]])
        self.final_data_old_2 = np.array([[0, 0, 0]])
        self.final_data_new_2 = []
        self.data_ok_2 = 0

        self.connect = True

    def decode_data(self, data):
        """Giải mã dữ liệu từ LR-1BS3/5 (gói 8 byte)."""
        if len(data) != 8:
            raise ValueError("Dữ liệu đầu vào phải là 8 byte.")
        angle_raw, distance_raw, signal_raw, _ = struct.unpack("<HHHH", data)
        angle = angle_raw * 0.01
        distance = distance_raw
        signal = signal_raw
        return angle, distance, signal, _

    def get_data(self):
        """Lấy dữ liệu quét mới nhất từ cả hai Lidar."""
        output_1, check_1 = self.final_data_old_1, False
        if self.data_ok_1 == 1:
            output_1 = self.final_data_1
            self.final_data_old_1 = self.final_data_1
            self.data_ok_1 = 0
            check_1 = True

        output_2, check_2 = self.final_data_old_2, False
        if self.data_ok_2 == 1:
            output_2 = self.final_data_2
            self.final_data_old_2 = self.final_data_2
            self.data_ok_2 = 0
            check_2 = True

        return output_1, output_2, check_1, check_2

    def process_data_1(self):
        """Xử lý dữ liệu cho Lidar 1."""
        while self.connect:
            try:
                data, addr = self.sock_1.recvfrom(1240)
            except socket.timeout:
                continue
            if addr[0] != host_lidar_1:
                continue  # bỏ qua gói không phải từ Lidar 1

            if len(data) < 44:
                continue
            body = data[40:]
            if len(body) % 8 != 0:
                continue
            data0 = np.frombuffer(body, dtype=np.uint16).reshape(-1, 4)
            data = data0[data0[:, 1] != 0]
            angles0 = data0[:, 0] * 0.01
            distances = data[:, 1]
            signals = data[:, 2]
            data_array = np.column_stack((signals, angles0[data0[:, 1] != 0], distances))

            if int(angles0[0]) == 0:
                if self.data_ok_1 == 0:
                    self.final_data_1 = np.array(self.final_data_new_1)
                    self.data_ok_1 = 1
                self.final_data_new_1 = []
            else:
                self.final_data_new_1.extend(data_array)
        print("Lidar 1 thread stopped.")
        self.sock_1.close()

    def process_data_2(self):
        """Xử lý dữ liệu cho Lidar 2."""
        while self.connect:
            try:
                data, addr = self.sock_2.recvfrom(1240)
            except socket.timeout:
                continue
            if addr[0] != host_lidar_2:
                continue  # bỏ qua gói không phải từ Lidar 2

            if len(data) < 44:
                continue
            body = data[40:]
            if len(body) % 8 != 0:
                continue
            data0 = np.frombuffer(body, dtype=np.uint16).reshape(-1, 4)
            data = data0[data0[:, 1] != 0]
            angles0 = data0[:, 0] * 0.01
            distances = data[:, 1]
            signals = data[:, 2]
            data_array = np.column_stack((signals, angles0[data0[:, 1] != 0], distances))

            if int(angles0[0]) == 0:
                if self.data_ok_2 == 0:
                    self.final_data_2 = np.array(self.final_data_new_2)
                    self.data_ok_2 = 1
                self.final_data_new_2 = []
            else:
                self.final_data_new_2.extend(data_array)
        print("Lidar 2 thread stopped.")
        self.sock_2.close()

    def start_processing(self):
        """Bắt đầu các luồng xử lý dữ liệu."""
        threading.Thread(target=self.process_data_1, daemon=True).start()
        threading.Thread(target=self.process_data_2, daemon=True).start()

    def disconnect(self):
        """Dừng các luồng và đóng sockets."""
        self.connect = False
        
        
        print("Lidar sockets closed.")
