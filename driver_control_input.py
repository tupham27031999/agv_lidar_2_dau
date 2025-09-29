import webserver
from support_main import crop_img_Atar, tim_duong_di, connect_driver, music
import numpy as np
import path, os
from support_main.lib_main import edit_csv_tab
import scan_an_toan
import process_lidar
import ket_noi_esp
import threading, math
import controller_motor
import time

path_phan_mem = path.path_phan_mem

distan_scan_all = [0, 10000]
dis_min = 0
dis_max = 2000
anpha_scan = [130,200]
anpha_scan_an_toan_tien = [0,70,280,360]
anpha_scan_an_toan_re_phai = [50, 130, 50, 130]
anpha_scan_an_toan_re_trai =[200, 300, 200, 300]
path_admin = path_phan_mem + "/setting/admin_window.csv"
if os.name == "nt":
    print("Hệ điều hành là Windows")
    # Đọc file cài đặt cho Windows
    path_admin = path_phan_mem + "/setting/admin_window.csv"
    on_music = 1
elif os.name == "posix":
    print("Hệ điều hành là Ubuntu (Linux)")
    # Đọc file cài đặt cho Ubuntu
    path_admin = path_phan_mem + "/setting/admin_ubuntu.csv"
    on_music = 0
data_admin = edit_csv_tab.load_all_stt(path_admin)

for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "khoang_cach_duong_di":
            khoang_cach_duong_di = int(float(data_admin[i][1]))
        if data_admin[i][0] == "khoang_cach_dich":
            khoang_cach_dich = int(float(data_admin[i][1]))
        if data_admin[i][0] == "anpha_scan_an_toan_tien":
            anpha_scan_an_toan_tien = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 
        if data_admin[i][0] == "anpha_scan_an_toan_re_trai":
            anpha_scan_an_toan_re_trai = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 
        if data_admin[i][0] == "anpha_scan_an_toan_re_phai":
            anpha_scan_an_toan_re_phai = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 
        if data_admin[i][0] == "anpha_scan":
            anpha_scan = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        if data_admin[i][0] == "khoang_canh_an_toan_tien":
            khoang_canh_an_toan_tien = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        if data_admin[i][0] == "khoang_cach_tim_duoi_di":
            khoang_cach_tim_duoi_di = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        if data_admin[i][0] == "khoang_cach_an_toan_re":
            khoang_cach_an_toan_re = int(float(data_admin[i][1]))
        if data_admin[i][0] == "alpha_star_scan_trai":
            alpha_star_scan_trai = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 
        if data_admin[i][0] == "alpha_star_scan_phai":
            alpha_star_scan_phai = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 
        if data_admin[i][0] == "khoang_cach_astar_scan":
            khoang_cach_astar_scan = int(float(data_admin[i][1]))
        if data_admin[i][0] == "distan_scan_all":
            distan_scan_all = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        if data_admin[i][0] == "dis_min":
            dis_min = int(float(data_admin[i][1]))
        if data_admin[i][0] == "dis_max":
            dis_max = int(float(data_admin[i][1]))
        if data_admin[i][0] == "khoang_cach_dich_min":
            khoang_cach_dich_min = int(float(data_admin[i][1]))
        if data_admin[i][0] == "khoang_cach_dich_max":
            khoang_cach_dich_max = int(float(data_admin[i][1]))


on_music = 1
class detect_data_sent_driver:
    def __init__(self, load_data_esp = 1, driver_motor_check = 0):
        if on_music == 1:
            threading.Thread(target=music.sound_speak).start()
            music.name_music = "none"

        self.map_all = np.full((5000, 5000, 4), (150, 150, 150, 0), np.uint8)
        self.img1 = self.map_all.copy()

        self.khoang_canh_an_toan_tien = khoang_canh_an_toan_tien
        self.driver_motor_check = driver_motor_check
        if self.driver_motor_check == 1:
            self.driver_motor = connect_driver.sent_data_driver()
        else:
            self.driver_motor = ""
        
        self.scan_vat_can = scan_an_toan.kiem_tra_vat_can()
        self.load_data_driver_motor = {}
        self.convert_data_run_agv0 = {"run_diem_2": "NG", "run_huong": "NG", "run_tin_hieu": "NG", "run_tin_hieu_tam_thoi": "NG"}
        if load_data_esp == 1:
            try:
                pass
                threading.Thread(target=ket_noi_esp.python_esp32).start() ######`##################
            except OSError as e:
                print("khong ket noi duoc esp")
                pass
        self.input_esp = ket_noi_esp.input_esp
        self.connect_esp32 = ket_noi_esp.check_connect_esp
        ########################################################################################
        self.v_tien_max = 0                 # load_data_web - ok
        self.v_re_max = 0                   # load_data_web - ok
        self.point_start_LQR = []           # load_data_process - ok
        self.point_end_LQR = []             # xu_ly_tin_hieu - ok
        self.angle = 0                      # xu_ly_tin_hieu - ok
        self.distance = 0                   # xu_ly_tin_hieu - ok
        self.check_angle_distance = 0       # xu_ly_tin_hieu - ok
        self.stop_sent_driver = 0           # xu_ly_tin_hieu - ok
        self.di_cham = 0                    # load_data_process - ok
        self.a_v = 800
        self.dang_re = 0                    # xu_ly_tin_hieu - ok
        self.tien_rl = 0
        self.v_tien_max_new = 0                 # load_data_web - ok
        self.v_re_max_new = 0                   # load_data_web - ok
        
        self.toa_do_diem_dau = [] # dùng để tạo 
        self.toa_do_hien_tai = [] # dùng để tạo self.point_old
        self.toa_do_diem_dich = [] # dùng để tạo self.point_old, kiểm tra vị trí điểm 1 so với point_old
        # Format: danh_sach_diem = {"tên điểm": [tọa độ x, tọa độ y, "loại điểm", góc agv]}
        self.danh_sach_diem = {}
        # Format: danh_sach_duong = {"tên đường": ["tên điểm 1", "tên điểm 2"]}
        self.done_tin_hieu_tam_thoi = 0
        self.danh_sach_duong = {}
        self.robot_direction = []
        self.point_old = []

        self.stop_rmse = 0
        self.stop_vat_can = 0
        self.stop = 0
        self.name_music = "none"

        self.check_tien = 1
        self.check_trai = 0
        self.check_phai = 0

        self.run_auto_controller = 0

        self.convert_data_run_agv = self.convert_data_run_agv0.copy()

        self.distance_old = 1000
        self.dk_agv_thu_cong = 0
        self.data_dk_tay = ""
        self.check_an_toan = ""

        self.connect_driver = 0
        self.run_stop = 0
        self.toa_do_x_agv = 0
        self.toa_do_y_agv = 0
        self.distan_max = 100

        self.vt_trai = 100
        self.vt_phai = 0
        self.quay_phai = 0
        self.quay_trai = 0

        self.ten_diem_bat_dau = ""

        self.danh_sach_diem_moi = ["P1", "P2", "P3"]
        self.thay_doi_diem = 0

        
    def reset_data(self):
        pass
    def load_run_and_stop(self):
        self.run_auto_controller = webserver.run_and_stop
    def void_loop(self):
        self.load_run_and_stop() # load trạng thái run/stop từ web
        if self.data_dk_tay == "":
            self.load_data_process() # load các giá trị tọa độ góc của agv, check an toàn
            self.xu_ly_tin_hieu() # load các giá trị điểm đầu điểm đích, ... từ web, sau đó tính các giá trị góc, khoảng cách yêu cầu
        if self.driver_motor_check == 1 and self.driver_motor.connect == True:
            self.driver_motor.check_connect()
            if self.data_dk_tay != "":
                self.driver_motor.dk_agv_thu_cong = self.dk_agv_thu_cong
                self.dk_ban_phim(self.data_dk_tay, 3000)
            else:
                if self.run_auto_controller == 1:
                    self.data_sent_drive() # gửi các giá trị tính toán, khai báo qua driver motor
                else:
                    self.stop_data_sent_drive()

            
            
            

    def convert_tin_hieu(self, tin_hieu_agv, name_agv, danh_sach_diem, danh_sach_duong):
        """
        Kiểm tra và tách chuỗi tín hiệu AGV.

        Args:
            tin_hieu: json tín hiệu đầu vào.
                                Ví dụ:  {'name_agv': 'agv1', 'dich_den': "P1", 'trang_thai': 'run'}
            danh_sach_diem (dict): Dictionary chứa thông tin các điểm.
                                Ví dụ: {"P1": [100, 200, "không hướng", 0], ...}
            danh_sach_duong (dict): Dictionary chứa thông tin các đường đi.
                                    Ví dụ: {"P1_P2": ["P1", "P2"], ...}

        Returns:
            dict: Dictionary chứa thông tin đã được xử lý.
        """
        # data_all = {
        #     "tín hiệu hợp lệ": "True", # Mặc định là True, sẽ đổi thành False nếu cần
        #     "điểm hiện tại": self.diem_hien_tai,
        #     "điểm đích": "None",
        #     "góc điểm đích": ["không hướng", 0],
        #     "tiến max": "None",
        #     "rẽ max": "None",
        #     "tín hiệu input": [],
        #     "tín hiệu tạm thời": []
        # }
        print("tin_hieu", tin_hieu_agv)
        tin_hieu = tin_hieu_agv[name_agv]
        data_all = {
            "tín hiệu hợp lệ": "True", # Mặc định là True, sẽ đổi thành False nếu cần
            "tên điểm bắt đầu": "",
            "tọa độ điểm bắt đầu": [],
            "tên điểm đích": "None",
            "tọa độ điểm đích": [],
            "góc điểm đích": ["không hướng", 0],
            "tiến max": "None",
            "rẽ max": "None",
            "tín hiệu input": [],
            "tín hiệu tạm thời": []
        }
        # {'name_agv': 'agv1', 'dich_den': 'P2', 'trang_thai': 'run'}
        # data_all["tín hiệu hợp lệ"] = ""
        diem_dich = tin_hieu["dich_den"]
        diem_dau, khoang_cach = self.tim_diem_gan_nhat([self.toa_do_x_agv, self.toa_do_y_agv], danh_sach_diem)
        # print("khoang_cach", diem_dau, khoang_cach)
        if khoang_cach <= 25:
            if self.ten_diem_bat_dau == "":
                self.ten_diem_bat_dau = diem_dau
        # else:
        #     self.ten_diem_bat_dau = ""
        # print(tin_hieu)

        # diem_dich = tin_hieu.get('dich_den', "None")
        tien_max = tin_hieu.get('van_toc_tien_max', "None")
        # print("tien_maxx", tien_max)
        if tien_max != "None":
            data_all["tiến max"] = tien_max
            # print("tien_maxx", self.v_tien_max)
        re_max = tin_hieu.get('van_toc_re_max', "None")
        # print("re_max", re_max)
        if re_max == "None":
            data_all["rẽ max"] = re_max
            # print("re_max", self.v_re_max)
        tin_hieu_input = tin_hieu.get('tin_hieu_input', [])
        tin_hieu_tam_thoi = tin_hieu.get('tin_hieu_tam_thoi', [])

        # Kiểm tra sự tồn tại của điểm bắt đầu 
        if self.ten_diem_bat_dau in danh_sach_diem:
            data_all["tên điểm bắt đầu"] = self.ten_diem_bat_dau
            data_all["tọa độ điểm bắt đầu"] = [danh_sach_diem[self.ten_diem_bat_dau][0], danh_sach_diem[self.ten_diem_bat_dau][1]]
        else:
            data_all["tín hiệu hợp lệ"] = "Điểm bắt đầu không tồn tại trong danh sách điểm."
            # print("Điểm bắt đầu không tồn tại trong danh sách điểm.")
        # Kiểm tra sự tồn tại của điểm đích
        if diem_dich in danh_sach_diem:
            data_all["tên điểm đích"] = diem_dich
            data_all["tọa độ điểm đích"] = [danh_sach_diem[diem_dich][0], danh_sach_diem[diem_dich][1]]
            # Lấy thông tin góc của điểm đích
            data_all["góc điểm đích"] = [danh_sach_diem[diem_dich][2], danh_sach_diem[diem_dich][3]]
        else:
            data_all["tín hiệu hợp lệ"] = "Điểm đích không tồn tại trong danh sách điểm."
            # print("Điểm đích không tồn tại trong danh sách điểm.")

        # Kiểm tra đường đi giữa điểm 1 và điểm 2
        # print(danh_sach_diem)
        # print(danh_sach_duong)
        # Chỉ thực hiện kiểm tra đường đi nếu cả hai điểm đều hợp lệ
        if data_all["tên điểm bắt đầu"] != "None" and data_all["tên điểm đích"] != "None":
            duong_truc_tiep = str(data_all["tên điểm đích"]) + "_" + str(data_all["tên điểm bắt đầu"])
            duong_nguoc_lai = str(data_all["tên điểm bắt đầu"]) + "_" + str(data_all["tên điểm đích"])
            if (duong_truc_tiep not in danh_sach_duong) and (duong_nguoc_lai not in danh_sach_duong):
                print("llllllllllll", duong_truc_tiep, duong_nguoc_lai, danh_sach_duong)
                data_all["tín hiệu hợp lệ"] = "Không có đường đi trực tiếp giữa điểm bắt đầu và điểm đích."
                # print("Không có đường đi trực tiếp giữa điểm bắt đầu và điểm đích.")
        else: # Nếu điểm không hợp lệ, đường đi chắc chắn không hợp lệ
            data_all["tín hiệu hợp lệ"] = "Điểm bắt đầu hoặc điểm đích không hợp lệ."
            # print("Điểm bắt đầu hoặc điểm đích không hợp lệ.")

        

        return data_all

    def kiem_tra_tin_hieu_esp32(self, data):
        self.input_esp = ket_noi_esp.input_esp
        self.connect_esp32 = ket_noi_esp.check_connect_esp

        """
        Kiểm tra xem tất cả các điều kiện trong check_data có khớp với input_esp không.

        Args:
            check_data (list): Danh sách các điều kiện cần kiểm tra.
                            Mỗi điều kiện là một list con dạng [key, value_mong_muon].
                            Ví dụ: [['IN4', 0], ['IN3', 1]]
            input_esp (dict): Dictionary chứa trạng thái của các input.
                            Ví dụ: {"IN1":0, "IN2":0, ..., "IN12":0}

        Returns:
            bool: True nếu tất cả các điều kiện trong check_data khớp với input_esp,
                ngược lại là False.
        """
        if not data:
            # Nếu không có điều kiện nào để kiểm tra, mặc định là True.
            # Bạn có thể thay đổi hành vi này nếu cần (ví dụ: trả về False).
            return True
        if self.connect_esp32 == False:
            # Nếu không kết nối được với esp32, coi như không khớp.
            return False

        for dieu_kien in data:
            # Đảm bảo mỗi điều kiện là một cặp [key, value]
            if not isinstance(dieu_kien, list) or len(dieu_kien) != 2:
                # Điều kiện không hợp lệ, coi như không khớp.
                # Hoặc bạn có thể raise một Exception ở đây nếu muốn.
                return False
            
            key_can_kiem_tra = dieu_kien[0]
            gia_tri_mong_muon = dieu_kien[1]

            # 1. Kiểm tra xem key có tồn tại trong input_esp không
            if key_can_kiem_tra not in self.input_esp:
                return False  # Key không tồn tại, điều kiện không khớp

            # 2. Kiểm tra xem giá trị có khớp không
            # Giả định rằng kiểu dữ liệu của giá trị mong muốn và giá trị trong input_esp là tương thích để so sánh.
            if self.input_esp[key_can_kiem_tra] != gia_tri_mong_muon:
                return False  # Giá trị không khớp

        # Nếu tất cả các điều kiện trong check_data đều được kiểm tra và khớp
        return True
    def load_data_web(self):
        # {
        #             "agv1": {"vi_tri_hien_tai": "P1", "dich_den": "P1", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
        #             "agv2": {"vi_tri_hien_tai": "P2", "dich_den": "P2", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
        #             "agv3": {"vi_tri_hien_tai": "P3", "dich_den": "P3", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
        #             "agv4": {"vi_tri_hien_tai": "P4", "dich_den": "P4", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
        #             "agv5": {"vi_tri_hien_tai": "P5", "dich_den": "P5", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
        #             "agv6": {"vi_tri_hien_tai": "P6", "dich_den": "P6", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
        #             "agv7": {"vi_tri_hien_tai": "P7", "dich_den": "P7", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []}
        #         } # test
        # print(webserver.tin_hieu_nhan)
        tin_hieu_nhan = webserver.tin_hieu_nhan # {'name_agv': 'agv1', 'dich_den': "P2", 'trang_thai': 'run'}
        name_agv = webserver.name_agv
        dict_cai_dat = webserver.dict_cai_dat
        self.danh_sach_diem = webserver.danh_sach_diem
        self.danh_sach_duong = webserver.danh_sach_duong
        self.run_stop = webserver.run_and_stop
        self.toa_do_x_agv = webserver.dict_dieu_chinh_vi_tri_agv["toa_do_x"]
        self.toa_do_y_agv = webserver.dict_dieu_chinh_vi_tri_agv["toa_do_y"]
        self.v_tien_max = dict_cai_dat["van_toc_tien_max"]
        self.v_re_max = dict_cai_dat["van_toc_re_max"]

        return tin_hieu_nhan, name_agv
    
    def xu_ly_tin_hieu(self):

        # {"run_diem_2": "NG", "run_huong": "NG", "run_tin_hieu": "NG", "run_tin_hieu_tam_thoi": "NG"}
        
        # data_all = {
        #     "tín hiệu hợp lệ": "True", # Mặc định là True, sẽ đổi thành False nếu cần
        #     "điểm 1": "None",
        #     "điểm 2": "None",
        #     "góc điểm 2": ["không hướng", 0],
        #     "tiến max": "None",
        #     "rẽ max": "None",
        #     "tín hiệu": [],
        #     "tín hiệu tạm thời": []
        # }

        tin_hieu_nhan, name_agv = self.load_data_web()
        if tin_hieu_nhan != {} and self.run_stop == 1:
            data_tin_hieu_nhan =  self.convert_tin_hieu(tin_hieu_nhan, name_agv, self.danh_sach_diem, self.danh_sach_duong)
            # data_tin_hieu_nhan = {'tín hiệu hợp lệ': 'True', 
            #                     'tên điểm bắt đầu': 'P1', 
            #                     'tọa độ điểm bắt đầu': [972, 892], 
            #                     'tên điểm đích': 'P2', 
            #                     'tọa độ điểm đích': [1010, 972], 
            #                     'góc điểm đích': ['không hướng', 0.0], 
            #                     'tiến max': 2000, 'rẽ max': 500, 
            #                     'tín hiệu input': [], 
            #                     'tín hiệu tạm thời': []}

            tien_max = data_tin_hieu_nhan["tiến max"]
            re_max = data_tin_hieu_nhan["rẽ max"]
            ten_diem_dau = data_tin_hieu_nhan["tên điểm bắt đầu"]
            toa_do_diem_dau = data_tin_hieu_nhan["tọa độ điểm bắt đầu"]
            ten_diem_dich = data_tin_hieu_nhan["tên điểm đích"]
            toa_do_diem_dich = data_tin_hieu_nhan["tọa độ điểm đích"]
            goc_dich = data_tin_hieu_nhan["góc điểm đích"]
            tin_hieu_hop_le = data_tin_hieu_nhan["tín hiệu hợp lệ"]
            tin_hieu = data_tin_hieu_nhan["tín hiệu input"]
            tin_hieu_tam_thoi = data_tin_hieu_nhan["tín hiệu tạm thời"]
            # print("data_tin_hieu_nhan", data_tin_hieu_nhan)
            # print("--------------------------------------------------------------------------")
            print("tin_hieu_hop_leor self.stop_rmse == 1 or self.stop_vat_can == 1", tin_hieu_hop_le, self.stop_rmse, self.stop_vat_can == 1)
            stop = 0
            if tin_hieu_hop_le != "True" or self.stop_rmse == 1 or self.stop_vat_can == 1:
                stop = 1
                self.stop_sent_driver = 1
                print("stop do tin hieu khong hop le hoac rmse hoac vat can")

            if self.convert_data_run_agv["run_diem_2"] == "OK" and self.convert_data_run_agv["run_huong"] == "OK": ##################### test tạm thời
                stop = 1
                if ten_diem_dau == "P1" and ten_diem_dich == "P2":
                    webserver.tin_hieu_nhan[name_agv] = {"vi_tri_hien_tai": "P2", 
                                                            "dich_den": "P3", 
                                                            "trang_thai": "run", 
                                                            "message": "None", 
                                                            "danh_sach_duong_di": []}
                if ten_diem_dau == "P2" and ten_diem_dich == "P3":
                    webserver.tin_hieu_nhan[name_agv] = {"vi_tri_hien_tai": "P3", 
                                                            "dich_den": "P2", 
                                                            "trang_thai": "run", 
                                                            "message": "None", 
                                                            "danh_sach_duong_di": []}

                if ten_diem_dau == "P3" and ten_diem_dich == "P2":
                    webserver.tin_hieu_nhan[name_agv] = {"vi_tri_hien_tai": "P2", 
                                                            "dich_den": "P1", 
                                                            "trang_thai": "run", 
                                                            "message": "None", 
                                                            "danh_sach_duong_di": []}
                if ten_diem_dau == "P2" and ten_diem_dich == "P1":
                    webserver.tin_hieu_nhan[name_agv] = {"vi_tri_hien_tai": "P1", 
                                                            "dich_den": "P2", 
                                                            "trang_thai": "run", 
                                                            "message": "None", 
                                                            "danh_sach_duong_di": []}
                self.convert_data_run_agv = self.convert_data_run_agv0.copy()
                self.ten_diem_bat_dau = ""
                self.distance_old = 1000


            self.stop = stop
            if self.stop == 1 or self.stop_sent_driver == 1:
                if self.name_music != "none":
                    if self.name_music == "re_trai":
                        self.name_music = "none"
                    if self.name_music == "re_phai":
                        self.name_music = "none"
            # print("stop", self.stop)
            if self.stop == 0:
                
                self.stop_xu_ly_tin_hieu = 1
                if tien_max != "None":
                    self.v_tien_max_new = int(float(tien_max))
                else:
                    self.v_tien_max_new = self.v_tien_max
                    # v_tien_max
                if re_max != "None":
                    self.v_re_max_new = int(float(re_max))
                else:
                    self.v_re_max_new = self.v_re_max

                if goc_dich[0] != "có hướng":
                    self.convert_data_run_agv["run_huong"] = "OK"
                
                # kiem tra tin hieu
                if len(tin_hieu) != 0:
                    if self.kiem_tra_tin_hieu_esp32(tin_hieu) == True:
                        tin_hieu = []
                if len(tin_hieu_tam_thoi) != 0:
                    if self.kiem_tra_tin_hieu_esp32(tin_hieu_tam_thoi) == True:
                        self.done_tin_hieu_tam_thoi = 1

                if len(tin_hieu) == 0:
                    self.convert_data_run_agv["run_tin_hieu"] = "OK"
                else:
                    self.convert_data_run_agv["run_tin_hieu"] = "NG"
                if self.done_tin_hieu_tam_thoi == 1:
                    self.convert_data_run_agv["run_tin_hieu_tam_thoi"] = "OK"

                self.toa_do_diem_dau = toa_do_diem_dau
                self.toa_do_diem_dich = toa_do_diem_dich

                # xử lý tín hiệu web truyền tới
                if len(self.toa_do_diem_dau) != 0 and len(self.toa_do_diem_dich) != 0:
                    point_end_LQR = self.toa_do_diem_dich
                    goc_quay_check = 0
                    distance = 0
                    check_angle_distance = "distance"
                    if self.convert_data_run_agv["run_diem_2"] != "OK":

                        # load điểm gần agv để đi tới
                        check, distance, angle_deg = tim_duong_di.calculate_distance_and_angle(self.toa_do_hien_tai, self.toa_do_diem_dich, self.robot_direction)
                        print("1111111111111",distance,angle_deg, self.toa_do_hien_tai, self.toa_do_diem_dich, self.robot_direction, ten_diem_dau, ten_diem_dich)
                        if distance > 80:
                            img_Astar, max_point_Astar, x_min_Astar, y_min_Astar, x_max_Astar, y_max_Astar = crop_img_Atar.img_crop(self.img1.copy(), 
                                                                                                self.toa_do_diem_dau.copy(), self.toa_do_diem_dich.copy(), distance = 80)
                            point_end_LQR = [max_point_Astar[0],max_point_Astar[1]]

                        # xác định khoảng cách và góc điểm đích 
                        # check, distance, angle_deg = tim_duong_di.calculate_distance_and_angle(self.diem_dau, self.diem_dich, self.robot_direction)
                        check_kc, distance_kc, angle_deg_kc = tim_duong_di.calculate_distance_and_angle(self.toa_do_diem_dau, self.toa_do_hien_tai, self.robot_direction)
                        check_line, distance_line, angle_deg_line = tim_duong_di.calculate_distance_and_angle(self.toa_do_diem_dich, self.toa_do_hien_tai, self.toa_do_diem_dau)
                        # goc bám sát đường đi
                        if angle_deg_line > 90:
                            angle_deg_line = 180 - angle_deg_line
                        if angle_deg_line < -90:
                            angle_deg_line = -180 - angle_deg_line
                        # goc quay yêu cầu
                        print("222222222222222", angle_deg, angle_deg_line)
                        goc_quay_check = angle_deg + angle_deg_line

                        # thay đổi quét vật cản khi vật đi chậm và gần đích
                        
                        # vt_check_kc = max(self.driver_motor.vt_phai * 10, self.driver_motor.vt_trai * 10)
                        vt_check_kc = 0  ######################################################################### test tạm thời
                        number_kc = 0
                        if vt_check_kc <= 2000 and (distance < 50 or distance_kc < 50):
                        # if vt_check_kc <= 2000 and (distance < 50):
                            a = int(vt_check_kc / 100)
                            number_kc = 20 - a
                            if number_kc > 15:
                                number_kc = 15
                        self.khoang_canh_an_toan_tien = [khoang_canh_an_toan_tien[0], khoang_canh_an_toan_tien[1] - number_kc]

                        # agv rẽ trái hoặc phải
                        if abs(angle_deg) > 20:
                            self.dang_re = 1
                        if abs(angle_deg) < 12 and self.dang_re == 1:
                            self.dang_re = 0
                        if on_music == 1:
                            # music
                            if angle_deg < -20:
                                self.name_music = "re_phai"
                            else:
                                if self.name_music == "re_phai":
                                    self.name_music = "none"

                            if angle_deg > 20:
                                self.name_music = "re_trai"
                            else:
                                if self.name_music == "re_trai":
                                    self.name_music = "none"

                        # lựa chọn khoảng cách đến đích
                        khoang_cach_dich = khoang_cach_dich_min
                        if self.convert_data_run_agv["run_huong"] == "OK":
                            khoang_cach_dich = khoang_cach_dich_max
                        # kiểm tra điều kiện đã đến đích
                        if (distance <= 2 or distance <= khoang_cach_dich  or (distance > self.distance_old and self.distance_old <= khoang_cach_dich)):
                            self.convert_data_run_agv["run_diem_2"] = "OK"
                        if distance < self.distance_old:
                            self.distance_old = distance
                    else:
                        if self.convert_data_run_agv["run_huong"] != "OK":
                            check_angle_distance = "angle"
                            start_point = self.point_start_LQR
                            robot_direction = self.robot_direction
                            A = np.array(toa_do_diem_dich)
                            C = np.array(start_point)
                            B = np.array(robot_direction)
                            # Tính vector AC
                            AC = C - A
                            # Tịnh tiến điểm B theo vector AC
                            end_point_angle = B + AC
                            check_ang, distance_ang, angle_deg = tim_duong_di.calculate_distance_and_angle(start_point, end_point_angle, robot_direction)

                            goc_quay_check = angle_deg
                            self.dang_re = 1

                            # music
                            if on_music == 1:
                                if angle_deg < 0:
                                    self.name_music = "re_phai"
                                else:
                                    if self.name_music == "re_phai":
                                        self.name_music = "none"

                                if angle_deg > 0:
                                    self.name_music = "re_trai"
                                else:
                                    if self.name_music == "re_trai":
                                        self.name_music = "none"
                            


                                    

                    self.point_end_LQR = point_end_LQR
                    self.check_angle_distance = check_angle_distance
                    self.angle = goc_quay_check
                    self.distance = distance
                    print("self.point_start_LQR, self.point_end_LQR, self.angle, self.distance, self.check_angle_distance", self.point_start_LQR, self.point_end_LQR, self.angle, self.distance, self.check_angle_distance)
                else:
                    self.stop_sent_driver = 1
                    print("khong co diem dau hoac diem dich")

    def kiem_tra_dich_den(self):
        pass
    def load_data_process(self):
        sent_data_driver_motor = self.load_data_driver_motor
        # điểm 1 lần đầu (nếu là rỗng) scan
        self.toa_do_hien_tai = [sent_data_driver_motor["tam_x_agv"], sent_data_driver_motor["tam_y_agv"]]
        # self.point_old = self.toa_do_diem_dau

        self.point_start_LQR = [sent_data_driver_motor["tam_x_agv"], sent_data_driver_motor["tam_y_agv"]]
        self.goc_agv = sent_data_driver_motor["rotation"]
        self.robot_direction = [sent_data_driver_motor["huong_x"], sent_data_driver_motor["huong_y"]]
        self.img1 = sent_data_driver_motor["img1"]
        self.stop_rmse = sent_data_driver_motor["stop"]

        # # load_du_lieu scan vat can
        # self.check_an_toan = self.load_scan_vat_can(sent_data_driver_motor["scan"], 
        #                                             self.check_tien, self.check_trai, self.check_phai,
        #                                             sent_data_driver_motor["huong_agv"], 
        #                                             sent_data_driver_motor["tam_x_agv"], sent_data_driver_motor["tam_y_agv"], 
        #                                             sent_data_driver_motor["huong_x"], sent_data_driver_motor["huong_y"],
        #                                             sent_data_driver_motor["scaling_factor"], 
        #                                             sent_data_driver_motor["window_size_x_all"], sent_data_driver_motor["window_size_y_all"])
        if self.check_an_toan == "stop":
            self.stop_vat_can = 1
        else:
            self.stop_vat_can = 0
        if self.check_an_toan == "slow":
            self.di_cham = 1
        else:
            self.di_cham = 0
    def load_scan_vat_can(self, scan, check_tien, check_trai, check_phai, 
                          rotation, x_goc, y_goc, huong_x, huong_y, 
                          scaling_factor, window_size_x_all, window_size_y_all):
        output = self.scan_vat_can.detect(scan, check_tien, check_trai, check_phai, 
                                 rotation, x_goc, y_goc, huong_x, huong_y, 
                                 scaling_factor, window_size_x_all, window_size_y_all,
                                 self.khoang_canh_an_toan_tien, khoang_cach_an_toan_re, khoang_cach_tim_duoi_di)
        return output
    
    
    
    def stop_data_sent_drive(self):
        self.driver_motor.load_data_sent_drive(self.v_tien_max_new, self.v_re_max_new, self.point_start_LQR, self.point_end_LQR,
                             self.goc_agv, self.angle, self.distance, self.check_angle_distance,
                             1, self.di_cham, self.a_v, self.dang_re, self.tien_rl)
    def data_sent_drive(self):
        self.driver_motor.load_data_sent_drive(self.v_tien_max_new, self.v_re_max_new, self.point_start_LQR, self.point_end_LQR,
                             self.goc_agv, self.angle, self.distance, self.check_angle_distance,
                             self.stop, self.di_cham, self.a_v, self.dang_re, self.tien_rl)
        # return v_tien_max,  v_re_max, point_start_LQR, point_end_LQR, \
        #     goc_agv, goc_dich, angle, distance, check_angle_distance, \
        #         stop = 0, check_an_toan = [], tim_duong = 0, a_v = 800, dang_re = 0, tien_rl = 200
    
    def dk_ban_phim(self,data_input, van_toc_max_tien):
        if data_input != "":
            if data_input == "stop":
                self.driver_motor.sent_data_controller(vt_trai = 0, vt_phai = 0)
            if data_input == "tien":
                self.driver_motor.sent_data_controller(vt_trai = van_toc_max_tien, vt_phai = van_toc_max_tien)
            if data_input == "trai":
                self.driver_motor.sent_data_controller(vt_trai = -int(van_toc_max_tien/4), vt_phai = int(van_toc_max_tien/4))
            if data_input == "phai":
                self.driver_motor.sent_data_controller(vt_trai = int(van_toc_max_tien/4), vt_phai = -int(van_toc_max_tien/4))
            if data_input == "dich_tien_trai":
                self.driver_motor.sent_data_controller(vt_trai = int((van_toc_max_tien/4)*3), vt_phai = van_toc_max_tien)
            if data_input == "dich_tien_phai":
                self.driver_motor.sent_data_controller(vt_trai = van_toc_max_tien, vt_phai = int((van_toc_max_tien/4)*3))

            if data_input == "lui":
                self.driver_motor.sent_data_controller(vt_trai = -van_toc_max_tien, vt_phai = -van_toc_max_tien)
            if data_input == "dich_lui_trai":
                self.driver_motor.sent_data_controller(vt_trai = -int((van_toc_max_tien/4)*3), vt_phai = -van_toc_max_tien)
            if data_input == "dich_lui_phai":
                self.driver_motor.sent_data_controller(vt_trai = -int(van_toc_max_tien), vt_phai = -int((van_toc_max_tien/4)*3))
    
    def tim_diem_gan_nhat(self, current_position, danh_sach_diem):
        """
        Tìm điểm gần nhất trong danh_sach_diem so với một tọa độ cho trước.

        Args:
            current_position (list or tuple): Tọa độ hiện tại [x, y].
            danh_sach_diem (dict): Dictionary chứa thông tin các điểm.
                                    Ví dụ: {"P1": [100, 200, "không hướng", 0], ...}

        Returns:
            tuple: Một tuple chứa (tên điểm gần nhất, khoảng cách nhỏ nhất).
                Trả về (None, float('inf')) nếu danh_sach_diem rỗng.
        """
        if not danh_sach_diem:
            return None, float('inf')

        nearest_point_name = None
        min_distance = float('inf')
        
        current_x, current_y = current_position

        for point_name, point_data in danh_sach_diem.items():
            point_x, point_y = point_data[0], point_data[1]
            
            # Tính khoảng cách Euclidean
            distance = math.sqrt((point_x - current_x)**2 + (point_y - current_y)**2)
            
            if distance < min_distance:
                min_distance = distance
                nearest_point_name = point_name
                
        return nearest_point_name, min_distance

# if __name__ == '__main__':
    # Ví dụ sử dụng hàm find_nearest_point
    # danh_sach_diem_test = {
    #     "P1": [100, 200, "không hướng", 0],
    #     "P2": [550, 480, "không hướng", 0],
    #     "P3": [120, 210, "có hướng", 90]
    # }
    # agv_position = [125, 215]

    # ten_diem_gan_nhat, khoang_cach = find_nearest_point(agv_position, danh_sach_diem_test)

    # print(f"Tọa độ AGV: {agv_position}")
    # print(f"Điểm gần nhất là: {ten_diem_gan_nhat}")
    # print(f"Khoảng cách tới điểm đó là: {khoang_cach}")
    
