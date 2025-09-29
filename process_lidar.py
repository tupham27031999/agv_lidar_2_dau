import numpy as np
import cv2, threading, time, math, shutil, os
from support_main.lib_main import edit_csv_tab, remove
from support_main import angle_and_distance, gicp_lidar, tim_duong_di, music
import path
# import crop_img_Atar
# import detect_pallet
# import dich_diem
import webserver
from skimage.draw import line
import open3d as o3d
import detect_gicp






path_phan_mem = path.path_phan_mem

# Directories for saving data and maps
SAVED_DATA_DIR = path_phan_mem + "/data_input_output"
PATH_MAPS_DIR = SAVED_DATA_DIR + "/maps"

distan_scan_all = [0, 10000]
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
x_goc0 = "none"
y_goc0 = "none"
rotation0 = "none"
kc_them_map = 170
for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "distan_scan_all":
            distan_scan_all = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        if data_admin[i][0] == "x_goc0":
            if data_admin[i][1] != "none":
                x_goc0 = int(float(data_admin[i][1]))
        if data_admin[i][0] == "y_goc0":
            if data_admin[i][1] != "none":
                y_goc0 = int(float(data_admin[i][1]))
        if data_admin[i][0] == "rotation0":
            if data_admin[i][1] != "none":
                rotation0 = float(data_admin[i][1])
        if data_admin[i][0] == "kc_them_map":
            kc_them_map = int(float(data_admin[i][1]))
        if data_admin[i][0] == "anpha_scan":
            anpha_scan = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        
print("distan_scan_all", distan_scan_all, x_goc0, y_goc0, rotation0, anpha_scan)

class process_data_lidar:
    def __init__(self, window_size, window_size_all, scaling_factor, rmse1, rmse2, x_goc0 = None, y_goc0 = None, rotation0 = None):
        print("img0.shape", window_size)
        self.window_size_x = window_size
        self.window_size_y = window_size

        self.window_size_x_img2 = 400
        self.window_size_y_img2 = 400

        self.window_size_x_all = window_size_all
        self.window_size_y_all = window_size_all
        self.scaling_factor = scaling_factor
        self.scan_xy = np.array([[0, 0, 0]])

        self.map_all = np.full((self.window_size_y_all, self.window_size_x_all, 3), 128, dtype=np.uint8)
        self.mask_map_all = np.full((self.window_size_y_all, self.window_size_x_all), 0.5, dtype=np.float32)  # bắt đầu ở 0.5
        self.global_map = o3d.geometry.PointCloud() # danh sách đám mây điểm gốc
        self.map_all_2d = np.zeros((self.window_size_y_all, self.window_size_x_all), dtype=np.float32)
        self.img0 = np.ones((self.window_size_y, self.window_size_x, 3), np.uint8) *128
        self.img1 = self.map_all.copy()
        self.img2 = self.map_all.copy()[int(self.window_size_x_all//2 - self.window_size_y_img2):int(self.window_size_x_all//2 + self.window_size_y_img2),
                                     int(self.window_size_x_all//2 - self.window_size_x_img2):int(self.window_size_x_all//2 + self.window_size_x_img2)]

        self.rmse = 0
        self.transformation_matrix = np.eye(4)
        self.rmse1 = rmse1
        self.rmse2 = rmse2

        if x_goc0 == None:
            self.x_goc = int(self.window_size_x_all / 2)
        else:
            self.x_goc = x_goc0
        if y_goc0 == None:
            self.y_goc = int(self.window_size_y_all / 2)
        else:
            self.y_goc = y_goc0
        if rotation0 == None:
            self.rotation = 0
        else:
            self.rotation = rotation0 * np.pi / 180
        self.x_goc_old = self.x_goc
        self.y_goc_old = self.y_goc
        self.tam_x_agv, self.tam_y_agv = [int(self.window_size_x_all / 2), int(self.window_size_y_all / 2)]
        self.huong_x = 0
        self.huong_y = 0

        # self.window_size_img2 = 600
        # self.number_img2 = int(self.window_size_img2/2)
        # self.number_img3 = 20

        self.arr_goc0 = np.array([])

        self.time_start = 0
        self.update_vi_tri_agv_ban_dau = 1

        # nếu khoảng cách hay góc giữa 2 lần quét liền kề quá lơn sẽ dừng agv
        self.delta_distan = 0
        self.delta_angle = 0

        # bằng 1 thì cập nhật vào map (thực hiện quét map)
        self.add_all_point = 0
        # thêm vào map nếu = 1 ( bỏ qua điều khiện < rmse)
        self.add_map = 0

        self.color_check = [0,0,0]
        self.color = [255,255,255]

        self.sent_data_driver_motor = {"stop": 0, "scan": self.scan_xy, "img1": self.img1, "tam_x_agv": self.tam_x_agv, "tam_y_agv": self.tam_y_agv, 
                                       "huong_agv": self.rotation, "huong_x": self.huong_x, "huong_y": self.huong_y, 
                                       "scaling_factor": self.scaling_factor, "window_size_x_all": self.window_size_x_all, "window_size_y_all": self.window_size_y_all}
        self.vi_tri_agv_new = 0

        self.bien_dem = 0
        self.trung_binh = 0



        self.trans_init = np.eye(4)
        self.detect_gicp_lidar = detect_gicp.Config()
    def main_loop(self, scan_xy):
        self.process_data_lidar(scan_xy)
        self.load_data_web()
        # self.update_vi_tri_agv()
    def load_data_web(self):
        if webserver.dict_chon_ban_do["update"] == 1:
            webserver.dict_chon_ban_do["update"] = 0
            map_name = webserver.dict_chon_ban_do["ten_ban_do"]
            map_name, extension = os.path.splitext(map_name)
            map_path = os.path.join(PATH_MAPS_DIR, map_name)

            load_ok = 0
            
            if os.path.exists(map_path + ".npy"):
                loaded_map_all = np.load(map_path + ".npy")
                if loaded_map_all is not None and loaded_map_all.dtype != np.uint8:
                    loaded_map_all = loaded_map_all.astype(np.uint8)
                load_ok = load_ok + 1
            if os.path.exists(map_path + ".npz"):    
                loaded_mask_map_all = np.load(map_path + ".npz")['mask_map']
                if loaded_mask_map_all is not None and loaded_mask_map_all.dtype != np.float32:
                    loaded_mask_map_all = loaded_mask_map_all.astype(np.float32)
                load_ok = load_ok + 1
            if os.path.exists(map_path + ".pcd"):  
                global_map = o3d.io.read_point_cloud(map_path + ".pcd")
                load_ok = load_ok + 1
            if load_ok == 3:
                self.map_all = loaded_map_all
                self.mask_map_all = loaded_mask_map_all
                self.global_map = global_map
                print("----------------------- load_map")
    
    def process_data_lidar(self, scan_xy):
        if distan_scan_all[0] == 1:
            x_coords = scan_xy[:, 0]
            y_coords = scan_xy[:, 1]
            distances = np.sqrt(x_coords**2 + y_coords**2)
            scan_all = scan_xy[distances < distan_scan_all[1]]
            # print("scan_all.shape", scan_all.shape)
        else:
            scan_all = scan_xy
        self.scan_xy = scan_all
        self.sent_data_driver_motor["scan"] = scan_xy

        self.img1 = self.map_all.copy()
        h_img1,w_img1,_ = self.img1.shape

        px = self.scan_xy[:,0]
        py = self.scan_xy[:,1]
        arr_test = np.vstack((px, py, np.zeros_like(px))).T  # Thêm chiều z = 0 để tạo PointCloud 3D
        if self.time_start == 0:
            self.time_start = time.time()
        
        if (webserver.dict_dieu_chinh_vi_tri_agv["setup"] == 0) and time.time() - self.time_start > 0:
            if arr_test.shape[0] > 10:
                stop_agv = 0
                tt = time.time()
                self.map_all, self.mask_map_all, self.global_map, self.rmse, new_arr_ok, r, t = self.detect_gicp_lidar.detect(self.map_all.copy(), self.mask_map_all.copy(), self.global_map, arr_test, self.rmse1, self.rmse2, self.scaling_factor, update=self.add_all_point)
                # print("-----------",time.time() - tt, self.rmse)
                # Sử dụng arctan2 để tính góc quay từ ma trận xoay r
                # Đây là phương pháp chính xác và ổn định nhất, tránh các vấn đề của arccos
                # Dấu trừ (-) có thể cần thiết tùy thuộc vào quy ước hệ tọa độ của bạn
                self.rotation = -np.arctan2(r[1, 0], r[0, 0])
                self.x_goc = int(t[0] * self.scaling_factor + self.window_size_x_all / 2)
                self.y_goc = int(-t[1] * self.scaling_factor + self.window_size_y_all / 2)
                # print(self.x_goc, self.y_goc)

                
                new_arr_ok[:, 0] = new_arr_ok[:, 0] * self.scaling_factor + self.window_size_x_all//2 
                new_arr_ok[:, 1] = - new_arr_ok[:, 1] * self.scaling_factor + self.window_size_y_all//2 
                for ii in range(0, max(len(new_arr_ok), new_arr_ok.shape[0])):
                    # hiển thị kết quả của icp lên ảnh
                    if ii < len(new_arr_ok[:, 0]) - 1 and int(new_arr_ok[ii, 0]) < w_img1 and int(new_arr_ok[ii, 1]) < h_img1:
                        cv2.circle(self.img1, (int(new_arr_ok[ii, 0]), int(new_arr_ok[ii, 1])), 1, (255, 0, 255), -1)

                self.add_map = 0
            else:
                stop_agv = 1
                new_arr_ok = arr_test
            
            # self.rotation = self.normalize_angle_rad(self.rotation)

            self.tam_x_agv, self.tam_y_agv = self.translate_point(self.x_goc, self.y_goc, - self.rotation, distance=0)

        else:
            
            new_arr_ok = arr_test
            
            if (webserver.dict_dieu_chinh_vi_tri_agv["setup"] == 1):
                # print(webserver.dict_dieu_chinh_vi_tri_agv)
                self.x_goc = int(float(webserver.dict_dieu_chinh_vi_tri_agv["toa_do_x"]))
                self.y_goc = int(float(webserver.dict_dieu_chinh_vi_tri_agv["toa_do_y"]))
                self.rotation = float(webserver.dict_dieu_chinh_vi_tri_agv["goc_agv"]) * np.pi / 180

                delta_x = (self.x_goc - self.window_size_x_all//2) / self.scaling_factor
                # Sửa lại công thức tính delta_y để bù cho trục y ngược của ảnh
                delta_y = (self.window_size_y_all//2 - self.y_goc) / self.scaling_factor

                # Tạo ma trận biến đổi 4x4 (pose) mới và cập nhật cho GICP
                cos_rot = math.cos(self.rotation)
                sin_rot = math.sin(self.rotation)
                self.detect_gicp_lidar.global_pose = np.array([
                    [cos_rot, -sin_rot, 0, delta_x],
                    [sin_rot,  cos_rot, 0, delta_y],
                    [0,              0, 1,    0   ],
                    [0,              0, 0,    1   ]])
                # global_pose = np.array([
                #     [cos_rot, -sin_rot, 0, delta_x],
                #     [sin_rot,  cos_rot, 0, delta_y],
                #     [0,              0, 1,    0   ],
                #     [0,              0, 0,    1   ]])
                # print("Đã cập nhật Global Pose mới cho GICP.")
                

                # self.detect_gicp_lidar.global_pose = self.tao_ma_tran_xoay_tinh_tien(delta_x/self.scaling_factor, delta_y/self.scaling_factor, -self.scaling_factor) 

                self.map_all, self.mask_map_all, self.global_map, self.rmse, new_arr_ok, r, t = self.detect_gicp_lidar.detect(self.map_all.copy(), self.mask_map_all.copy(), self.global_map, arr_test, self.rmse1, self.rmse2, self.scaling_factor, update=0, show_map_new=1)
                new_arr_ok[:, 0] = new_arr_ok[:, 0] * self.scaling_factor + self.window_size_x_all//2
                new_arr_ok[:, 1] = - new_arr_ok[:, 1] * self.scaling_factor + self.window_size_y_all//2
                
                
                # current_points_global = detect_gicp.transform_points(arr_test, global_pose[:3, :3], global_pose[:3, 3])

                # new_arr_ok[:, 0] = arr_test[:, 0] * self.scaling_factor + self.window_size_x_all//2
                # new_arr_ok[:, 1] = - arr_test[:, 1] * self.scaling_factor + self.window_size_y_all//2

                # print("new_arr_ok", new_arr_ok)
                for ii in range(0, max(len(new_arr_ok), new_arr_ok.shape[0])):
                    # hiển thị kết quả của icp lên ảnh
                    if ii < len(new_arr_ok[:, 0]) - 1 and int(new_arr_ok[ii, 0]) < w_img1 and int(new_arr_ok[ii, 1]) < h_img1:
                        cv2.circle(self.img1, (int(new_arr_ok[ii, 0]), int(new_arr_ok[ii, 1])), 1, (255, 0, 255), -1)
            # print(new_arr_ok)
            self.tam_x_agv, self.tam_y_agv = self.translate_point(self.x_goc, self.y_goc, - self.rotation, distance=0)

            stop_agv = 1
        
        h_img1, w_img1, _ = self.img1.shape
        cv2.circle(self.img1, (int(self.tam_x_agv), int(self.tam_y_agv)), 5, (255, 0, 0), -1)
        self.huong_x = int(self.tam_x_agv + 20 * math.cos(self.rotation - np.pi/2)) # thuan am, nghich dương
        self.huong_y = int(self.tam_y_agv + 20 * math.sin(self.rotation - np.pi/2))
        # print("@@@@@@@@@@@@@@@@@", (-self.rotation - np.pi/2)*180/np.pi)

        # print("self.rotation", self.rotation * 180 / np.pi, self.tam_x_agv, self.tam_y_agv)
        self.huong_x2 = int(self.tam_x_agv + 20 * math.cos(self.rotation - np.pi/2))
        self.huong_y2 = int(self.tam_y_agv + 20 * math.sin(self.rotation - np.pi/2))

        cv2.arrowedLine(self.img1, (int(self.tam_x_agv), int(self.tam_y_agv)), (self.huong_x2, self.huong_y2), (255, 0, 0), 1, tipLength=0.2)

        x1 = max(0, self.x_goc - self.window_size_x_img2)
        y1 = max(0, self.y_goc - self.window_size_y_img2)
        x2 = min(self.x_goc + self.window_size_x_img2, w_img1)
        y2 = min(self.y_goc + self.window_size_y_img2, h_img1)

        self.img2 = self.img1.copy()[y1:y2,x1:x2,:]


        webserver.points_color_blue = new_arr_ok[:,:2]
        if webserver.dict_dieu_chinh_vi_tri_agv["update"] == 0 and webserver.dict_dieu_chinh_vi_tri_agv["setup"] == 0:
            webserver.dict_dieu_chinh_vi_tri_agv["toa_do_x"] = self.tam_x_agv
            webserver.dict_dieu_chinh_vi_tri_agv["toa_do_y"] = self.tam_y_agv
            webserver.dict_dieu_chinh_vi_tri_agv["goc_agv"] = (self.rotation * 180 / np.pi) % 360
        else:
            webserver.run_and_stop = 0
        self.sent_data_driver_motor["tam_x_agv"] = self.tam_x_agv
        self.sent_data_driver_motor["tam_y_agv"] = self.tam_y_agv
        self.sent_data_driver_motor["rotation"] = self.rotation
        self.sent_data_driver_motor["stop"] = stop_agv
        self.sent_data_driver_motor["huong_x"] = self.huong_x
        self.sent_data_driver_motor["huong_y"] = self.huong_y

    def tao_ma_tran_xoay_tinh_tien(self, deltax, deltay, goc_quay_do):
        """
        Tạo ma trận biến đổi 4x4 cho phép xoay 2D và tịnh tiến 2D.

        Ma trận này thực hiện phép xoay quanh trục Z và sau đó tịnh tiến
        trên mặt phẳng XY. Đây là phép biến đổi affine phổ biến trong không gian 2D.

        Args:
            deltax (float): Độ dịch chuyển theo trục x.
            deltay (float): Độ dịch chuyển theo trục y.
            goc_quay_do (float): Góc quay quanh trục Z (tính bằng độ).
                                Giá trị dương là quay ngược chiều kim đồng hồ.

        Returns:
            np.ndarray: Ma trận biến đổi 4x4.
        """
        # 1. Chuyển đổi góc từ độ sang radian vì các hàm sin/cos trong Python dùng radian
        goc_quay_rad = math.radians(goc_quay_do)
        cos_theta = math.cos(goc_quay_rad)
        sin_theta = math.sin(goc_quay_rad)

        # 2. Tạo ma trận biến đổi 4x4
        #    Cấu trúc ma trận:
        #    | cosθ  -sinθ   0   deltax |
        #    | sinθ   cosθ   0   deltay |
        #    |  0      0     1      0   |
        #    |  0      0     0      1   |
        transformation_matrix = np.array([
            [cos_theta, -sin_theta, 0, deltax],
            [sin_theta,  cos_theta, 0, deltay],
            [0,                0, 1,      0],
            [0,                0, 0,      1]
        ])

        return transformation_matrix
    def xoay_va_tinh_tien_diem(self, px, py, deltax, deltay, goc_quay_rad,  scaling_factor=1):
        """
        Xoay và tịnh tiến một tập hợp các điểm 2D.

        Hàm này thực hiện hai phép biến đổi theo thứ tự:
        1. Xoay các điểm quanh gốc tọa độ (0,0).
        2. Tịnh tiến các điểm đã xoay.

        Args:
            px (np.ndarray): Mảng NumPy chứa các tọa độ x của các điểm.
            py (np.ndarray): Mảng NumPy chứa các tọa độ y của các điểm.
            deltax (float): Độ dịch chuyển theo trục x.
            deltay (float): Độ dịch chuyển theo trục y.
            goc_quay_do (float): Góc quay (tính bằng độ). Dương là ngược chiều kim đồng hồ.

        Returns:
            tuple[np.ndarray, np.ndarray]: Một tuple chứa hai mảng NumPy:
                                        - px_moi: Mảng tọa độ x mới.
                                        - py_moi: Mảng tọa độ y mới.
        """
        cos_theta = math.cos(goc_quay_rad)
        sin_theta = math.sin(goc_quay_rad)

        # 2. Tạo ma trận xoay 2D cho phép quay ngược chiều kim đồng hồ
        # R = [[cos, -sin],
        #      [sin,  cos]]
        rotation_matrix = np.array([
            [cos_theta, -sin_theta],
            [sin_theta,  cos_theta]
        ])

        # 3. Kết hợp px và py thành một mảng các điểm (N, 2)
        # Mỗi hàng là một điểm [x, y]
        points = np.vstack((px*scaling_factor, py*scaling_factor)).T

        # 4. Áp dụng phép xoay
        # Vì các điểm là vector hàng, ta nhân với ma trận chuyển vị của R
        # P_new = P_old * R^T
        rotated_points = np.dot(points, rotation_matrix.T)

        # 5. Áp dụng phép tịnh tiến
        translation_vector = np.array([deltax, deltay])
        transformed_points = rotated_points + translation_vector

        # 6. Tách mảng kết quả thành px_moi và py_moi
        px_moi = transformed_points[:, 0] + self.window_size_x_all // 2
        py_moi = transformed_points[:, 1] + self.window_size_y_all // 2

        return px_moi, py_moi
    def translate_point(self, x, y, alpha, distance=5):
        """
        Dịch chuyển điểm (x, y) theo góc alpha và khoảng cách distance.
        
        Args:
            x (float): Tọa độ x của điểm đầu vào.
            y (float): Tọa độ y của điểm đầu vào.
            alpha (float): Góc alpha (đơn vị: radians).
            distance (float): Khoảng cách dịch chuyển (mặc định: 5 pixel).
        
        Returns:
            tuple: Tọa độ điểm mới (x_new, y_new).
        """
        x_new = x + distance * math.cos(alpha)
        y_new = y + distance * math.sin(alpha)
        return x_new, y_new
    def normalize_angle_rad(self, angle):
        """
        Chuyển đổi góc (radian) về khoảng [-pi, pi] radian.
        Ví dụ: 7 -> ~0.716 rad, -4 -> ~2.283 rad
        """
        angle = angle % (2 * np.pi)
        if angle > np.pi:
            angle -= 2 * np.pi
        if angle < -np.pi:
            angle += 2 * np.pi
        return angle    
    
    def convert_gird_data_web(self, dict_data_grid, tam_x_agv, tam_y_agv):
        list_data_key = []
        list_data_name = []
        list_data_grid = []
        list_data_diem = []
        # for key, value in dict_data_grid:
        # {
        #     "grid_00": {"name": "0.0", "vi_tri": [2400, 2400, 2500, 2500], "diem": [2430, 2430], "mau": "yellow", "loai_diem": "duong_di"},
        #     "grid_01": {"name": "0.1", "vi_tri": [2500, 2500, 2600, 2600], "diem": [2530, 2530], "mau": "yellow", "loai_diem": "duong_di"},
        #     "grid_02": {"name": "0.2", "vi_tri": [2600, 2600, 2700, 2700], "diem": [2630, 2630], "mau": "yellow", "loai_diem": "duong_di"}
        # }
        pass

    

