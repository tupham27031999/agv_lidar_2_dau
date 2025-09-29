from flask import Flask, Response, request, jsonify
import cv2
import os
# import signal
# import sys

import numpy as np
import time
import threading
import random
import webbrowser
from flask import send_from_directory
import json
import io # Để làm việc với image bytes in memory
import scan_an_toan
import math
import path
from support_main.lib_main import edit_csv_tab

path_phan_mem = path.path_phan_mem
path_admin = path_phan_mem + "/setting/admin_window.csv"

tien_max = 2000
re_max = 500

data_admin = edit_csv_tab.load_all_stt(path_admin)
for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "window_size": # kích thước ảnh check map
            window_size = int(float(data_admin[i][1]))
        if data_admin[i][0] == "window_size_all": # kich thuoc ảnh tổng
            window_size_all = int(float(data_admin[i][1]))
        if data_admin[i][0] == "tien_max": 
            tien_max = int(float(data_admin[i][1]))
        if data_admin[i][0] == "re_max": 
            re_max = int(float(data_admin[i][1]))
        if data_admin[i][0] == "x_goc0": 
            x_goc0 = int(float(data_admin[i][1]))
        if data_admin[i][0] == "y_goc0": 
            y_goc0 = int(float(data_admin[i][1]))

# --- Configuration ---
AGV_TITLE = "AGV 1"
IMG_WIDTH = window_size_all  # Default canvas width if no map is loaded
IMG_HEIGHT = window_size_all # Default canvas height if no map is loaded

# Directories for saving data and maps
SAVED_DATA_DIR = path_phan_mem + "/data_input_output"
PATH_MAPS_DIR = SAVED_DATA_DIR + "/maps"
PATH_POINTS_DIR = SAVED_DATA_DIR + "/point_lists"
PATH_PATHS_DIR = SAVED_DATA_DIR + "/path_lists"
PATH_GRID_LISTS_DIR = SAVED_DATA_DIR + "/grid_lists"
PATH_LOG_GIAO_TIEP_DIR = SAVED_DATA_DIR + "/log_giao_tiep"

# Create directories if they don't exist
os.makedirs(PATH_MAPS_DIR, exist_ok=True)
os.makedirs(PATH_POINTS_DIR, exist_ok=True)
os.makedirs(PATH_PATHS_DIR, exist_ok=True)
os.makedirs(PATH_GRID_LISTS_DIR, exist_ok=True)
os.makedirs(PATH_LOG_GIAO_TIEP_DIR, exist_ok=True)


# Global variable for the current image being served
current_image0 = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
current_image = current_image0.copy()
image_lock = threading.Lock()
image_initialized = False

# Biến để lưu trữ gốc tọa độ của img2 (current_image) trên map_all (ảnh gốc lớn từ process_lidar)
x_goc_img2_origin_on_map_all = 0
y_goc_img2_origin_on_map_all = 0

# --- Data Structures (as per new request) ---
# Default settings
dict_cai_dat = {"van_toc_tien_max": tien_max, "van_toc_re_max": re_max}
# Map selection
dict_chon_ban_do = {"ten_ban_do": "", "update": 0}
# AGV position adjustment
dict_dieu_chinh_vi_tri_agv = {"toa_do_x": 2500, "toa_do_y": 2500, "goc_agv": 0, "setup": 0, "update": 0}

# Server-side storage for "live" points and paths
file_diem_da_chon = ""
file_duong_da_chon = ""
file_grid_da_chon = ""
# Format: danh_sach_diem = {"tên điểm": [tọa độ x, tọa độ y, "loại điểm", góc agv]}
danh_sach_diem = {}
# Format: danh_sach_duong = {"tên đường": ["tên điểm 1", "tên điểm 2"]}
danh_sach_duong = {}
update_ds_diem_duong = 1

# Lists for UI dropdowns
list_tien_max = [7000, 6500, 6000, 5500, 5000, 4500, 4000, 3500, 3000, 2500, 2000, 1500, 1000, 500]
list_re_max = [1000, 900, 800, 700, 600, 500, 400, 300]

# detect_scan_an_toan = scan_an_toan.kiem_tra_vat_can()
# points_color_blue = np.array([[50,500,300],[500,100,200]])
# points_color_red = [[500,50],[600,250],[600,20],[500,250]]

points_color_blue = np.array([])
points_color_red = np.array([])

# --- NEW: Grid Data and Paint Flag ---
dict_data_grid = {}
# {
#     "grid_00": {"name": "0.0", "vi_tri": [2400, 2400, 2500, 2500], "diem": [2430, 2430], "mau": "yellow", "loai_diem": "duong_di"},
#     "grid_01": {"name": "0.1", "vi_tri": [2500, 2500, 2600, 2600], "diem": [2530, 2530], "mau": "yellow", "loai_diem": "duong_di"},
#     "grid_02": {"name": "0.2", "vi_tri": [2600, 2600, 2700, 2700], "diem": [2630, 2630], "mau": "yellow", "loai_diem": "duong_di"}
# }
paint_dict_data_grid = True # New flag to control drawing grids

# --- Signal Communication Variables ---
# {'agv1': {'dich_den': 'P1', 'trang_thai': 'run'}, 
#  'agv2': {'dich_den': 'P2', 'trang_thai': 'run'}, 
#  'agv3': {'dich_den': 'P3', 'trang_thai': 'run'}, 
#  'agv4': {'dich_den': 'P4', 'trang_thai': 'run'}, 
#  'agv5': {'dich_den': 'P5', 'trang_thai': 'run'}, 
#  'agv6': {'dich_den': 'P6', 'trang_thai': 'run'}, 
#  'agv7': {'dich_den': 'P7', 'trang_thai': 'run'}}
name_agv = "agv1"
tin_hieu_nhan = {
                    "agv1": {"vi_tri_hien_tai": "P1", "dich_den": "P2", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
                    "agv2": {"vi_tri_hien_tai": "P2", "dich_den": "P2", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
                    "agv3": {"vi_tri_hien_tai": "P3", "dich_den": "P3", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
                    "agv4": {"vi_tri_hien_tai": "P4", "dich_den": "P4", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
                    "agv5": {"vi_tri_hien_tai": "P5", "dich_den": "P5", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
                    "agv6": {"vi_tri_hien_tai": "P6", "dich_den": "P6", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
                    "agv7": {"vi_tri_hien_tai": "P7", "dich_den": "P7", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []}
                } # test

danh_sach_duong_di = []
vi_tri_hien_tai = "P1"
dich_den = "P1"
trang_thai = "run"
message_agv = "none"

tin_hieu_gui = {name_agv: {"vi_tri_hien_tai": danh_sach_duong_di, 
                           'dich_den': dich_den, 
                           'trang_thai': trang_thai, 
                           "message": message_agv,
                           "danh_sach_duong_di": danh_sach_duong_di}}


thoi_gian_nhan_str = "N/A"

run_and_stop = 0 # "run" or "stop"
open_loa = 0


# points_color_red 

def log_communication(log_type, timestamp_str, signal_value):
    """
    Ghi log giao tiếp vào file.
    log_type: "nhan" hoặc "gui"
    timestamp_str: Thời gian dạng string (YYYY-MM-DD HH:MM:SS)
    signal_value: Giá trị tín hiệu (có thể là string hoặc dict/list)
    """
    try:
        now = time.localtime()
        date_hour_str = time.strftime("%Y-%m-%d_%H", now)
        filename = f"log_{date_hour_str}.txt"
        filepath = os.path.join(PATH_LOG_GIAO_TIEP_DIR, filename)

        # Chuyển đổi signal_value thành chuỗi JSON nếu nó là dict hoặc list
        if isinstance(signal_value, (dict, list)):
            log_content = json.dumps(signal_value, ensure_ascii=False)
        else:
            log_content = str(signal_value)

        with open(filepath, "a", encoding="utf-8") as f:
            f.write(f"{timestamp_str}\t{log_type}\t{log_content}\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")

def get_available_maps():
    """Scans the map directory for image files and .npy files."""
    maps = []
    if os.path.exists(PATH_MAPS_DIR):
        for f_name in os.listdir(PATH_MAPS_DIR):
            if f_name.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif', '.npy')):
                maps.append(f_name)
    return sorted(maps)

def get_saved_lists(directory):
    """Scans a directory for .json files."""
    saved_files = []
    if os.path.exists(directory):
        for f_name in os.listdir(directory):
            if f_name.lower().endswith('.json'):
                saved_files.append(f_name.replace('.json', '')) # Return name without .json
    return sorted(saved_files)

list_ban_do = get_available_maps()
# These will be populated by scanning directories later or can be initialized if needed
list_danh_sach_diem_da_luu = get_saved_lists(PATH_POINTS_DIR)
list_danh_sach_duong_da_luu = get_saved_lists(PATH_PATHS_DIR)
list_danh_sach_grid_da_luu = get_saved_lists(PATH_GRID_LISTS_DIR)


def initial_image_setup_task():
    global current_image0, image_initialized, image_lock, current_image
    if image_initialized:
        return

    print("Initializing current_image...")
    default_map_loaded = False
    # Try to load the first available map as default if dict_chon_ban_do is empty
    if not dict_chon_ban_do.get("ten_ban_do") and list_ban_do:
        dict_chon_ban_do["ten_ban_do"] = list_ban_do[0]

    if dict_chon_ban_do.get("ten_ban_do"):
        map_name = dict_chon_ban_do["ten_ban_do"]
        map_path = os.path.join(PATH_MAPS_DIR, map_name)
        if os.path.exists(map_path):
            try:
                loaded_img = None
                if map_name.lower().endswith(".npy"):
                    img_array = np.load(map_path)
                    if img_array.ndim == 2: # Grayscale
                        loaded_img = cv2.cvtColor(img_array, cv2.COLOR_GRAY2BGR)
                    elif img_array.ndim == 3:
                        loaded_img = img_array
                    else:
                        print(f"Unsupported .npy array dimension: {img_array.ndim} for {map_name}")

                    if loaded_img is not None and loaded_img.dtype != np.uint8:
                         # Basic scaling for float arrays, assuming range 0-1 or 0-255
                        if loaded_img.max() <= 1.0 and loaded_img.min() >= 0.0:
                            loaded_img = (loaded_img * 255).astype(np.uint8)
                        else:
                            loaded_img = cv2.normalize(loaded_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

                else: # Standard image
                    loaded_img = cv2.imread(map_path)

                if loaded_img is not None:
                    with image_lock:
                        current_image0 = loaded_img
                        dict_chon_ban_do["update"] = 1
                    default_map_loaded = True
                    print(f"Loaded default map: {map_name}")
                else:
                    print(f"Error loading or processing map image: {map_path}")
            except Exception as e:
                print(f"Exception loading map {map_path}: {e}")

    if not default_map_loaded:
        print("No default map or failed to load. Using black canvas.")
        with image_lock:
            current_image0 = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
    update_img()
    image_initialized = True
    print(f"Initial current_image set ({current_image.shape}).")


# def release_resources_and_exit(sig, frame):
#     print("Ctrl+C pressed. Exiting...")
#     sys.exit(0)

# signal.signal(signal.SIGINT, release_resources_and_exit)
app = Flask(__name__)

@app.route('/')
def main_web():
    global list_ban_do, list_danh_sach_diem_da_luu, list_danh_sach_duong_da_luu, list_danh_sach_grid_da_luu
    list_ban_do = get_available_maps()
    list_danh_sach_diem_da_luu = get_saved_lists(PATH_POINTS_DIR)
    list_danh_sach_duong_da_luu = get_saved_lists(PATH_PATHS_DIR)
    list_danh_sach_grid_da_luu = get_saved_lists(PATH_GRID_LISTS_DIR)

    # Ensure default selections are valid if lists are populated
    if not dict_chon_ban_do["ten_ban_do"] and list_ban_do:
        dict_chon_ban_do["ten_ban_do"] = list_ban_do[0]
    
    # For saved lists, we don't pre-select one unless specified elsewhere
    # The client-side JS will handle the initial selection for these based on the list.

    html_content = f"""
    <!DOCTYPE html>
    <html lang="vi">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>{AGV_TITLE} - Control Panel</title>
        <style>
            body {{ font-family: Arial, sans-serif; margin: 0; padding: 0; background-color: #f0f0f0; display: flex; flex-direction: column; height: 100vh; }}
            .header {{ background-color: #333; color: white; padding: 10px 20px; text-align: center; }}
            .header h3 {{ margin: 0; }}
            .main-container {{ display: flex; flex-direction: row; flex-grow: 1; padding: 10px; box-sizing: border-box; overflow: hidden; }}
            #openseadragon-viewer {{ flex-grow: 1; border: 1px solid black; background-color: #333; margin-right: 10px; }}
            .sidebar {{ width: 380px; display: flex; flex-direction: column; padding: 10px; border: 1px solid #ccc; background-color: #ffffff; box-shadow: 2px 0 5px rgba(0,0,0,0.1); overflow-y: auto; }}
            .function-frame {{ border: 1px solid #e0e0e0; border-radius: 5px; padding: 10px; margin-bottom: 15px; background-color: #f9f9f9; }}
            .function-frame h4 {{ margin-top: 0; margin-bottom: 10px; color: #333; border-bottom: 1px solid #ddd; padding-bottom: 5px; }}
            .sidebar label {{ display: block; margin-bottom: 5px; font-weight: bold; }}
            .sidebar select, .sidebar input[type="text"], .sidebar input[type="number"] {{ width: calc(100% - 22px); padding: 8px; margin-bottom: 10px; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }}
            .sidebar button {{ display: block; width: 100%; padding: 10px; margin-top: 5px; margin-bottom: 10px; background-color: #007bff; color: white; border: none; border-radius: 4px; cursor: pointer; }}
            .sidebar button:hover {{ background-color: #0056b3; }}
            .sidebar button.active {{ background-color: #0056b3; border: 2px solid #003d80; font-weight: bold;}}
            .button-row {{ display: flex; justify-content: space-between; gap: 10px; }}
            .button-row button {{ width: 100%; }} /* Let flex handle distribution */
            /* Modal styles */
            .sidebar button.btn-run-state {{
                background-color: #e74c3c; /* Red */
                color: white;
                border-color: #2980b9;
            }}
            .sidebar button.btn-run-state:hover {{
                background-color: #217dbb;
            }}
            .sidebar button.btn-stop-state {{
                
                background-color: #3498db; /* Blue */
                color: white;
                border-color: #c0392b;
            }}
            .sidebar button.btn-stop-state:hover {{
                background-color: #d64535;
            }}
            body {{ font-family: Arial, sans-serif; margin: 0; padding: 0; background-color: #f0f0f0; display: flex; flex-direction: column; height: 100vh; }}
            .header {{ background-color: #333; color: white; padding: 10px 20px; text-align: center; }}
            .header h3 {{ margin: 0; }}
            .main-container {{ display: flex; flex-direction: row; flex-grow: 1; padding: 10px; box-sizing: border-box; overflow: hidden; }}
            .modal {{ display: none; position: fixed; z-index: 1000; left: 0; top: 0; width: 100%; height: 100%; overflow: auto; background-color: rgba(0,0,0,0.4); }}
            .modal-content {{ background-color: #fefefe; margin: 10% auto; padding: 20px; border: 1px solid #888; width: 80%; max-width: 450px; border-radius: 5px; }}
            .modal-content label {{ margin-top:10px; }}
            .modal-content input, .modal-content select {{ width: calc(100% - 16px); }}
            .modal-buttons {{ margin-top: 15px; display: flex; justify-content: flex-end; gap: 10px; }}
            .modal-buttons button {{ width: auto; }}

        </style>
        
        <script src="static/js/openseadragon.min.js"></script>
    </head>
    <body>
        <div class="header"><h3>{AGV_TITLE}</h3></div>
        <div class="main-container">
            <div id="openseadragon-viewer"></div>
            <div class="sidebar">
                <!-- Trạng thái hoạt động Panel -->
                <div class="function-frame" id="frame-operation-status">
                    <h4>Trạng thái hoạt động</h4>
                    <button id="runStopBtn" onclick="toggleRunStop()">Run</button>
                </div>

                <!-- Loa Panel -->
                <div class="function-frame" id="frame-speaker-control">
                    <h4>Điều khiển Loa</h4>
                    <button id="speakerBtn" onclick="toggleSpeaker()">Loa</button>
                </div>

                <!-- Cài đặt Panel -->
                <div class="function-frame" id="frame-settings">
                    <h4>Cài đặt vận tốc</h4>
                    <label for="selectMaxSpeedForward">Vận tốc tiến max:</label>
                    <select id="selectMaxSpeedForward" onchange="updateSetting('van_toc_tien_max', this.value)">
                        { "".join([f'<option value="{val}" {"selected" if val == dict_cai_dat["van_toc_tien_max"] else ""}>{val}</option>' for val in list_tien_max]) }
                    </select>
                    <label for="selectMaxSpeedTurn">Vận tốc rẽ max:</label>
                    <select id="selectMaxSpeedTurn" onchange="updateSetting('van_toc_re_max', this.value)">
                        { "".join([f'<option value="{val}" {"selected" if val == dict_cai_dat["van_toc_re_max"] else ""}>{val}</option>' for val in list_re_max]) }
                    </select>
                </div>

                <!-- Chọn bản đồ Panel -->
                <div class="function-frame" id="frame-map-selection">
                    <h4>Chọn bản đồ</h4>
                    <label for="selectMapName">Tên bản đồ:</label>
                    <select id="selectMapName">
                        { "".join([f'<option value="{val}" {"selected" if val == dict_chon_ban_do["ten_ban_do"] else ""}>{val}</option>' for val in list_ban_do]) }
                    </select>
                    <button onclick="confirmMapSelection()">Cập nhật bản đồ</button>
                </div>

                <!-- Điều chỉnh AGV Panel -->
                <div class="function-frame" id="frame-agv-adjustment">
                    <h4>Điều chỉnh vị trí AGV</h4>
                    <label for="inputAgvX">Tọa độ X AGV (pixel):</label>
                    <input type="number" id="inputAgvX" value="{dict_dieu_chinh_vi_tri_agv['toa_do_x']}">
                    <label for="inputAgvY">Tọa độ Y AGV (pixel):</label>
                    <input type="number" id="inputAgvY" value="{dict_dieu_chinh_vi_tri_agv['toa_do_y']}">
                    <label for="inputAgvAngle">Góc AGV (độ):</label>
                    <input type="number" id="inputAgvAngle" value="{dict_dieu_chinh_vi_tri_agv['goc_agv']}">
                    <div class="button-row">
                        <button id="btnToggleAGVPositionMode" onclick="toggleMode('agvPos')">Đặt vị trí AGV</button>
                        <button onclick="confirmAGVUpdate()">Cập nhật vị trí</button>
                    </div>
                </div>

                <!-- Quản lý Điểm -->
                <div class="function-frame" id="frame-point-management">
                    <h4>Quản lý điểm</h4>
                    <div class="button-row">
                        <button id="btnAddPointMode" onclick="toggleMode('addPoint')">Thêm điểm</button>
                        <button id="btnEditPointMode" onclick="toggleMode('editPoint')">Sửa điểm</button>
                    </div>
                    <label for="inputPointListNameSave">Tên file lưu DS điểm:</label>
                    <input type="text" id="inputPointListNameSave" placeholder="VD: points_khu_A">
                    <button onclick="savePointList()">Lưu điểm</button>
                    <label for="selectSavedPointList">Tải DS điểm từ file:</label>
                    <select id="selectSavedPointList">
                        { "".join([f'<option value="{val}" {"selected" if val == file_diem_da_chon else ""}>{val}</option>' for val in list_danh_sach_diem_da_luu]) }
                    </select>
                    <button onclick="loadPointList()">Tải danh sách điểm</button>
                </div>

                <!-- Quản lý Đường Đi -->
                <div class="function-frame" id="frame-path-management">
                    <h4>Quản lý đường đi</h4>
                    <div class="button-row">
                        <button id="btnAddPathMode" onclick="toggleMode('addPath')">Thêm đường</button>
                        <button id="btnDeletePathMode" onclick="toggleMode('deletePath')">Xóa đường</button>
                    </div>
                    <label for="inputPathListNameSave">Tên file lưu DS đường:</label>
                    <input type="text" id="inputPathListNameSave" placeholder="VD: paths_khu_A">
                    <button onclick="savePathList()">Lưu đường đi</button>
                    <label for="selectSavedPathList">Tải DS đường từ file:</label>
                    <select id="selectSavedPathList">
                        { "".join([f'<option value="{val}" {"selected" if val == file_duong_da_chon else ""}>{val}</option>' for val in list_danh_sach_duong_da_luu]) }
                    </select>
                    <button onclick="loadPathList()">Tải danh sách đường</button>
                </div>
            </div>
        </div>

        <!-- Point Modal -->
        <div id="pointModal" class="modal">
            <div class="modal-content">
                <h4 id="pointModalTitle">Thêm Điểm Mới</h4>
                <input type="hidden" id="modalOriginalPointName"> <!-- For storing original name during edit/rename -->
                <label for="modalPointName">Tên Điểm:</label>
                <input type="text" id="modalPointName" required>
                <label for="modalPointX">Tọa độ X (pixel):</label>
                <input type="number" id="modalPointX">
                <label for="modalPointY">Tọa độ Y (pixel):</label>
                <input type="number" id="modalPointY">
                <label for="modalPointType">Loại Điểm:</label>
                <select id="modalPointType" onchange="toggleModalPointAngleInput()">
                    <option value="không hướng">Không hướng</option>
                    <option value="có hướng">Có hướng</option>
                </select>
                <div id="modalPointAngleContainer" style="display:none;">
                    <label for="modalPointAngle">Góc AGV (độ):</label>
                    <input type="number" id="modalPointAngle" value="0">
                </div>
                <div class="modal-buttons">
                    <button onclick="closePointModal()">Hủy</button>
                    <button id="deletePointModalButton" onclick="deletePointFromModal()" style="display:none; background-color: #dc3545;">Xóa Điểm</button>
                    <button onclick="submitPointModal()">Lưu Điểm</button>
                </div>
            </div>
        </div>

        <!-- Grid Modal -->
        <div id="gridModal" class="modal">
            <div class="modal-content">
                <h4 id="gridModalTitle">Thêm Grid Mới</h4>
                <input type="hidden" id="modalOriginalGridName">
                <label for="modalGridName">Tên Grid:</label>
                <input type="text" id="modalGridName" required>
                <label>Vị trí (x1, y1, x2, y2):</label>
                <div style="display: flex; gap: 5px;">
                    <input type="number" id="modalGridX1" placeholder="x1" style="width: 25%;">
                    <input type="number" id="modalGridY1" placeholder="y1" style="width: 25%;">
                    <input type="number" id="modalGridX2" placeholder="x2" style="width: 25%;">
                    <input type="number" id="modalGridY2" placeholder="y2" style="width: 25%;">
                </div>
                <label>Điểm (x, y) trong Grid:</label>
                <div style="display: flex; gap: 5px;">
                    <input type="number" id="modalGridDiemX" placeholder="Điểm X" style="width: 50%;">
                    <input type="number" id="modalGridDiemY" placeholder="Điểm Y" style="width: 50%;">
                </div>
                <label for="modalGridColor">Màu:</label>
                <select id="modalGridColor">
                    <option value="yellow">Yellow</option>
                    <option value="red">Red</option>
                    <option value="blue">Blue</option>
                    <option value="green">Green</option>
                    <option value="purple">Purple</option>
                    <option value="gray">Gray</option>
                </select>
                <label for="modalGridType">Loại:</label>
                <input type="text" id="modalGridType" placeholder="VD: duong_di">
                <div class="modal-buttons">
                    <button onclick="closeGridModal()">Hủy</button>
                    <button id="deleteGridModalButton" onclick="deleteGridFromModal()" style="display:none; background-color: #dc3545;">Xóa Grid</button>
                    <button onclick="submitGridModal()">Lưu Grid</button>
                </div>
            </div>
        </div>

        <script>
            let viewer;
            // Client-side data stores (using objects for easier lookup by name)
            let clientPointsData = {{}}; // {{ "P1": {{name: "P1", x: 10, y: 20, type: "có hướng", angle: 90, osdOverlay: obj, textOverlay: obj}}, ... }}
            let clientPathsData = {{}};  // {{ "P1_P2": {{name: "P1_P2", p1_name: "P1", p2_name: "P2", osdOverlay: obj, textOverlay: obj}}, ... }}
            let clientGridData = {{}}; // {{ "grid_00": {{name: "00", vi_tri: [x1,y1,x2,y2], diem: [x,y], ...}}, ...}}
            
            let nextPointNumericId = 0; // For generating default point names like P1, P2

            // UI Interaction Modes
            let currentMode = 'none'; // 'agvPos', 'addPoint', 'editPoint', 'addPath_start', 'addPath_end', 'deletePath'
            let tempPathStartPointName = null; // For 'addPath' mode, stores name of the first point clicked

            // Initial data from server (already embedded in HTML for selects)
            let clientDictCaiDat = JSON.parse('{json.dumps(dict_cai_dat)}');
            // dict_chon_ban_do and dict_dieu_chinh_vi_tri_agv are handled by direct value embedding or fetched

            let bluePointOverlays = []; // Store OSD overlay elements for blue points
            let redPointOverlays = [];  // Store OSD overlay elements for red points

            let osdAgvBodyPointOverlays = []; // Store OSD overlay elements for AGV body points
            let osdAgvArrowPointOverlays = [];// Store OSD overlay elements for AGV arrow points

            
            
            document.addEventListener('DOMContentLoaded', function () {{
                viewer = OpenSeadragon({{
                    id: "openseadragon-viewer",
                    prefixUrl: "static/images/",
                    tileSources: {{
                        type: 'image',
                        url:  '/full_image.jpg?' + new Date().getTime(),
                    }},
                    animationTime: 0.1, // Faster animation
                    blendTime: 0,       // No blend
                    constrainDuringPan: true,
                    visibilityRatio: 0.5,
                    zoomPerClick: 1.0, 
                    maxZoomPixelRatio: 5, // Cho phép zoom sâu hơn, ví dụ gấp 5 lần
                    zoomPerScroll: 1.2,
                    showNavigator: true,
                    navigatorPosition: "TOP_RIGHT",
                }});

                // --- Lưu trạng thái Viewer trước khi refresh/đóng trang ---
                window.addEventListener('beforeunload', function() {{
                    if (viewer) {{
                        const center = viewer.viewport.getCenter();
                        const zoom = viewer.viewport.getZoom();
                        const viewerState = {{
                            center: {{ x: center.x, y: center.y }},
                            zoom: zoom
                        }};
                        sessionStorage.setItem('osdViewerState', JSON.stringify(viewerState));
                        sessionStorage.setItem('clientPointsData', JSON.stringify(clientPointsData));
                        sessionStorage.setItem('clientPathsData', JSON.stringify(clientPathsData));
                        sessionStorage.setItem('clientGridData', JSON.stringify(clientGridData));
                        sessionStorage.setItem('nextPointNumericId', nextPointNumericId.toString());
                        // Lưu trạng thái của dropdowns
                        sessionStorage.setItem('selectedPointListFile', document.getElementById('selectSavedPointList').value);
                        sessionStorage.setItem('selectedPathListFile', document.getElementById('selectSavedPathList').value);
                        sessionStorage.setItem('selectedGridListFile', document.getElementById('selectSavedGridList').value);
                    }}
                }});

                // --- Khôi phục trạng thái Viewer sau khi tải ảnh ---
                viewer.addHandler('open', function() {{
                    const savedState = sessionStorage.getItem('osdViewerState');
                    // Initial fetch and polling for Run/Stop status
                    //  fetchRunStopStatus();
                    setInterval(fetchRunStopStatus, 2000); // Poll every 2 seconds

                    if (savedState) {{
                        const viewerState = JSON.parse(savedState);
                        console.log("Restoring viewer state:", viewerState);
                        viewer.viewport.panTo(new OpenSeadragon.Point(viewerState.center.x, viewerState.center.y), true); // true for immediately
                        viewer.viewport.zoomTo(viewerState.zoom, true); // true for immediately
                        sessionStorage.removeItem('osdViewerState'); // Xóa trạng thái đã lưu sau khi dùng
                    }}

                    // Start polling for AGV state and special points AFTER viewer is open
                    loadCurrentState();
                    if (!viewer || !viewer.isOpen() || viewer.world.getItemCount() === 0) {{
                        return;
                    }}
                    setInterval(fetchAGVState, 1000); // Lấy trạng thái AGV mỗi 200ms
                    
                    console.log("OSD viewer is open. Started AGV state polling.");

                    // LOGGING KÍCH THƯỚC ẢNH
                    if (viewer.world.getItemCount() > 0) {{
                        const imageItem = viewer.world.getItemAt(0);
                        if (imageItem && imageItem.source && imageItem.source.dimensions) {{
                            const imageDimensions = imageItem.source.dimensions;
                            console.log("[OSD 'open'] Image dimensions from OSD:", JSON.stringify(imageDimensions));
                        }} else {{ console.warn("[OSD 'open'] Image item or source or dimensions not available."); }}
                    }}
                }});
                
                viewer.addHandler('canvas-click', function(event) {{
                    if (!viewer.world.getItemAt(0) || !viewer.world.getItemAt(0).source) {{ // Thêm kiểm tra source
                        console.warn("Map image or its source not loaded yet in OSD.");
                        return;
                    }}
                    const viewportPoint = viewer.viewport.pointFromPixel(event.position);
                    const imagePoint = viewer.viewport.viewportToImageCoordinates(viewportPoint);
                    const x = Math.round(imagePoint.x);
                    const y = Math.round(imagePoint.y);

                    console.log(`Canvas click: X=${"{x}"}, Y=${"{y}"}, Mode: ${"{currentMode}"}`);

                    if (currentMode === 'agvPos') {{
                        document.getElementById('inputAgvX').value = x;
                        document.getElementById('inputAgvY').value = y;
                        
                        // Automatically send setup update on click when in 'agvPos' mode
                        const angleFromInput = parseFloat(document.getElementById('inputAgvAngle').value) || 0;
                        const setupData = {{
                            toa_do_x: x,
                            toa_do_y: y,
                            goc_agv: angleFromInput,
                            setup: 1 // Indicate this is a setup action
                            // 'update' flag is not set here, as this is a setup, not a final confirm
                        }};
                        fetch('/confirm_agv_update', {{
                            method: 'POST',
                            headers: {{ 'Content-Type': 'application/json' }},
                            body: JSON.stringify(setupData)
                        }})
                        .then(response => response.json())
                        .then(data => {{
                            console.log("AGV setup position on click response:", data);
                            if(data.status !== 'success') {{
                                alert("Lỗi gửi vị trí setup AGV khi click: " + data.message);
                            }}
                            // Optionally, provide feedback to user that setup was sent
                        }})
                        .catch(error => console.error('Error sending AGV setup position on click:', error));

                    }} else if (currentMode === 'addPoint') {{
                        openPointModal(null, x, y); // Pass null for pointData (adding new)
                    }} else if (currentMode === 'editPoint') {{
                        const nearestPoint = findNearestPoint(x, y);
                        if (nearestPoint) {{
                            openPointModal(nearestPoint, nearestPoint.x, nearestPoint.y, true);
                        }} else {{
                            alert("Không tìm thấy điểm nào gần vị trí click.");
                        }}
                    }} else if (currentMode === 'addPath_start') {{
                        const startPoint = findNearestPoint(x, y, true, 100); // Snap to point
                        if (startPoint) {{
                            tempPathStartPointName = startPoint.name;
                            currentMode = 'addPath_end';
                            // alert(`Đã chọn điểm bắt đầu: ${"{startPoint.name}"}. Chọn điểm kết thúc.`);
                            highlightPoint(startPoint.name, true);
                        }} else {{
                            alert('Vui lòng click gần một điểm đã tạo để bắt đầu đường đi.');
                        }}
                    }} else if (currentMode === 'addPath_end') {{
                        const endPoint = findNearestPoint(x, y, true, 100); // Snap to point
                        if (endPoint && tempPathStartPointName && endPoint.name !== tempPathStartPointName) {{
                            addPath(tempPathStartPointName, endPoint.name);
                            highlightPoint(tempPathStartPointName, false); // Unhighlight
                            tempPathStartPointName = null;
                            currentMode = 'addPath_start'; // Ready for next path's start point
                            // alert("Đã thêm đường. Sẵn sàng thêm đường mới hoặc đổi chế độ.");
                            // updateSpecialPoints
                        }} else if (endPoint && endPoint.name === tempPathStartPointName) {{
                            alert('Điểm kết thúc không được trùng với điểm bắt đầu.');
                        }} else {{
                            alert('Vui lòng click gần một điểm đã tạo để kết thúc đường đi.');
                        }}
                    }} else if (currentMode === 'deletePath') {{
                        const nearestPath = findNearestPath(x,y);
                        if(nearestPath) {{
                            if(confirm(`Bạn có chắc muốn xóa đường "${"{nearestPath.name}"}"?`)){{
                                deletePath(nearestPath.name);
                            }}
                        }} else {{
                            alert("Không tìm thấy đường nào gần vị trí click.");
                        }}
                    }}
                }});
                // Khôi phục lựa chọn cho dropdowns
                const savedSelectedPointList = sessionStorage.getItem('selectedPointListFile');
                const savedSelectedPathList = sessionStorage.getItem('selectedPathListFile');
                const savedSelectedGridList = sessionStorage.getItem('selectedGridListFile');

                // --- Khôi phục trạng thái client-side từ sessionStorage ---
                const savedPointsJSON = sessionStorage.getItem('clientPointsData');
                const savedPathsJSON = sessionStorage.getItem('clientPathsData');
                const savedGridJSON = sessionStorage.getItem('clientGridData');
                const savedNextId = sessionStorage.getItem('nextPointNumericId');

                if (savedPointsJSON) {{ // Nếu có dữ liệu điểm trong session, ưu tiên khôi phục từ đó
                    console.log("Attempting to restore client state from sessionStorage.");
                    try {{
                        clientPointsData = JSON.parse(savedPointsJSON);
                        Object.values(clientPointsData).forEach(p => {{
                            p.x = parseFloat(p.x); // Đảm bảo tọa độ là số
                            p.y = parseFloat(p.y); // Đảm bảo tọa độ là số
                            p.angle = parseFloat(p.angle); // Đảm bảo góc là số
                            // Overlays sẽ được tạo lại bởi drawPointOnOSD
                            p.osdOverlay = null; 
                            p.textOverlay = null;
                            // drawPointOnOSD(p);
                        }});
                        console.log("Restored clientPointsData from sessionStorage. Count:", Object.keys(clientPointsData).length);
                    }} catch (e) {{
                        console.error("Error parsing clientPointsData from sessionStorage:", e);
                        clientPointsData = {{}}; // Reset nếu có lỗi
                        loadCurrentState(); // Tải lại từ server nếu parse lỗi clientPathsData
                    }}

                    if (savedPathsJSON) {{
                        try {{
                            clientPathsData = JSON.parse(savedPathsJSON);
                            Object.values(clientPathsData).forEach(path => {{
                                path.osdOverlay = null;
                                path.textOverlay = null;
                                // if (clientPointsData[path.p1_name] && clientPointsData[path.p2_name]) {{
                                //     drawPathOnOSD(path);
                                // }}
                            }});
                            console.log("Restored clientPathsData from sessionStorage. Count:", Object.keys(clientPathsData).length);
                        }} catch (e) {{
                            console.error("Error parsing clientPathsData from sessionStorage:", e);
                            clientPathsData = {{}}; // Reset nếu có lỗi
                        }}
                    }}

                    if (savedNextId) {{
                        nextPointNumericId = parseInt(savedNextId, 10);
                        console.log("Restored nextPointNumericId from sessionStorage:", nextPointNumericId);
                    }}

                    if (savedGridJSON) {{
                        try {{
                            clientGridData = JSON.parse(savedGridJSON);
                            populateGridList();
                            console.log("Restored clientGridData from sessionStorage. Count:", Object.keys(clientGridData).length);
                        }} catch (e) {{
                            console.error("Error parsing clientGridData from sessionStorage:", e);
                            clientGridData = {{}};
                        }}
                    }}
                }} else {{
                    // Không có dữ liệu điểm trong session, tải mới từ server
                    console.log("No clientPointsData in sessionStorage. Loading current state from server.");
                    loadCurrentState(); 
                }}
                // Load initial lists for dropdowns
                updateSavedFileLists(savedSelectedPointList, savedSelectedPathList, savedSelectedGridList); 
                // Start polling for signal status
            }});
            
            function toggleRunStop() {{
                fetch('/api/toggle_run_stop', {{ method: 'POST' }})
                    .then(response => response.json())
                    .then(data => {{
                        if (data.status === 'success') {{
                            updateRunStopButton(data.currentState);
                        }}
                    }})
                    .catch(error => console.error('Error toggling run/stop state:', error));
            }}

            function updateNextPointId() {{
                let maxId = 0;
                // Lặp qua tất cả các điểm đã biết để tìm ID số lớn nhất
                for (const name in clientPointsData) {{
                    if (name.startsWith('P')) {{
                        // Lấy phần số từ tên điểm (ví dụ: 'P12' -> 12)
                        const num = parseInt(name.substring(1), 10);
                        // Kiểm tra xem có phải là số hợp lệ và có lớn hơn maxId hiện tại không
                        if (!isNaN(num) && num > maxId) {{
                            maxId = num;
                        }}
                    }}
                }}
                // ID tiếp theo sẽ lớn hơn ID lớn nhất đã tìm thấy 1 đơn vị
                nextPointNumericId = maxId + 1;
                console.log(`[updateNextPointId] ID điểm tiếp theo được cập nhật thành: ${{nextPointNumericId}}`);
            }}

            function updatePointsPath(newPoints, newPaths) {{
                //1. Xóa các overlay cũ khỏi viewer
                for (const name in clientPointsData) {{
                    removePointOverlay(name);
                }}
                for (const name in clientPathsData) {{
                    removePathOverlay(name);
                }}

                // 2. Reset lại dữ liệu phía client
                clientPointsData = {{}};
                clientPathsData = {{}};

                // 3. Nạp dữ liệu điểm mới và vẽ chúng
                for (const name in newPoints) {{
                    const p = newPoints[name];
                    clientPointsData[name] = {{
                        name: name,
                        x: p[0],
                        y: p[1],
                        type: p[2],
                        angle: p[3],
                        osdOverlay: null,
                        textOverlay: null
                    }};
                    drawPointOnOSD(clientPointsData[name]);
                    drawPointLabel(clientPointsData[name]);
                }}

                // 4. Nạp dữ liệu đường đi mới và vẽ chúng
                for (const name in newPaths) {{
                    const p = newPaths[name];
                    // Đảm bảo các điểm của đường đi này tồn tại trước khi vẽ
                    if (clientPointsData[p[0]] && clientPointsData[p[1]]) {{
                        clientPathsData[name] = {{
                            name: name,
                            p1_name: p[0],
                            p2_name: p[1],
                            osdOverlay: null,
                            textOverlay: null
                        }};
                        drawPathOnOSD(clientPathsData[name]);
                        drawPathLabel(clientPathsData[name]);
                    }}
                }}
            }}

            function updateRunStopButton(state) {{
                const btn = document.getElementById('runStopBtn');
                if (!btn) return;
                if (state === 'run') {{
                    btn.textContent = 'Stop';
                    btn.className = 'btn-run-state';
                }} else {{
                    btn.textContent = 'Run';
                    btn.className = 'btn-stop-state';
                }}
            }}

            function statusRunStopButton(state) {{
                const btn = document.getElementById('runStopBtn');
                if (!btn) return;
                if (state === '1') {{
                    btn.textContent = 'Stop';
                    btn.className = 'btn-run-state';
                }} else {{
                    btn.textContent = 'Run';
                    btn.className = 'btn-stop-state';
                }}
            }}

            function fetchRunStopStatus() {{
                const btn = document.getElementById('runStopBtn');
                fetch('/api/get_run_stop_status')
                    .then(response => response.json())
                    .then(data => {{
                        statusRunStopButton(data.status);
                    }})
                    .catch(error => console.error('Error fetching run/stop status:', error));
            }}
            
            //function reopenViewerPreserveState() {{
            //    if (viewer && viewer.isOpen()) {{
            //        const center = viewer.viewport.getCenter();
            //        const zoom = viewer.viewport.getZoom();
            //        sessionStorage.setItem('osdViewerState', JSON.stringify({{ center, zoom }}));
            //   }}
            //    viewer.open({{
            //        type: 'image',
            //        url: '/full_image.jpg?' + new Date().getTime()
            //    }});
            //}}
            function updateSetting(key, value) {{
                clientDictCaiDat[key] = parseFloat(value); // Assuming speeds are numbers
                fetch('/update_setting', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{ key: key, value: clientDictCaiDat[key] }})
                }})
                .then(response => response.json())
                .then(data => console.log("Setting update response:", data))
                .catch(error => console.error('Error updating setting:', error));
            }}

            function fetchAGVState() {{
                fetch('/get_agv_state') // Endpoint để lấy vị trí hiện tại của AGV
                    .then(response => response.json())
                    .then(data => {{
                        
                        //if (data.points_blue) {{
                        //    updateSpecialPoints(data.points_blue, 'blue', bluePointOverlays);
                        //}}
                        //if (data.points_red) {{
                        //    updateSpecialPoints(data.points_red, 'red', redPointOverlays);
                        //}}
                        if (data.agv_body_coords && data.agv_arrow_coords) {{ // Use new keys
                            updateAGVOnMap(data.agv_body_coords, data.agv_arrow_coords);
                        }}
                        if (data.update_points_paths === 1) {{
                            console.log("Reloading points and paths...", data.paths);
                            clearAllClientDataAndOverlays();

                            for (const name in data.points) {{
                                const p = data.points[name];
                                clientPointsData[name] = {{
                                    name: name,
                                    x: p[0],
                                    y: p[1],
                                    type: p[2],
                                    angle: p[3],
                                    osdOverlay: null,
                                    textOverlay: null
                                }};
                                drawPointOnOSD(clientPointsData[name]);
                                drawPointLabel(clientPointsData[name]);
                            }}

                            for (const pname in data.paths) {{
                                const p = data.paths[pname];
                                clientPathsData[pname] = {{
                                    name: pname,
                                    p1_name: p[0],
                                    p2_name: p[1],
                                    osdOverlay: null,
                                    textOverlay: null
                                }};
                                drawPathOnOSD(clientPathsData[pname]);
                                drawPathLabel(clientPathsData[pname]);
                            }}
                        }}
                        // 5. Tính toán lại ID điểm tiếp theo dựa trên dữ liệu mới
                        updateNextPointId();
                    }})
                    .catch(error => console.error('Error fetching AGV state:', error));
            }}

            function drawPointLabel(point) {{
                if (!viewer || !viewer.isOpen() || viewer.world.getItemCount() === 0) {{
                    return;
                }}
                const imageItem = viewer.world.getItemAt(0);
                if (!imageItem || !imageItem.source || !imageItem.source.dimensions) return;

                const imageWidth = imageItem.source.dimensions.x;
                const imageHeight = imageItem.source.dimensions.y;

                const xForOSD = point.x / imageWidth;
                const yForOSD = point.y / imageHeight;

                const label = document.createElement("div");
                label.textContent = point.name;
                label.style.color = "yellow";
                label.style.fontSize = "10px";
                label.style.fontWeight = "bold";
                label.style.backgroundColor = "rgba(0,0,0,0.5)";
                label.style.padding = "1px 2px";
                label.style.borderRadius = "3px";
                label.style.transform = "translate(-50%, -150%)"; // lệch lên trên điểm

                viewer.addOverlay({{
                    element: label,
                    location: new OpenSeadragon.Point(xForOSD, yForOSD),
                    placement: OpenSeadragon.Placement.CENTER
                }});

                point.textOverlay = label;
            }}

            function drawPathLabel(path) {{
                if (!viewer || !viewer.isOpen() || viewer.world.getItemCount() === 0) {{
                    return;
                }}
                const imageItem = viewer.world.getItemAt(0);
                if (!imageItem || !imageItem.source || !imageItem.source.dimensions) return;

                const imageWidth = imageItem.source.dimensions.x;
                const imageHeight = imageItem.source.dimensions.y;

                const p1 = clientPointsData[path.p1_name];
                const p2 = clientPointsData[path.p2_name];
                if (!p1 || !p2) return;

                const midX = (p1.x + p2.x) / 2;
                const midY = (p1.y + p2.y) / 2;

                const label = document.createElement("div");
                label.textContent = path.name;
                label.style.color = "lime";
                label.style.fontSize = "10px";
                label.style.fontWeight = "bold";
                label.style.backgroundColor = "rgba(0,0,0,0.5)";
                label.style.padding = "1px 2px";
                label.style.borderRadius = "3px";
                label.style.transform = "translate(-50%, -50%)";

                viewer.addOverlay({{
                    element: label,
                    location: new OpenSeadragon.Point(midX / imageWidth, midY / imageHeight),
                    placement: OpenSeadragon.Placement.CENTER
                }});

                path.textOverlay = label;
            }}



            function drawPointOnOSD(point) {{
                if (!viewer || !viewer.isOpen() || viewer.world.getItemCount() === 0) {{
                    console.warn("Viewer not ready for drawing point:", point.name);
                    return;
                }}

                const imageItem = viewer.world.getItemAt(0);
                let imageWidth = 0, imageHeight = 0;

                if (imageItem && imageItem.source && imageItem.source.dimensions) {{
                    imageWidth = imageItem.source.dimensions.x;
                    imageHeight = imageItem.source.dimensions.y;
                }}

                if (imageWidth === 0 || imageHeight === 0) return;

                const xForOSD = point.x / imageWidth;
                const yForOSD = point.y / imageHeight;

                const pointElement = document.createElement("div");
                pointElement.style.width = "6px";
                pointElement.style.height = "6px";
                pointElement.style.backgroundColor = "yellow";
                pointElement.style.border = "1px solid black";
                pointElement.style.borderRadius = "50%";
                pointElement.style.transform = "translate(-50%, -50%)";

                viewer.addOverlay({{
                    element: pointElement,
                    location: new OpenSeadragon.Point(xForOSD, yForOSD),
                    placement: OpenSeadragon.Placement.CENTER,
                    checkResize: false
                }});

                // Lưu lại overlay element để xóa khi cần openPointModal
                point.osdOverlay = pointElement;
            }}

            function drawPathOnOSD(path) {{
                if (!viewer || !viewer.isOpen() || viewer.world.getItemCount() === 0) return;

                const imageItem = viewer.world.getItemAt(0);
                let imageWidth = 0, imageHeight = 0;
                if (imageItem && imageItem.source && imageItem.source.dimensions) {{
                    imageWidth = imageItem.source.dimensions.x;
                    imageHeight = imageItem.source.dimensions.y;
                }}

                if (imageWidth === 0 || imageHeight === 0) return;

                const p1 = clientPointsData[path.p1_name];
                const p2 = clientPointsData[path.p2_name];
                if (!p1 || !p2) return;

                const x1 = p1.x / imageWidth;
                const y1 = p1.y / imageHeight;
                const x2 = p2.x / imageWidth;
                const y2 = p2.y / imageHeight;

                // Tạo SVG line
                const svgNS = "http://www.w3.org/2000/svg";
                const svg = document.createElementNS(svgNS, "svg");
                const line = document.createElementNS(svgNS, "line");
                line.setAttribute("x1", x1 * 100 + "%");
                line.setAttribute("y1", y1 * 100 + "%");
                line.setAttribute("x2", x2 * 100 + "%");
                line.setAttribute("y2", y2 * 100 + "%");
                line.setAttribute("stroke", "green");
                line.setAttribute("stroke-width", "2");
                svg.appendChild(line);

                viewer.addOverlay({{
                    element: svg,
                    location: new OpenSeadragon.Rect(0, 0, 1, 1), // toàn ảnh
                    placement: OpenSeadragon.Placement.TOP_LEFT,
                    checkResize: false
                }});

                path.osdOverlay = svg;
            }}


            // Gọi hàm này khi trang được tải để điền vào dropdown
            document.addEventListener('DOMContentLoaded', updateGridListDropdown);

            function updateAGVOnMap(bodyCoordsList, arrowCoordsList) {{ // NEW SIGNATURE
                if (!viewer || !viewer.isOpen() || !viewer.world.getItemCount() > 0) {{
                    return;
                }}

                const imageItem = viewer.world.getItemAt(0);
                if (!imageItem || !imageItem.source || !imageItem.source.dimensions) {{
                    return;
                }}

                // 1. Clear existing AGV body point overlays
                osdAgvBodyPointOverlays.forEach(overlayObj => {{
                    if (overlayObj && overlayObj.element) {{
                        viewer.removeOverlay(overlayObj.element);
                    }}
                }});
                osdAgvBodyPointOverlays.length = 0;

                // 2. Clear existing AGV arrow point overlays
                osdAgvArrowPointOverlays.forEach(overlayObj => {{
                    if (overlayObj && overlayObj.element) {{
                        viewer.removeOverlay(overlayObj.element);
                    }}
                }});
                osdAgvArrowPointOverlays.length = 0;

                let imageWidth = 0;
                let imageHeight = 0;
                if (viewer.world.getItemCount() > 0) {{
                    const tiledImage = viewer.world.getItemAt(0);
                    if (tiledImage && tiledImage.source && tiledImage.source.dimensions) {{
                        imageWidth = tiledImage.source.dimensions.x;
                        imageHeight = tiledImage.source.dimensions.y;
                    }} else if (tiledImage && tiledImage.source && tiledImage.source.width && tiledImage.source.height) {{
                        imageWidth = tiledImage.source.width;
                        imageHeight = tiledImage.source.height;
                    }}
                }}

                if (imageWidth === 0 || imageHeight === 0) {{
                    console.warn("updateAGVOnMap: Image dimensions not available.");
                    return; 
                }}

                // --- Draw AGV Body (Rectangle as Polygon) ---
                if (bodyCoordsList && bodyCoordsList.length > 0) {{   // NEW CONDITION
                    // END MODIFICATION
                    bodyCoordsList.forEach(coords => {{
                        const xForOSD = parseFloat(coords[0]) / imageWidth;
                        const yForOSD = parseFloat(coords[1]) / imageHeight;
                        const pointElement = document.createElement("div");
                        pointElement.style.width = "3px"; // Kích thước pixel của điểm AGV
                        pointElement.style.height = "3px";
                        pointElement.style.backgroundColor = "blue"; // Màu cho body AGV
                        pointElement.style.borderRadius = "50%";
                        pointElement.style.transform = "translate(-50%, -50%)";
                        viewer.addOverlay({{ element: pointElement, location: new OpenSeadragon.Point(xForOSD, yForOSD), placement: OpenSeadragon.Placement.CENTER, checkResize: false }});
                        osdAgvBodyPointOverlays.push({{ element: pointElement }});
                    }});
                }}

                // --- Draw AGV Arrow (Triangle as Polygon) ---
                // if (arrowCoordsList && arrowCoordsList.length === 3) // OLD CONDITION
                // BEGIN MODIFICATION: Change condition to handle more than 3 points for filled AGV arrow
                // Now, `arrowCoordsList` will contain all points to fill the AGV arrow.
                // So, we just check if the list has any points.
                if (arrowCoordsList && arrowCoordsList.length > 0) {{   // NEW CONDITION
                // END MODIFICATION
                    arrowCoordsList.forEach(coords => {{
                        const xForOSD = parseFloat(coords[0]) / imageWidth;
                        const yForOSD = parseFloat(coords[1]) / imageHeight;
                        const pointElement = document.createElement("div");
                        pointElement.style.width = "3px"; // Kích thước pixel của điểm mũi tên
                        pointElement.style.height = "3px";
                        pointElement.style.backgroundColor = "red"; // Màu cho mũi tên AGV
                        pointElement.style.borderRadius = "50%";
                        pointElement.style.transform = "translate(-50%, -50%)";
                        viewer.addOverlay({{ element: pointElement, location: new OpenSeadragon.Point(xForOSD, yForOSD), placement: OpenSeadragon.Placement.CENTER, checkResize: false }});
                        osdAgvArrowPointOverlays.push({{ element: pointElement }});
                    }});
                }}
            }}
            
            // Shared onDraw function for SVG overlays
            function svgOnDraw(position, size, element) {{
                element.setAttribute('width', size.x);
                element.setAttribute('height', size.y);

                let imagePixelWidth = 0; // Default to 0 if not found
                let imagePixelHeight = 0; // Default to 0 if not found

                if (viewer.world.getItemCount() > 0) {{
                    const tiledImage = viewer.world.getItemAt(0);
                    if (tiledImage && tiledImage.source && tiledImage.source.dimensions) {{
                        imagePixelWidth = tiledImage.source.dimensions.x;
                        imagePixelHeight = tiledImage.source.dimensions.y;
                    }} else if (tiledImage && tiledImage.source && tiledImage.source.width && tiledImage.source.height) {{
                        imagePixelWidth = tiledImage.source.width;
                        imagePixelHeight = tiledImage.source.height;
                    }}
                }}

                if (imagePixelWidth > 0 && imagePixelHeight > 0) {{
                    element.setAttribute("viewBox", `0 0 ${{imagePixelWidth}} ${{imagePixelHeight}}`);
                }} else {{
                    // console.warn("svgOnDraw: Image dimensions not available, cannot set viewBox reliably.");
                    // Fallback to a tiny viewBox or hide if dimensions are unknown to prevent misdrawing
                    element.setAttribute("viewBox", `0 0 1 1`); 
                }}
            }}



            function updateSpecialPoints(pointsArray, colorString, overlayStorageArray) {{
                if (!viewer || !viewer.isOpen()) {{
                    console.warn(`Viewer not ready for special points (${"{colorString}"})`);
                    return;
                }}

                // 1. Clear existing overlays for this color
                overlayStorageArray.forEach(overlayObj => {{
                    if (overlayObj && overlayObj.element) {{ // Bỏ kiểm tra isOverlayVisible
                        viewer.removeOverlay(overlayObj.element);
                    }}
                }});
                overlayStorageArray.length = 0; // Clear the storage array

                let imageWidth = 0; // Default to 0
                let imageHeight = 0; // Default to 0

                if (viewer.world.getItemCount() > 0) {{
                    const tiledImage = viewer.world.getItemAt(0);
                    if (tiledImage && tiledImage.source && tiledImage.source.dimensions) {{
                        imageWidth = tiledImage.source.dimensions.x;
                        imageHeight = tiledImage.source.dimensions.y;
                    }} else if (tiledImage && tiledImage.source && tiledImage.source.width && tiledImage.source.height) {{
                        imageWidth = tiledImage.source.width;
                        imageHeight = tiledImage.source.height;
                    }}
                }}

                if (imageWidth === 0 || imageHeight === 0) {{
                    // console.warn(`updateSpecialPoints: Image dimensions not available for ${"{colorString}"} points.`);
                    return; // Don't draw if image dimensions are unknown
                }}


                pointsArray.forEach(pointCoords => {{ 
                    const x_intent = parseFloat(pointCoords[0]);
                    const y_intent = parseFloat(pointCoords[1]);

                    if (isNaN(x_intent) || isNaN(y_intent)) {{
                        console.warn(`Invalid coordinates for ${"{colorString}"} point:`, pointCoords);
                        return;
                    }}

                    

                    let xForOSD = x_intent;
                    let yForOSD = y_intent;

                    xForOSD = x_intent / imageWidth;
                    yForOSD = y_intent / imageHeight;
                    
                    const pointLocation = new OpenSeadragon.Point(xForOSD, yForOSD);
                    const pointElement = document.createElement("div");
                    pointElement.style.width = "2px";
                    pointElement.style.height = "2px";
                    pointElement.style.backgroundColor = colorString;
                    pointElement.style.borderRadius = "50%";
                    pointElement.style.border = `1px solid dark${"{colorString}"}`;
                    pointElement.style.transform = "translate(-50%, -50%)";

                    viewer.addOverlay({{
                        element: pointElement,
                        location: pointLocation,
                        placement: OpenSeadragon.Placement.CENTER,
                        checkResize: false
                    }});
                    overlayStorageArray.push({{ element: pointElement }}); 
                }});
            }}

            function loadCurrentState() {{
                fetch('/get_current_state')
                    .then(response => response.json())
                    .then(data => {{
                        if (data.status === 'success') {{
                            clearAllClientDataAndOverlays(); // Clear existing before loading
                            
                            clientGridData = data.grid_data || {{}};
                            populateGridList();
                            const serverPoints = data.points_data || {{}};
                            const serverPaths = data.paths_data || {{}};

                            // Load points first
                            for (const name in serverPoints) {{
                                const pArray = serverPoints[name];
                                clientPointsData[name] = {{ name: name, x: pArray[0], y: pArray[1], type: pArray[2], angle: pArray[3], osdOverlay: null, textOverlay: null }};
                                }}
                            console.log("clientPathsData for :", clientPathsData);
                            // Then load paths (which depend on points)
                            for (const name in serverPaths) {{
                                const pArray = serverPaths[name];
                                if(clientPointsData[pArray[0]] && clientPointsData[pArray[1]]) {{ 
                                    clientPathsData[name] = {{ name: name, p1_name: pArray[0], p2_name: pArray[1], osdOverlay: null, textOverlay: null }}; 
                                }}
                            }}
                            console.log("clientPathsData for :", clientPathsData);
                            updateNextPointId();
                            
                        }}
                    }});
            }}

            function confirmMapSelection() {{
                const mapName = document.getElementById('selectMapName').value;
                if (!mapName) {{ alert("Vui lòng chọn một bản đồ."); return; }}
                
                // Clear current points and paths when map changes
                //if (confirm("Đổi bản đồ sẽ xóa các điểm và đường hiện tại trên giao diện. Bạn có muốn tiếp tục?")) {{
                fetch('/confirm_map_update', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{ ten_ban_do: mapName, update: 1 }})
                }})
                .then(response => response.json())
                .then(data => {{
                    console.log("Map selection response:", data);
                    if (data.status === 'success') {{
                        clearAllClientDataAndOverlays(); // Clear points/paths
                        alert(`Bản đồ "${"{mapName}"}" đã được tải.`);
                        window.location.reload(); // Refresh toàn bộ trang web sau khi thay đổi điểm
                    }} else {{
                        alert("Lỗi khi cập nhật bản đồ: " + data.message);
                    }}
                }})
                .catch(error => console.error('Error confirming map selection:', error));
                // }}
            }}
            
            function clearAllClientDataAndOverlays() {{
                for (const name in clientPointsData) removePointOverlay(name);
                for (const name in clientPathsData) removePathOverlay(name);
                clientPointsData = {{}};
                clientGridData = {{}};
                clientPathsData = {{}};
                nextPointNumericId = 1; // Reset ID counter
            }}
            function updatePointsPath(Points, Paths) {{
                clientPointsData = Points;
                clientPathsData = Paths;
                
                drawPointOnOSD(clientPointsData[name]);
                drawPathOnOSD(clientPathsData[name]);
            }}

            function removePointOverlay(name) {{
                const point = clientPointsData[name];
                if (point) {{
                    if (point.osdOverlay) {{
                        viewer.removeOverlay(point.osdOverlay);
                        point.osdOverlay = null;
                    }}
                    if (point.textOverlay) {{
                        viewer.removeOverlay(point.textOverlay);
                        point.textOverlay = null;
                    }}
                }}
            }}

            function removePathOverlay(name) {{
                const path = clientPathsData[name];
                if (path) {{
                    if (path.osdOverlay) {{
                        viewer.removeOverlay(path.osdOverlay);
                        path.osdOverlay = null;
                    }}
                    if (path.textOverlay) {{
                        viewer.removeOverlay(path.textOverlay);
                        path.textOverlay = null;
                    }}
                }}
            }}

            function confirmAGVUpdate() {{
                const agvData = {{
                    toa_do_x: parseFloat(document.getElementById('inputAgvX').value) || 0,
                    toa_do_y: parseFloat(document.getElementById('inputAgvY').value) || 0,
                    goc_agv: parseFloat(document.getElementById('inputAgvAngle').value) || 0,
                    update: 1, // This is a confirm action from the button
                    setup: 0   // Not a setup action when button is clicked
                }};
                fetch('/confirm_agv_update', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify(agvData)
                }})
                .then(response => response.json())
                .then(data => {{
                    console.log("AGV confirm update response:", data);
                    // Reset button state on error as well
                    document.getElementById('btnToggleAGVPositionMode')?.classList.remove('active');
                    currentMode = 'none';
                    if(data.status === 'success') {{
                        alert("Vị trí AGV đã được gửi cập nhật lên server.");
                    }} else {{
                        alert("Lỗi xác nhận cập nhật vị trí AGV: " + data.message);
                    }}
                }})
                .catch(error => console.error('Error confirming AGV update:', error));
            }}
             
            function toggleMode(mode) {{
                // If trying to enter 'addPath' but it's already in a sub-mode, treat as exiting 'addPath'
                // Khi nhấn nút "Đặt vị trí AGV", gửi trạng thái setup lên server
                if (mode === 'agvPos') {{
                    // Logic toggle đã chạy ở trên, vì vậy `currentMode` đã phản ánh trạng thái MỚI.
                    // Nếu `currentMode` bây giờ là 'agvPos', có nghĩa là chế độ vừa được BẬT.
                    const isNowActive = (currentMode === 'agvPos');
                    const agvData = {{
                        toa_do_x: parseFloat(document.getElementById('inputAgvX').value) || 0,
                        toa_do_y: parseFloat(document.getElementById('inputAgvY').value) || 0,
                        goc_agv: parseFloat(document.getElementById('inputAgvAngle').value) || 0,
                        setup: isNowActive ? 0 : 1, // Gửi 1 nếu BẬT, 0 nếu TẮT
                        update: 0 // Không phải là hành động xác nhận cuối cùng
                    }};

                    fetch('/confirm_agv_update', {{
                        method: 'POST',
                        headers: {{ 'Content-Type': 'application/json' }},
                        body: JSON.stringify(agvData)
                    }})
                    .then(response => response.json())
                    .then(data => {{
                        console.log(`AGV setup mode ${{isEnteringSetupMode ? 'bật' : 'tắt'}}. Phản hồi:`, data);
                        if (data.status !== 'success') {{
                            alert(`Lỗi khi ${{isEnteringSetupMode ? 'bật' : 'tắt'}} chế độ đặt vị trí: ` + data.message);
                        }}
                    }})
                    .catch(error => console.error('Error toggling AGV setup mode:', error));
                }}

                if (mode === 'addPath' && (currentMode === 'addPath_start' || currentMode === 'addPath_end')) {{
                    currentMode = 'none';
                }} else if (currentMode === mode) {{
                    currentMode = 'none';
                }} else {{
                    currentMode = mode;
                }}

                if (tempPathStartPointName && !(currentMode === 'addPath_start' || currentMode === 'addPath_end')) {{
                    highlightPoint(tempPathStartPointName, false); // Unhighlight if exiting addPath mode
                    tempPathStartPointName = null;
                }}
                
                if (currentMode === 'addPath') currentMode = 'addPath_start'; // Default to start of addPath

                updateModeButtons();

            }}
            
            function updateModeButtons() {{
                ['btnToggleAGVPositionMode', 'btnAddPointMode', 'btnEditPointMode', 'btnAddPathMode', 'btnDeletePathMode'].forEach(id => {{
                    const btn = document.getElementById(id);
                    if (btn) btn.classList.remove('active');
                }});
                if (currentMode === 'agvPos') document.getElementById('btnToggleAGVPositionMode')?.classList.add('active');
                else if (currentMode === 'addPoint') document.getElementById('btnAddPointMode')?.classList.add('active');
                else if (currentMode === 'editPoint') document.getElementById('btnEditPointMode')?.classList.add('active');
                else if (currentMode === 'addPath_start' || currentMode === 'addPath_end') document.getElementById('btnAddPathMode')?.classList.add('active');
                else if (currentMode === 'deletePath') document.getElementById('btnDeletePathMode')?.classList.add('active');
            }}

            // --- Point Modal Functions ---
            function openPointModal(pointData, clickX, clickY, isEditing = false) {{
                const modal = document.getElementById('pointModal');

                document.getElementById('modalOriginalPointName').value = pointData ? pointData.name : '';

                if (isEditing && pointData) {{
                    document.getElementById('modalPointName').value = pointData.name;
                }} else {{
                    document.getElementById('modalPointName').value = `P${{nextPointNumericId}}`;
                    // nextPointNumericId++;  // tăng ngay khi mở modal
                }}

                document.getElementById('modalOriginalPointName').value = pointData ? pointData.name : '';
                document.getElementById('modalPointName').value = pointData ? pointData.name : `P${"{nextPointNumericId}"}`;
                document.getElementById('modalPointX').value = pointData ? pointData.x : clickX;
                document.getElementById('modalPointY').value = pointData ? pointData.y : clickY;
                document.getElementById('modalPointType').value = pointData ? pointData.type : 'không hướng';
                document.getElementById('modalPointAngle').value = pointData ? (pointData.angle || 0) : 0;
                
                document.getElementById('pointModalTitle').textContent = isEditing ? 'Sửa Điểm' : 'Thêm Điểm Mới';
                document.getElementById('deletePointModalButton').style.display = isEditing ? 'inline-block' : 'none';
                
                toggleModalPointAngleInput();
                modal.style.display = 'block';
            }}

            function closePointModal() {{
                document.getElementById('pointModal').style.display = 'none';
            }}

            function toggleModalPointAngleInput() {{
                const type = document.getElementById('modalPointType').value;
                document.getElementById('modalPointAngleContainer').style.display = type === 'có hướng' ? 'block' : 'none';
            }}

            function submitPointModal() {{
                const originalName = document.getElementById('modalOriginalPointName').value;
                const newName = document.getElementById('modalPointName').value.trim();
                if (!newName) {{ alert("Tên điểm không được để trống."); return; }}
                const xVal = document.getElementById('modalPointX').value;
                const yVal = document.getElementById('modalPointY').value;

                if (xVal.trim() === '' || yVal.trim() === '') {{
                    alert("Tọa độ X và Y không được để trống.");
                    return;
                }}

                const xCoord = parseInt(xVal, 10); // Sử dụng radix 10
                const yCoord = parseInt(yVal, 10); // Sử dụng radix 10

                if (isNaN(xCoord) || isNaN(yCoord)) {{
                    alert("Tọa độ X và Y phải là số hợp lệ.");
                    return;
                }}
                const pointDetails = {{
                    name: newName,
                    x: xCoord,
                    y: yCoord,
                    type: document.getElementById('modalPointType').value,
                    angle: document.getElementById('modalPointType').value === 'có hướng' ? parseFloat(document.getElementById('modalPointAngle').value) : 0
                }};

                // Check for name collision if adding new or renaming to a different existing name
                if ((!originalName || originalName !== newName) && clientPointsData[newName]) {{
                    alert(`Tên điểm "${"{newName}"}" đã tồn tại. Vui lòng chọn tên khác.`);
                    return;
                }}
                
                const action = originalName ? 'update_point' : 'add_point';
                let serverPayload = {{ ...pointDetails }};
                if (action === 'update_point' && originalName !== newName) {{
                    serverPayload.old_name = originalName; // Signal rename to server
                }}
                
                fetch(`/${"{action}"}`, {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify(serverPayload)
                }})
                .then(response => response.json())
                .then(data => {{
                    if (data.status === 'success') {{
                        // Đóng modal sau khi gửi yêu cầu thành công.
                        // Tiến trình nền (fetchAGVState -> updatePointsPath) sẽ tự động
                        // lấy dữ liệu mới nhất từ server và vẽ lại toàn bộ điểm/đường đi một cách chính xác.
                        // Việc này giúp tránh xung đột cập nhật và lỗi hiển thị trùng lặp.
                        closePointModal();
                    }} else {{
                        alert("Lỗi từ server: " + data.message);
                        console.error("[submitPointModal] Server error response:", data.message);
                    }}
                }})
                .catch(error => {{
                    console.error('Error submitting point:', error);
                    alert("Lỗi kết nối khi gửi dữ liệu điểm.");
                }});
            }}

            function deletePointFromModal() {{
                const pointName = document.getElementById('modalOriginalPointName').value;
                if (pointName && confirm(`Bạn có chắc muốn xóa điểm "${{pointName}}"? Thao tác này cũng sẽ xóa các đường đi liên quan.`)) {{
                    fetch('/delete_point', {{
                        method: 'POST',
                        headers: {{ 'Content-Type': 'application/json' }},
                        body: JSON.stringify({{ name: pointName }})
                    }})
                    .then(response => response.json())
                    .then(data => {{
                        if (data.status === 'success') {{
                            // Đóng modal. Tiến trình nền sẽ tự động cập nhật giao diện và tính toán lại ID điểm tiếp theo.
                            closePointModal();
                        }} else {{
                            alert("Lỗi xóa điểm từ server: " + data.message);
                        }}
                    }})
                    .catch(error => console.error('Error deleting point:', error));
                }}
            }}
            
            function deletePoint(pointName, calledFromPathUpdate = false) {{ // calledFromPathUpdate to avoid circular calls if server cascades
                fetch('/delete_point', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{ name: pointName }})
                }})
                .then(response => response.json())
                .then(data => {{
                    if (data.status === 'success') {{
                        removePointOverlay(pointName);
                        // Client-side cascade delete paths connected to this point is handled below
                        delete clientPointsData[pointName];
                        
                        // Client-side cascade delete paths connected to this point
                        const pathsToDeleteClient = [];
                        for (const pathName in clientPathsData) {{
                            if (clientPathsData[pathName].p1_name === pointName || clientPathsData[pathName].p2_name === pointName) {{
                                pathsToDeleteClient.push(pathName);
                            }}
                        }}
                        pathsToDeleteClient.forEach(pName => {{
                            removePathOverlay(pName);
                            delete clientPathsData[pName];
                        }});
                        console.log(`Point ${"{pointName}"} and its paths deleted from client.`);
                        if (!calledFromPathUpdate) alert(`Điểm "${"{pointName}"}" và các đường liên quan đã được xóa.`);
                    }} else {{
                        alert("Lỗi xóa điểm từ server: " + data.message);
                    }}
                }})
                .catch(error => console.error('Error deleting point:', error));
            }}

            function updatePathsAfterPointRename(oldPointName, newPointName) {{
                const pathsToUpdateOnServer = [];
                for (const pathId in clientPathsData) {{
                    let pathChanged = false;
                    const currentPath = clientPathsData[pathId];
                    let newP1Name = currentPath.p1_name;
                    let newP2Name = currentPath.p2_name;

                    if (currentPath.p1_name === oldPointName) {{
                        newP1Name = newPointName;
                        pathChanged = true;
                    }}
                    if (currentPath.p2_name === oldPointName) {{
                        newP2Name = newPointName;
                        pathChanged = true;
                    }}

                    if (pathChanged) {{
                        const newPathName = `${"{newP1Name}"}_${"{newP2Name}"}`;
                        removePathOverlay(pathId); // Remove old path overlay
                        delete clientPathsData[pathId]; // Delete old path entry

                        clientPathsData[newPathName] = {{
                            name: newPathName,
                            p1_name: newP1Name,
                            p2_name: newP2Name,
                            osdOverlay: null, textOverlay: null
                        }};
                        pathsToUpdateOnServer.push({{old_path_name: pathId, new_path_data: clientPathsData[newPathName]}});
                    }}
                }}
                if (pathsToUpdateOnServer.length > 0) {{
                    fetch('/update_paths_after_point_rename', {{ // New endpoint needed
                        method: 'POST',
                        headers: {{ 'Content-Type': 'application/json' }},
                        body: JSON.stringify({{ renamed_point_old_name: oldPointName, renamed_point_new_name: newPointName, paths_to_update: pathsToUpdateOnServer }})
                    }})
                    .then(res => res.json())
                    .then(data => console.log("Server response from path updates after point rename:", data));
                }}
            }}


            function removePointOverlay(pointName) {{
                // console.log("[removePointOverlay] Attempting to remove overlays for point:", pointName);
                if (clientPointsData[pointName]) {{
                    if (clientPointsData[pointName].osdOverlay) viewer.removeOverlay(clientPointsData[pointName].osdOverlay);
                    if (clientPointsData[pointName].textOverlay) viewer.removeOverlay(clientPointsData[pointName].textOverlay);
                    clientPointsData[pointName].osdOverlay = null;
                    clientPointsData[pointName].textOverlay = null;
                }}
            }}
            
            function highlightPoint(pointName, SvgOverlay) {{
                const point = clientPointsData[pointName];
                if (point && point.osdOverlay) {{
                    point.osdOverlay.style.backgroundColor = SvgOverlay ? "blue" : "red";
                    point.osdOverlay.style.border = SvgOverlay ? "2px solid darkblue" : "1px solid darkred";
                }}
            }}
            
            function addPath(p1Name, p2Name) {{
                const pathName = `${"{p1Name}"}_${"{p2Name}"}`;
                const reversePathName = `${"{p2Name}"}_${"{p1Name}"}`;

                if (clientPathsData[pathName] || clientPathsData[reversePathName]) {{
                    alert(`Đường đi giữa ${"{p1Name}"} và ${"{p2Name}"} đã tồn tại.`);
                    return;
                }}

                const pathData = {{ name: pathName, p1_name: p1Name, p2_name: p2Name }};
                fetch('/add_path', {{
                    method: 'POST',
                    headers: {{'Content-Type': 'application/json'}},
                    body: JSON.stringify(pathData) // Server expects {{name, p1_name, p2_name}}
                }})
                .then(res => {{
                    if (!res.ok) {{
                        // Attempt to parse error as JSON, then fall back
                        return res.json().then(errData => {{
                            throw new Error(errData.message || `Server error: ${"{res.status}"}`);
                        }}).catch(() => {{ // If res.json() fails or it's not JSON
                            throw new Error(`Server error: ${"{res.status}"} ${"{res.statusText}"}`);
                        }});
                    }}
                    return res.json(); // If res.ok, proceed to parse JSON
                }})
                .then(data => {{
                    if (data.status === 'success') {{
                        clientPathsData[pathName] = {{ ...pathData, osdOverlay: null, textOverlay: null }};
                    }} else {{
                        alert("Lỗi thêm đường từ server: " + (data.message || "Lỗi không xác định từ server."));
                    }}
                }})
                .catch(error => {{ // Catches errors from !res.ok, network failures, or JSON parsing failures
                    console.error('Error adding path:', error);
                    alert("Không thể thêm đường: " + error.message);
                }});
            }}
            
            function deletePath(pathName) {{
                fetch('/delete_path', {{
                    method: 'POST',
                    headers: {{'Content-Type': 'application/json'}},
                    body: JSON.stringify({{name: pathName}})
                }})
                .then(res => res.json())
                .then(data => {{
                    if (data.status === 'success') {{
                        alert(`Đường "${"{pathName}"}" đã được xóa.`);
                    }} else {{
                        alert("Lỗi xóa đường từ server: " + data.message);
                    }}
                }});
            }}

            function findNearestPoint(x, y, mustSnap = false, snapRadius = 100) {{
                let nearest = null;
                let minDistSq = Infinity;

                if (Object.keys(clientPointsData).length === 0) {{
                    console.log("[findNearestPoint] No points in clientPointsData.");
                    return null;
                }}

                console.log(`[findNearestPoint] Searching for nearest point near (${{x}}, ${{y}}). mustSnap: ${{mustSnap}}, snapRadius: ${{snapRadius}}`);
                console.log("[findNearestPoint] Number of points in clientPointsData:", Object.keys(clientPointsData).length);
                // console.log("[findNearestPoint] Current clientPointsData:", JSON.stringify(clientPointsData)); // Log the actual dat
                for (const name in clientPointsData) {{
                    const p = clientPointsData[name];
                    if (typeof p.x !== 'number' || typeof p.y !== 'number' || isNaN(p.x) || isNaN(p.y)) {{
                        console.log(`[findNearestPoint] Skipping point "${{name}}" due to invalid coordinates: (${{p.x}}, ${{p.y}})`);
                        
                        continue; // Skip this point
                    }}
                    const distSq = (p.x - x)**2 + (p.y - y)**2;
                    if (distSq < minDistSq) {{
                        minDistSq = distSq;
                        nearest = p;
                    }}
                }}
                // Revised logic:
                console.log(`[findNearestPoint] Nearest point candidate: ${{nearest ? nearest.name : 'None'}}, Distance: ${{Math.sqrt(minDistSq)}}`);
                if (!nearest || minDistSq === Infinity) {{ // Check if any point was actually found
                    return null; // No points exist
                }}
                if (mustSnap && Math.sqrt(minDistSq) > snapRadius) {{
                    return null; // Snapping required, but nearest is too far
                }}
                return nearest; // Return the nearest point found (either snapped or just the closest)
            }}

            // Helper function to calculate the shortest distance from a point to a line segment
            function pointToLineSegmentDistance(px, py, x1, y1, x2, y2) {{
                const lenSq = (x2 - x1)**2 + (y2 - y1)**2;
                if (lenSq === 0) {{ // It's a point, not a segment
                    return Math.sqrt((px - x1)**2 + (py - y1)**2);
                }}
                // Parameter t represents the projection of point (px, py) onto the line AB
                let t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / lenSq;
                // Clamp t to the range [0, 1] to ensure the projection is on the segment
                t = Math.max(0, Math.min(1, t));
                // Find the point on the segment AB that is closest to (px, py)
                const closestX = x1 + t * (x2 - x1);
                const closestY = y1 + t * (y2 - y1);
                // Calculate the distance between (px, py) and the closest point on the segment
                return Math.sqrt((px - closestX)**2 + (py - closestY)**2);
            }}
            
            function findNearestPath(x, y) {{ // Bỏ searchRadius khỏi logic trả về chính
                let nearestPath = null;
                let minDistanceToSegment = Infinity; // Use this for point-to-segment distance

                console.log(`[findNearestPath] Searching for nearest path near (${"{x}"}, ${{y}}).`);
                console.log("[findNearestPath] Number of paths in clientPathsData:", Object.keys(clientPathsData).length);
                
                if (Object.keys(clientPathsData).length === 0) {{
                    console.log("[findNearestPath] No paths in clientPathsData.");
                    return null;
                }}
                console.log("data clientPathsData" , clientPathsData);
                for (const pathName in clientPathsData) {{
                    const path = clientPathsData[pathName];
                    const p1 = clientPointsData[path.p1_name];
                    const p2 = clientPointsData[path.p2_name];
                    if (!p1 || !p2) {{
                        console.warn(`[findNearestPath] Skipping path "${"{pathName}"}" due to missing points.`);
                        continue;
                    }}
                     if (typeof p1.x !== 'number' || typeof p1.y !== 'number' || typeof p2.x !== 'number' || typeof p2.y !== 'number' ||
                        isNaN(p1.x) || isNaN(p1.y) || isNaN(p2.x) || isNaN(p2.y)) {{
                        console.warn(`[findNearestPath] Skipping path "${"{pathName}"}" due to invalid point coordinates.`);
                        continue; // Skip this path
                    }}

                    // Use the accurate point-to-line-segment distance
                    const distance = pointToLineSegmentDistance(x, y, p1.x, p1.y, p2.x, p2.y);

                    if (distance < minDistanceToSegment) {{
                        minDistanceToSegment = distance;
                        nearestPath = path;
                    }}
                }}
                
                if (nearestPath) {{
                    console.log(`[findNearestPath] Geometrically closest path found: ${{nearestPath.name}}, Distance: ${{minDistanceToSegment}}`);
                }} else {{
                    console.log("[findNearestPath] No valid path found after checking all paths.");
                }}

                // Always return the nearest path found, regardless of distance
                return nearestPath;
            }}

            // --- Save/Load Functions ---
            function convertClientPointsToServerFormat() {{
                const serverPoints = {{}};
                for (const name in clientPointsData) {{
                    const p = clientPointsData[name];
                    serverPoints[name] = [p.x, p.y, p.type, p.angle];
                }}
                return serverPoints;
            }}
            function convertClientPathsToServerFormat() {{
                const serverPaths = {{}};
                for (const name in clientPathsData) {{
                    const p = clientPathsData[name];
                    serverPaths[name] = [p.p1_name, p.p2_name];
                }}
                return serverPaths;
            }}

            function savePointList() {{
                const filename = document.getElementById('inputPointListNameSave').value.trim();
                if (!filename) {{ alert("Vui lòng nhập tên file để lưu danh sách điểm."); return; }}
                
                const serverFormattedPoints = convertClientPointsToServerFormat();

                fetch('/save_points', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{ filename: filename }}) 
                }})
                .then(response => response.json())
                .then(data => {{
                    if (data.status === 'success') {{
                        alert(`Danh sách điểm đã được lưu vào file: ${"{filename}"}.json`);
                        updateSavedFileLists(); // Refresh dropdown
                    }} else {{
                        alert("Lỗi lưu danh sách điểm: " + data.message);
                    }}
                }})
                .catch(error => console.error('Error saving point list:', error));
            }}

            function loadPointList() {{
                const filename = document.getElementById('selectSavedPointList').value;
                if (!filename) {{ alert("Vui lòng chọn danh sách điểm để tải."); return; }} 
                if (confirm("Tải danh sách điểm mới sẽ xóa tất cả các điểm và đường đi hiện tại. Tiếp tục?")) {{
                    fetch(`/load_points/${{filename}}`)
                    .then(response => response.json())
                    .then(data => {{
                        if (data.status === 'success') {{
                            clearAllClientDataAndOverlays();
                            // Hàm updatePointsPath sẽ xử lý việc xóa dữ liệu cũ, tải và vẽ lại điểm mới,
                            // đồng thời xóa các đường đi cũ và cập nhật lại ID điểm tiếp theo.
                            updatePointsPath(data.points_data || {{}}, {{}}); // Truyền vào một đối tượng đường đi rỗng
                            alert(`Danh sách điểm "${{filename}}.json" đã được tải.`);
                        }} else {{
                            alert("Lỗi tải danh sách điểm: " + data.message);
                        }}
                    }})
                    .catch(error => console.error('Error loading point list:', error));
                }}
            }}
            
            function savePathList() {{
                const filename = document.getElementById('inputPathListNameSave').value.trim();
                if (!filename) {{ alert("Vui lòng nhập tên file để lưu danh sách đường."); return; }}
                
                const serverFormattedPaths = convertClientPathsToServerFormat();
                fetch('/save_paths', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    // Server saves its own state, no need to send client data
                    body: JSON.stringify({{ filename: filename }})
                }})
                .then(response => response.json())
                .then(data => {{
                    if (data.status === 'success') {{
                        alert(`Danh sách đường đã được lưu vào file: ${"{filename}"}.json`);
                        updateSavedFileLists();
                    }} else {{
                        alert("Lỗi lưu danh sách đường: " + data.message);
                    }}
                }})
                .catch(error => console.error('Error saving path list:', error));
            }}

            function loadPathList() {{
                const filename = document.getElementById('selectSavedPathList').value;
                if (!filename) {{ alert("Vui lòng chọn danh sách đường để tải."); return; }}
                fetch(`/load_paths/${"{filename}"}`)
                .then(response => response.json())
                .then(data => {{
                    if (data.status === 'success') {{
                        // Clear existing paths, but keep points
                        for(const name in clientPathsData) removePathOverlay(name);
                        clientPathsData = {{}};

                        const serverPaths = data.paths_data || {{}};
                        for (const name in serverPaths) {{
                            const pArray = serverPaths[name];
                            // Ensure points for this path exist before drawing
                            const p1Exists = clientPointsData[pArray[0]];
                                const p2Exists = clientPointsData[pArray[1]];
                                if(p1Exists && p2Exists) {{
                                    clientPathsData[name] = {{ name: name, p1_name: pArray[0], p2_name: pArray[1], osdOverlay: null, textOverlay: null }};
                                }} else {{
                                    console.warn(`[loadCurrentState] Skipping path "${{name}}" because point "${{pArray[0]}}" (${{p1Exists ? 'exists' : 'missing'}}) or point "${{pArray[1]}}" (${{p2Exists ? 'exists' : 'missing'}}) is missing from clientPointsData.`);
                            }}
                        }}
                        alert(`Danh sách đường "${"{filename}"}.json" đã được tải.`);
                    }} else {{
                        alert("Lỗi tải danh sách đường: " + data.message);
                    }}
                }})
                .catch(error => console.error('Error loading path list:', error));
            }}

            function updateSavedFileLists(selectedPointFile, selectedPathFile) {{
                fetch('/get_saved_file_lists')
                    .then(response => response.json())
                    .then(data => {{
                        if (data.status === 'success') {{
                            populateSelect('selectSavedPointList', data.point_lists, selectedPointFile);
                            populateSelect('selectSavedPathList', data.path_lists, selectedPathFile);
                        }}
                    }});
            }}

            function populateSelect(selectId, items, currentValue) {{ // currentValue is optional
                const select = document.getElementById(selectId);
                select.innerHTML = '<option value="">-- Chọn file --</option>'; // Add a default empty option
                items.forEach(item => {{
                    const option = document.createElement('option');
                    option.value = item; // item is filename without .json
                    option.textContent = item;
                    if (item === currentValue) {{
                        option.selected = true;
                    }}
                    select.appendChild(option);
                }});
            }}

            function toggleSpeaker() {{
                fetch('/api/toggle_speaker', {{ method: 'POST' }})
                    .then(response => response.json())
                    .then(data => {{
                        if (data.status === 'success') {{
                            alert('Đã gửi yêu cầu bật loa!');
                        }} else {{
                            alert('Lỗi: ' + data.message);
                        }}
                    }})
                    .catch(error => console.error('Error toggling speaker:', error));
            }}


        </script>
    </body>
    </html> 
    """
    return html_content
# get_current_state
@app.route('/full_image.jpg')
def full_image_route():
    global current_image, image_lock, image_initialized
    if not image_initialized:
        timeout = 5 
        start_time = time.time()
        while not image_initialized and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        if not image_initialized: # Still not ready
            error_img = np.zeros((100, 100, 3), dtype=np.uint8)
            cv2.putText(error_img, "Error", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2)
            _, enc_err_img = cv2.imencode('.jpg', error_img)
            return Response(enc_err_img.tobytes(), mimetype='image/jpeg'), 503


    with image_lock:
        img_copy = current_image.copy()

    success, encoded_image = cv2.imencode('.jpg', img_copy, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not success:
        return "Error encoding image", 500
    return Response(encoded_image.tobytes(), mimetype='image/jpeg')

@app.route('/favicon.ico')
def favicon():
    return send_from_directory(os.path.join(app.root_path, 'static'),
                               'favicon.ico', mimetype='image/vnd.microsoft.icon', conditional=True)

# --- Settings Endpoints ---
@app.route('/update_setting', methods=['POST'])
def update_setting_route():
    global dict_cai_dat
    data = request.get_json()
    if data and 'key' in data and 'value' in data:
        key = data['key']
        value = data['value']
        if key in dict_cai_dat:
            try:
                dict_cai_dat[key] = type(dict_cai_dat[key])(value) 
                print(f"Server updated setting: {key} = {dict_cai_dat[key]}")
                return jsonify({"status": "success", "message": f"Setting {key} updated"}), 200
            except ValueError:
                 return jsonify({"status": "error", "message": "Invalid value type for setting"}), 400
        return jsonify({"status": "error", "message": "Invalid setting key"}), 400
    return jsonify({"status": "error", "message": "Invalid setting data"}), 400

# --- Map Selection Endpoints ---
@app.route('/confirm_map_update', methods=['POST'])
def confirm_map_update_route():
    global dict_chon_ban_do, current_image0, image_lock, danh_sach_diem, danh_sach_duong, update_ds_diem_duong
    data = request.get_json()
    if data and 'ten_ban_do' in data and 'update' in data:
        if data['update'] == 1:
            new_map_name = data['ten_ban_do']
            map_path = os.path.join(PATH_MAPS_DIR, new_map_name)
            print(new_map_name, map_path)
            loaded_successfully = False
            if os.path.exists(map_path):
                try:
                    img_to_load = None
                    if new_map_name.lower().endswith(".npy"):
                        img_array = np.load(map_path)
                        if img_array.ndim == 2: img_to_load = cv2.cvtColor(img_array, cv2.COLOR_GRAY2BGR)
                        elif img_array.ndim == 3: img_to_load = img_array
                        
                        if img_to_load is not None and img_to_load.dtype != np.uint8:
                            if img_to_load.max() <= 1.0 and img_to_load.min() >= 0.0:
                                img_to_load = (img_to_load * 255).astype(np.uint8)
                            else:
                                img_to_load = cv2.normalize(img_to_load, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    else:
                        img_to_load = cv2.imread(map_path)

                    if img_to_load is not None:
                        with image_lock:
                            current_image0 = img_to_load
                        dict_chon_ban_do['ten_ban_do'] = new_map_name
                        dict_chon_ban_do['update'] = 1 # Reset flag after processing
                        print(f"Server loaded new map: {dict_chon_ban_do}")
                        # Clear points and paths on server when map changes
                        update_ds_diem_duong = 1
                        danh_sach_diem = {}
                        danh_sach_duong = {}
                        loaded_successfully = True
                        update_img()
                        print(f"Server loaded new map: {new_map_name}. Points and paths cleared.")
                        return jsonify({"status": "success", "message": "Map update processed."}), 200
                    else:
                        message = f"Failed to read or process image file: {new_map_name}"
                except Exception as e:
                    message = f"Error loading map: {str(e)}"
            else:
                message = f"Map file not found: {new_map_name}"
            
            dict_chon_ban_do['update'] = 0 # Reset flag even on failure to prevent loop
            return jsonify({"status": "error", "message": message}), 400 if not loaded_successfully else 500
    return jsonify({"status": "error", "message": "Invalid map update data"}), 400

# --- AGV Adjustment Endpoints ---
@app.route('/confirm_agv_update', methods=['POST'])
def confirm_agv_update_route():
    global dict_dieu_chinh_vi_tri_agv
    data = request.get_json()
    
    if not (data and 'toa_do_x' in data and 'toa_do_y' in data and 'goc_agv' in data):
        return jsonify({"status": "error", "message": "Missing AGV coordinate/angle data"}), 400

    try:
        new_x = float(data['toa_do_x'])
        new_y = float(data['toa_do_y'])
        new_angle = float(data['goc_agv'])

        is_setup_action = int(data.get('setup', 0)) == 1
        is_confirm_action = int(data.get('update', 0)) == 1

        dict_dieu_chinh_vi_tri_agv['toa_do_x'] = new_x
        dict_dieu_chinh_vi_tri_agv['toa_do_y'] = new_y
        dict_dieu_chinh_vi_tri_agv['goc_agv'] = new_angle
        
        if is_setup_action:
            dict_dieu_chinh_vi_tri_agv['setup'] = 1
            # For a setup click, 'update' flag is not set to 1 by this action.
            # It will retain its current value or be 0 if not previously set.
            print(f"Server AGV position SET UP: X={new_x}, Y={new_y}, Angle={new_angle}, Setup=1")
            return jsonify({"status": "success", "message": "AGV position setup processed."}), 200
        
        elif is_confirm_action: # This comes from the "Cập nhật vị trí" button
            # dict_dieu_chinh_vi_tri_agv['update'] = 1 # Signal main.py/process_lidar to process this
            dict_dieu_chinh_vi_tri_agv['setup'] = 0  # This is a confirmation, so setup mode is off
            print(f"Server AGV position CONFIRMED for update: X={new_x}, Y={new_y}, Angle={new_angle}, Update=1, Setup=0")
            # The 'update' flag will be reset by process_lidar.py after consumption.
            return jsonify({"status": "success", "message": "AGV position update confirmed."}), 200
        
        else:
            # Trường hợp này là khi setup=0 và update=0, tức là thoát khỏi chế độ setup.
            dict_dieu_chinh_vi_tri_agv['setup'] = 0
            print(f"Server AGV position data updated (setup mode off): X={new_x}, Y={new_y}, Angle={new_angle}")
            return jsonify({"status": "success", "message": "AGV position data updated (setup mode off)."}), 200

    except ValueError:
        return jsonify({"status": "error", "message": "Invalid data type for AGV position."}), 400
    except Exception as e:
        print(f"Error in /confirm_agv_update: {e}")
        return jsonify({"status": "error", "message": f"Internal server error: {e}"}), 500

# --- Point Management Endpoints ---
# Server expects: {"name": "P1", "x": 100, "y": 200, "type": "có hướng", "angle": 90}
@app.route('/add_point', methods=['POST'])
def add_point_route():
    global danh_sach_diem, update_ds_diem_duong
    data = request.get_json()
    point_name = data.get('name')
    if point_name and 'x' in data and 'y' in data and 'type' in data and 'angle' in data:
        if point_name in danh_sach_diem:
            return jsonify({"status": "error", "message": f"Point name '{point_name}' already exists."}), 409
        
        danh_sach_diem[point_name] = [
            int(data['x']), int(data['y']),
            data['type'], float(data['angle'])
        ]
        print(f"Server added point: {point_name} -> {danh_sach_diem[point_name]}")
        update_ds_diem_duong = 1
        return jsonify({"status": "success", "message": "Point added."}), 201
    return jsonify({"status": "error", "message": "Invalid point data for add"}), 400

# Server expects: {"old_name": "P1", "name": "P1_new", "x": 110, ...} or if no rename: {"name": "P1", "x":110, ...}
@app.route('/update_point', methods=['POST'])
def update_point_route():
    global danh_sach_diem, danh_sach_duong, update_ds_diem_duong
    data = request.get_json()
    point_name = data.get('name')
    old_name = data.get('old_name', point_name) # If old_name is provided, it's a rename

    if not (point_name and old_name and 'x' in data and 'y' in data and 'type' in data and 'angle' in data):
        return jsonify({"status": "error", "message": "Invalid point data for update"}), 400

    if old_name not in danh_sach_diem:
        return jsonify({"status": "error", "message": f"Point to update ('{old_name}') not found."}), 404
    
    if old_name != point_name and point_name in danh_sach_diem: # Renaming to an existing different point's name
        return jsonify({"status": "error", "message": f"New point name '{point_name}' already exists."}), 409

    new_point_data = [int(data['x']), int(data['y']), data['type'], float(data['angle'])]

    if old_name != point_name: # It's a rename
        danh_sach_diem.pop(old_name)
        # Update paths referencing the old point name
        affected_paths_new_names = {} # Store new path data for client if needed
        for path_id, points_in_path in list(danh_sach_duong.items()):
            p1_in_path, p2_in_path = points_in_path
            path_updated = False
            if p1_in_path == old_name:
                p1_in_path = point_name
                path_updated = True
            if p2_in_path == old_name:
                p2_in_path = point_name
                path_updated = True
            
            if path_updated:
                new_path_id = f"{p1_in_path}_{p2_in_path}"
                danh_sach_duong.pop(path_id) # Remove old path
                danh_sach_duong[new_path_id] = [p1_in_path, p2_in_path] # Add new/renamed path
                affected_paths_new_names[path_id] = new_path_id
                print(f"Server updated path '{path_id}' to '{new_path_id}' due to point rename.")
    
    danh_sach_diem[point_name] = new_point_data
    update_ds_diem_duong = 1
    # --- Draw points and paths on current_image ---
    # update_img()
    print(f"Server updated point: {old_name} -> {point_name} Data: {new_point_data}")
    return jsonify({"status": "success", "message": "Point updated.", "new_name": point_name, "affected_paths": affected_paths_new_names if old_name != point_name else {}}), 200


@app.route('/delete_point', methods=['POST'])
def delete_point_route():
    global danh_sach_diem, danh_sach_duong, update_ds_diem_duong
    data = request.get_json()
    point_name = data.get('name')
    if point_name:
        if point_name in danh_sach_diem:
            del danh_sach_diem[point_name]
            # Cascade delete paths connected to this point
            paths_to_delete = [path_id for path_id, points in danh_sach_duong.items()
                               if point_name in points]
            for path_id in paths_to_delete:
                del danh_sach_duong[path_id]
                print(f"Server cascade deleted path: {path_id}")
            print(f"Server deleted point: {point_name}")
            
            # update_img()
            update_ds_diem_duong = 1
            return jsonify({"status": "success", "message": "Point and associated paths deleted."}), 200
        return jsonify({"status": "error", "message": "Point not found."}), 404
    return jsonify({"status": "error", "message": "Invalid point name for deletion"}), 400

# --- Path Management Endpoints ---
# Server expects: {"name": "P1_P2", "p1_name": "P1", "p2_name": "P2"}
@app.route('/add_path', methods=['POST'])
def add_path_route():
    global danh_sach_duong, update_ds_diem_duong
    data = request.get_json()
    path_name = data.get('name')
    p1_name = data.get('p1_name')
    p2_name = data.get('p2_name')

    if path_name and p1_name and p2_name:
        if path_name in danh_sach_duong or f"{p2_name}_{p1_name}" in danh_sach_duong :
            return jsonify({"status": "error", "message": f"Path '{path_name}' or its reverse already exists."}), 409
        if p1_name not in danh_sach_diem or p2_name not in danh_sach_diem:
            return jsonify({"status": "error", "message": "One or both points for the path do not exist."}), 400
            
        danh_sach_duong[path_name] = [p1_name, p2_name]
        print(f"Server added path: {path_name} -> {danh_sach_duong[path_name]}")
        # update_img()
        update_ds_diem_duong = 1
        return jsonify({"status": "success", "message": "Path added."}), 201
    return jsonify({"status": "error", "message": "Invalid path data for add"}), 400

@app.route('/delete_path', methods=['POST'])
def delete_path_route():
    global danh_sach_duong, update_ds_diem_duong
    data = request.get_json()
    path_name = data.get('name')
    if path_name:
        if path_name in danh_sach_duong:
            del danh_sach_duong[path_name]
            print(f"Server deleted path: {path_name}")
            # --- Draw points and paths on current_image ---
            # update_img()
            update_ds_diem_duong = 1
            return jsonify({"status": "success", "message": "Path deleted."}), 200
        return jsonify({"status": "error", "message": "Path not found."}), 404
    return jsonify({"status": "error", "message": "Invalid path name for deletion"}), 400

@app.route('/update_paths_after_point_rename', methods=['POST'])
def update_paths_after_point_rename_route():
    # This endpoint is primarily for server-side logging or complex validation if needed.
    # The client-side JS already handles the logic for updating path names and re-linking.
    # The server's danh_sach_duong is updated during the point rename itself.
    data = request.get_json()
    print(f"Server received path update info after point rename: {data}")
    # Potentially, here you could do more complex server-side validation or updates
    # if the client-side logic wasn't comprehensive enough for the server's state.
    # For now, assume the point rename on server already handled path adjustments.
    return jsonify({"status": "success", "message": "Path rename info acknowledged by server."})


# --- Save/Load Endpoints ---
@app.route('/save_points', methods=['POST'])
def save_points_route():
    global danh_sach_diem # Use server's current state
    data = request.get_json()
    filename = data.get('filename')
    # points_data_from_client = data.get('points_data') # This is what client sends

    if not filename:
        return jsonify({"status": "error", "message": "Filename not provided."}), 400
    
    # Server saves its own `danh_sach_diem`
    filepath = os.path.join(PATH_POINTS_DIR, f"{filename}.json")
    try:
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(danh_sach_diem, f, indent=4, ensure_ascii=False)
        print(f"Points saved to {filepath} from server's danh_sach_diem")
        return jsonify({"status": "success", "message": "Points saved."}), 200
    except Exception as e:
        return jsonify({"status": "error", "message": f"Error saving points: {str(e)}"}), 500

@app.route('/load_points/<filename>')
def load_points_route(filename): # Change parameter name to match the variable rule
    global danh_sach_diem, danh_sach_duong, file_diem_da_chon, update_ds_diem_duong # Keep globals
    filename_base = filename.replace(".json", "") # Use a new variable name for the base filename
    file_diem_da_chon = filename_base
    filepath = os.path.join(PATH_POINTS_DIR, f"{filename_base}.json") # Use filename_base here
    print(filepath, "------------")
    if not os.path.exists(filepath):
        return jsonify({"status": "error", "message": f"File '{filename}.json' not found."}), 404
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            loaded_points = json.load(f)
        danh_sach_diem = loaded_points
        danh_sach_duong = {} # Clear paths when loading points, as they might be inconsistent
        print(f"Points loaded from {filepath}. Paths cleared.")
        # update_img()
        update_ds_diem_duong = 1
        print("danh_sach_diem: ", danh_sach_diem)
        return jsonify({"status": "success", "points_data": danh_sach_diem}), 200
    except Exception as e:
        return jsonify({"status": "error", "message": f"Error loading points: {str(e)}"}), 500

@app.route('/save_paths', methods=['POST'])
def save_paths_route():
    global danh_sach_duong # Use server's current state
    data = request.get_json()
    filename = data.get('filename')
    # paths_data_from_client = data.get('paths_data')

    if not filename:
        return jsonify({"status": "error", "message": "Filename not provided."}), 400
    filepath = os.path.join(PATH_PATHS_DIR, f"{filename}.json")
    try:
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(danh_sach_duong, f, indent=4, ensure_ascii=False)
        print(f"Paths saved to {filepath} from server's danh_sach_duong")
        return jsonify({"status": "success", "message": "Paths saved."}), 200
    except Exception as e:
        return jsonify({"status": "error", "message": f"Error saving paths: {str(e)}"}), 500

@app.route('/load_paths/<filename_with_ext>')
def load_paths_route(filename_with_ext):
    global danh_sach_duong, file_duong_da_chon, update_ds_diem_duong
    filename = filename_with_ext.replace(".json", "")
    file_duong_da_chon = filename
    filepath = os.path.join(PATH_PATHS_DIR, f"{filename}.json")
    if not os.path.exists(filepath):
        return jsonify({"status": "error", "message": f"File '{filename}.json' not found."}), 404
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            loaded_paths = json.load(f)
        
        # Validate if points for these paths exist in current danh_sach_diem
        valid_paths = {}
        for path_name, (p1_name, p2_name) in loaded_paths.items():
            if p1_name in danh_sach_diem and p2_name in danh_sach_diem:
                valid_paths[path_name] = [p1_name, p2_name]
            else:
                print(f"Warning: Path '{path_name}' skipped during load. Point(s) '{p1_name}' or '{p2_name}' not in current point list.")
        
        danh_sach_duong = valid_paths
        print(f"Paths loaded from {filepath}. Invalid paths skipped.")
        # update_img()
        update_ds_diem_duong = 1
        print("danh_sach_duong: ", danh_sach_duong)
        return jsonify({"status": "success", "paths_data": danh_sach_duong}), 200
    except Exception as e:
        return jsonify({"status": "error", "message": f"Error loading paths: {str(e)}"}), 500

# --- NEW FUNCTION: convert_color_name_to_bgr ---
def convert_color_name_to_bgr(color_name):
    """Converts a color name string to an OpenCV BGR tuple."""
    colors = {
        "red": (0, 0, 255),
        "green": (0, 255, 0),
        "blue": (255, 0, 0),
        "yellow": (0, 255, 255),
        "cyan": (255, 255, 0),
        "magenta": (255, 0, 255),
        "white": (255, 255, 255),
        "black": (0, 0, 0),
        "gray": (128, 128, 128),
        "orange": (0, 165, 255),
        "purple": (128, 0, 128)
    }
    return colors.get(color_name.lower(), (0, 0, 0)) # Default to black if color not found
def draw_centered_text_on_square(image, text, x1, y1, x2, y2, font_scale=0.5, font_thickness=1, font=cv2.FONT_HERSHEY_SIMPLEX, text_color=(0, 0, 255)):
    """
    Vẽ văn bản lên một khung vuông và căn giữa chữ trong khung.

    Args:
        text (str): Văn bản muốn viết.
        square_size (int): Kích thước cạnh của khung vuông (ví dụ: 300 cho 300x300 pixel).
        font_scale (float): Tỉ lệ kích thước font.
        font_thickness (int): Độ dày nét chữ.
        font (int): Kiểu font từ cv2 (ví dụ: cv2.FONT_HERSHEY_SIMPLEX).
        text_color (tuple): Màu của chữ dưới dạng BGR (ví dụ: (0, 0, 255) cho màu đỏ).
        background_color (tuple): Màu nền của khung vuông (ví dụ: (255, 255, 255) cho màu trắng).

    Returns:
        np.array: Hình ảnh khung vuông với văn bản đã được vẽ.
    """
    # 2. Tính toán kích thước của văn bản
    # getTextSize trả về (width, height) của văn bản và baseline (đường cơ sở)
    text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
    text_width, text_height = text_size[0], text_size[1]

    # 3. Tính toán tọa độ để căn giữa văn bản
    # Tâm của khung vuông
    center_x = int(x2-x1) // 3
    center_y = int(y2-y1) // 2

    # Tọa độ góc trên bên trái của văn bản để nó được căn giữa
    # (x_start, y_start) của cv2.putText là góc dưới bên trái của văn bản
    # Do đó, cần điều chỉnh y_start để nó là đường cơ sở
    x_start = center_x
    y_start = center_y

    # Đảm bảo tọa độ không nằm ngoài ảnh
    if x_start < 0: x_start = 0
    if y_start < text_height: y_start = text_height # Đảm bảo y_start không quá nhỏ
    print(x1,x_start, y1,y_start)
    # 4. Viết chữ lên khung vuông
    cv2.putText(image, text, (int(x1+x_start), int(y1+y_start)), font, font_scale, text_color, font_thickness, cv2.LINE_AA)

    return image
def update_img():
    global current_image, current_image0, paint_dict_data_grid, update_ds_diem_duong
    # --- Draw points and paths on current_image ---
    with image_lock:
        img_to_draw_on = current_image0.copy() # Start with the base map

        # --- NEW: Draw grid data if enabled ---
        if paint_dict_data_grid:
            overlay = img_to_draw_on.copy()
            alpha = 0.3 # Transparency factor for the rectangles

            for grid_name, grid_data in dict_data_grid.items():
                vi_tri = grid_data.get("vi_tri")
                mau_str = grid_data.get("mau", "gray") # Default to gray if color not specified
                name = grid_data.get("name")
                diem = grid_data.get("diem") # Get diem

                if vi_tri and len(vi_tri) == 4:
                    x1, y1, x2, y2 = vi_tri
                    number = 2
                    color_bgr = convert_color_name_to_bgr(mau_str)
                    
                    # Draw a filled rectangle on the overlay
                    cv2.rectangle(overlay, (x1+number, y1+number), (x2-number, y2-number), color_bgr, -1)
                    print("ggg")
                    
                    # Draw the grid name
                    x_text_pos = int(x1 + (x2 - x1) * 0.1) # Adjust text position
                    y_text_pos = int(y1 + (y2 - y1) * 0.2) # Adjust text position
                    cv2.putText(img_to_draw_on, name, (x_text_pos, y_text_pos),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 0, 255), 1, cv2.LINE_AA) # Magenta text

                    # Draw the 'diem' point if it exists
                    if diem and len(diem) == 2:
                        diemX, diemY = diem
                        # Draw a small circle for the 'diem'
                        cv2.circle(img_to_draw_on, (int(diemX), int(diemY)), 2, (0, 0, 0), -1) # Black circle, radius 5
                        cv2.putText(img_to_draw_on, name, (int(diemX) + 8, int(diemY) + 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.2, (0, 0, 0), 1, cv2.LINE_AA) # Black text for diem coords
            
            # Blend the overlay with the main image
            img_to_draw_on = cv2.addWeighted(overlay, alpha, img_to_draw_on, 1 - alpha, 0)
        # --- END NEW: Draw grid data ---

        # # Draw all paths from danh_sach_duong
        # for path_id, (p1_name, p2_name) in danh_sach_duong.items():
        #     if p1_name in danh_sach_diem and p2_name in danh_sach_diem:
        #         p1_coords = (int(danh_sach_diem[p1_name][0]), int(danh_sach_diem[p1_name][1]))
        #         p2_coords = (int(danh_sach_diem[p2_name][0]), int(danh_sach_diem[p2_name][1]))
        #         cv2.line(img_to_draw_on, p1_coords, p2_coords, (0, 255, 255), 2) # BGR: Yellow line, thickness 2
        # # Draw all paths from danh_sach_duong
        # for path_id, (p1_name, p2_name) in danh_sach_duong.items():
        #     if p1_name in danh_sach_diem and p2_name in danh_sach_diem:
        #         p1_coords = (int(danh_sach_diem[p1_name][0]), int(danh_sach_diem[p1_name][1]))
        #         p2_coords = (int(danh_sach_diem[p2_name][0]), int(danh_sach_diem[p2_name][1]))
        #         # Calculate midpoint for path name
        #         mid_x = (p1_coords[0] + p2_coords[0]) // 2
        #         mid_y = (p1_coords[1] + p2_coords[1]) // 2
        #         cv2.putText(img_to_draw_on, path_id, (mid_x, mid_y), # Position text slightly above midpoint
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 255), 1, cv2.LINE_AA) # Cyan text
                
        #  # Draw all points from danh_sach_diem
        # for name, p_data in danh_sach_diem.items():
        #     px, py, p_type, p_angle = p_data
        #     # Draw point (e.g., a red circle)
        #     cv2.circle(img_to_draw_on, (int(px), int(py)), 3, (255, 0, 0), -1) # BGR: Red, radius 8
        #     # Draw point name
        #     cv2.putText(img_to_draw_on, name, (int(px) + 3, int(py) + 3),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 255), 1, cv2.LINE_AA) # Magenta text

        #     # Nếu điểm là "có hướng", vẽ thêm mũi tên
        #     if p_type == "có hướng":
        #         arrow_length = 30  # Chiều dài mũi tên (pixel)
        #         # Giả định p_angle là độ, 0 độ hướng sang phải, tăng dần theo chiều kim đồng hồ
        #         angle_rad = math.radians(p_angle)
        #         end_x = int(px + arrow_length * math.cos(angle_rad))
        #         end_y = int(py + arrow_length * math.sin(angle_rad))
        #         cv2.arrowedLine(img_to_draw_on, (int(px), int(py)), (end_x, end_y), (0, 255, 0), 2, tipLength=0.3) # Mũi tên màu xanh lá
        update_ds_diem_duong = 1
        current_image = img_to_draw_on # Update the global image to be served
        print(f"Kích thước ảnh gửi lên web (cao, rộng, kênh): {current_image.shape}")
@app.route('/get_saved_file_lists')
def get_saved_file_lists_route():
    point_files = get_saved_lists(PATH_POINTS_DIR)
    path_files = get_saved_lists(PATH_PATHS_DIR)
    grid_files = get_saved_lists(PATH_GRID_LISTS_DIR)
    return jsonify({
        "status": "success",
        "point_lists": point_files,
        "path_lists": path_files,
        "grid_lists": grid_files
    })

@app.route('/get_current_state')
def get_current_state_route():
    global danh_sach_diem, danh_sach_duong
    global dict_data_grid
    print(f"Server sending current state. Points count: {len(danh_sach_diem)}, Paths count: {len(danh_sach_duong)}")
    print(danh_sach_diem, danh_sach_duong)
    return jsonify({
        "status": "success",
        "points_data": danh_sach_diem,
        "paths_data": danh_sach_duong,
        "update_points_paths": update_ds_diem_duong,
    })


# --- Signal Communication Endpoints ---
@app.route('/PC_sent_AGV', methods=['POST'])
def pc_sent_agv_endpoint():
    global tin_hieu_nhan, thoi_gian_nhan_str, tin_hieu_gui
    data = request.get_json()
    if data:
        tin_hieu_nhan = data
        thoi_gian_nhan_str = time.strftime("%Y-%m-%d %H:%M:%S")
        
        # tin_hieu_nhan = {
                    #     "agv1": {"vi_tri_hien_tai": "P1", "dich_den": "P1", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
                    #     "agv2": {"vi_tri_hien_tai": "P2", "dich_den": "P2", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
                    #     "agv3": {"vi_tri_hien_tai": "P3", "dich_den": "P3", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
                    #     "agv4": {"vi_tri_hien_tai": "P4", "dich_den": "P4", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
                    #     "agv5": {"vi_tri_hien_tai": "P5", "dich_den": "P5", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
                    #     "agv6": {"vi_tri_hien_tai": "P6", "dich_den": "P6", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []},
                    #     "agv7": {"vi_tri_hien_tai": "P7", "dich_den": "P7", "trang_thai": "run", "message": "None", "danh_sach_duong_di": []}
                    # } # test
        # tin_hieu_gui = {'agv1': {"vi_tri_hien_tai": "P1", 'dich_den': "P1", 'trang_thai': 'run', "message": "None","danh_danh_duong_di":[]}}

        tin_hieu_gui[name_agv]["dich_den"] = tin_hieu_nhan[name_agv]["dich_den"]
        tin_hieu_gui[name_agv]["trang_thai"] = tin_hieu_nhan[name_agv]["trang_thai"]
        tin_hieu_gui[name_agv]["danh_danh_duong_di"] = danh_sach_duong_di
        tin_hieu_gui = {name_agv: {"vi_tri_hien_tai": danh_sach_duong_di, 
                           'dich_den': dich_den, 
                           'trang_thai': trang_thai, 
                           "message": message_agv,
                           "danh_sach_duong_di": danh_sach_duong_di}}

        print(f"tín hiệu nhận: {tin_hieu_nhan} at {thoi_gian_nhan_str}")
        print(f"tín hiệu gửi: {tin_hieu_gui} at {thoi_gian_nhan_str}")

        log_communication("nhan", thoi_gian_nhan_str, tin_hieu_nhan)
        log_communication("gui", thoi_gian_nhan_str, tin_hieu_gui)
        return jsonify({"status": "success", "data": tin_hieu_gui}), 200
    return jsonify({"status": "error", "message": "Invalid signal data. Expecting {'signal': 'your_string'}."}), 400


def bresenham_line(x0, y0, x1, y1):
    """Thuật toán Bresenham (phiên bản an toàn hơn) để tìm các điểm trên đường thẳng."""
    points = []
    x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
    
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    
    # Sử dụng biến cục bộ để không thay đổi tham số đầu vào
    x, y = x0, y0
    
    if dx > dy:
        err = dx // 2
        while x != x1:
            points.append([x, y])
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy // 2
        while y != y1:
            points.append([x, y])
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
            
    points.append([x1, y1]) # Thêm điểm cuối cùng
    return points

# Endpoint để client lấy thông tin vị trí AGV
@app.route('/get_agv_state', methods=['GET'])
def get_agv_state_route():
    global dict_dieu_chinh_vi_tri_agv, points_color_blue, points_color_red, image_lock, update_ds_diem_duong, danh_sach_diem, danh_sach_duong
    agv_body_coords_list = []
    agv_arrow_coords_list = []
    blue_points_list = []
    red_points_list = []
    with image_lock: # Đảm bảo an toàn luồng khi đọc các biến này
        center_x_map = float(dict_dieu_chinh_vi_tri_agv['toa_do_x'])
        center_y_map = float(dict_dieu_chinh_vi_tri_agv['toa_do_y'])
        angle_deg = float(dict_dieu_chinh_vi_tri_agv['goc_agv']) - 90
        angle_rad = math.radians(angle_deg)

        # AGV Body (Rectangle) - Define local coords then rotate and translate
        rect_width_local = 30  # pixels
        rect_height_local = 20 # pixels
        
        hw = rect_width_local / 2 # half-width
        hh = rect_height_local / 2 # half-height
        local_body_corners_np  = np.array([
            [-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]
        ], dtype=np.float32) # Use float32 for cv2.pointPolygonTest


        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        rotation_m = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

        rotated_local_body_corners = np.dot(local_body_corners_np, rotation_m.T)
        final_body_corners_map_space = rotated_local_body_corners + np.array([center_x_map, center_y_map])

        # Lấy các điểm bao bên ngoài và điểm tâm cho thân AGV
        temp_body_coords_list = []
        
        # Thêm điểm tâm
        temp_body_coords_list.append([int(center_x_map), int(center_y_map)])

        # Lấy các điểm trên đường viền bằng thuật toán Bresenham
        corners = final_body_corners_map_space
        num_corners = len(corners)
        for i in range(num_corners):
            p1 = corners[i]
            p2 = corners[(i + 1) % num_corners]
            line_points = bresenham_line(p1[0], p1[1], p2[0], p2[1])
            temp_body_coords_list.extend(line_points)

        agv_body_coords_list = temp_body_coords_list

        # AGV Arrow (Triangle) - Define local coords then rotate and translate
        arrow_shaft_len_local = 25 
        arrow_head_len_local = 10
        arrow_head_width_local = 12

        p_tip_local = np.array([arrow_shaft_len_local + arrow_head_len_local, 0])
        p_base1_local = np.array([arrow_shaft_len_local, -arrow_head_width_local / 2])
        p_base2_local = np.array([arrow_shaft_len_local, arrow_head_width_local / 2])
        local_arrow_vertices_np = np.array([p_tip_local, p_base1_local, p_base2_local], dtype=np.float32)

        rotated_local_arrow_vertices = np.dot(local_arrow_vertices_np, rotation_m.T)
        final_arrow_vertices_map_space = rotated_local_arrow_vertices + np.array([center_x_map, center_y_map])

        # Rasterize the AGV arrow triangle
        min_x_arrow = int(np.floor(np.min(final_arrow_vertices_map_space[:, 0])))
        max_x_arrow = int(np.ceil(np.max(final_arrow_vertices_map_space[:, 0])))
        min_y_arrow = int(np.floor(np.min(final_arrow_vertices_map_space[:, 1])))
        max_y_arrow = int(np.ceil(np.max(final_arrow_vertices_map_space[:, 1])))

        contour_arrow_for_test = final_arrow_vertices_map_space.astype(np.float32)

        temp_arrow_coords_list = []
        for x_coord_test in range(min_x_arrow, max_x_arrow + 1):
            for y_coord_test in range(min_y_arrow, max_y_arrow + 1):
                if cv2.pointPolygonTest(contour_arrow_for_test, (float(x_coord_test), float(y_coord_test)), False) >= 0:
                    temp_arrow_coords_list.append([x_coord_test, y_coord_test])
        agv_arrow_coords_list = temp_arrow_coords_list

        # # Chuyển đổi NumPy arrays thành list để có thể serialize JSON
        # if isinstance(points_color_blue, np.ndarray):
        #     blue_points_list = points_color_blue.tolist()
        # else: # Nếu là list, đảm bảo nó là list các list (cho JSON)
        #     blue_points_list = [list(p) for p in points_color_blue] if points_color_blue else []

        # if isinstance(points_color_red, np.ndarray):
        #     red_points_list = points_color_red.tolist()
        # else: # Nếu là list, đảm bảo nó là list các list
        #     red_points_list = [list(p) for p in points_color_red] if points_color_red else []

    # response = {
    #     "status": "success",
    #     "agv_body_coords": agv_body_coords_list,
    #     "agv_arrow_coords": agv_arrow_coords_list,
    #     "points_blue": blue_points_list,
    #     "points_red": red_points_list,
    #     "update_points_paths": update_ds_diem_duong,
    #     "points": danh_sach_diem,
    #     "paths": danh_sach_duong
    # }

    response = {
        "status": "success",
        "agv_body_coords": agv_body_coords_list,
        "agv_arrow_coords": agv_arrow_coords_list,
        "update_points_paths": update_ds_diem_duong,
        "points": danh_sach_diem,
        "paths": danh_sach_duong
    }

    # Sau khi gửi thì reset cờ để không vẽ lại liên tục
    update_ds_diem_duong = 0

    return jsonify(response),200


@app.route('/api/toggle_run_stop', methods=['POST'])
def toggle_run_stop_route():
    global run_and_stop
    if run_and_stop == 1:
        run_and_stop = 0
        return_run_stop = "stop"
    else:
        run_and_stop = 1
        return_run_stop = "run"
    print(f"Run/Stop state changed to: {run_and_stop}")
    return jsonify({"status": "success", "currentState": return_run_stop})

@app.route('/api/get_run_stop_status', methods=['GET'])
def get_run_stop_status_route():
    global run_and_stop
    return jsonify({"status": str(run_and_stop)})

@app.route('/api/toggle_speaker', methods=['POST'])
def toggle_speaker_route():
    global open_loa
    open_loa = 1
    print(f"Biến open_loa đã được đặt thành: {open_loa}")
    return jsonify({"status": "success", "message": "Yêu cầu bật loa đã được xử lý."})




    
# --- Luồng cập nhật vị trí AGV tự động ---
def auto_update_agv_position_task():
    global dict_dieu_chinh_vi_tri_agv, image_lock, IMG_WIDTH, IMG_HEIGHT
    global points_color_blue, points_color_red # Thêm global cho 2 list này
    print("AGV auto position update thread started.")
    increment_x = 50 # Giá trị tăng cho x mỗi lần
    increment_y = 30 # Giá trị tăng cho y mỗi lần
    while True:
        print("Auto updating AGV position...")
        time.sleep(0.5) # Đợi 2 giây
        with image_lock: # Sử dụng lock để đảm bảo an toàn khi truy cập dict
            dict_dieu_chinh_vi_tri_agv['toa_do_x'] = (dict_dieu_chinh_vi_tri_agv['toa_do_x'] + increment_x) % IMG_WIDTH
            dict_dieu_chinh_vi_tri_agv['toa_do_y'] = (dict_dieu_chinh_vi_tri_agv['toa_do_y'] + increment_y) % IMG_HEIGHT     
            dict_dieu_chinh_vi_tri_agv['goc_agv'] = (dict_dieu_chinh_vi_tri_agv['goc_agv'] + random.randint(5, 15)) % 360 # Thay đổi góc AGV       
            # Blue points: thêm một điểm ngẫu nhiên mới (tối đa 10 điểm)
            if len(points_color_blue) < 10:
                new_blue_point = [random.randint(0, IMG_WIDTH), random.randint(0, IMG_HEIGHT), random.randint(100, 500)]
                # Chuyển đổi points_color_blue thành list nếu nó là np.array để append
                if isinstance(points_color_blue, np.ndarray):
                    points_color_blue = points_color_blue.tolist()
                points_color_blue.append(new_blue_point)
            elif points_color_blue: # Nếu đã đủ 10, xóa điểm đầu tiên và thêm điểm mới
                if isinstance(points_color_blue, np.ndarray):
                    points_color_blue = points_color_blue.tolist()
                points_color_blue.pop(0)
                new_blue_point = [random.randint(0, IMG_WIDTH), random.randint(0, IMG_HEIGHT), random.randint(100, 500)]
                points_color_blue.append(new_blue_point)

            # Red points: thay đổi tọa độ của điểm đầu tiên nếu có
            # Chuyển points_color_red sang np.array để dễ thao tác nếu nó đang là list
            if isinstance(points_color_red, list):
                points_color_red_np = np.array(points_color_red)
            else:
                points_color_red_np = points_color_red

            if isinstance(points_color_red_np, np.ndarray) and points_color_red_np.ndim == 2 and points_color_red_np.shape[0] > 0 and points_color_red_np.shape[1] >=2 :
                points_color_red_np[0, 0] = (points_color_red_np[0, 0] + random.randint(-20, 20)) % IMG_WIDTH
                points_color_red_np[0, 1] = (points_color_red_np[0, 1] + random.randint(-20, 20)) % IMG_HEIGHT
                # Gán lại cho points_color_red nếu nó ban đầu là list
                if isinstance(points_color_red, list):
                    points_color_red = points_color_red_np.tolist()
            # print(f"Updated blue points: {points_color_blue}")
            # print(f"Updated red points: {points_color_red}")
            # update_img() # Vẽ lại ảnh nền với các điểm mới updateSpecialPoints

# host = "172.26.76.151"
host = "192.168.143.1"
port = 5000
url = "http://" + host + ":" + str(port)
if __name__ == '__main__':
    run_once = os.environ.get('WERKZEUG_RUN_MAIN') == 'true' or not app.debug

    opened_browser = False # Biến cờ để đảm bảo trình duyệt chỉ mở một lần

    # Mở trình duyệt một lần trong tiến trình ban đầu
    # WERKZEUG_RUN_MAIN không được đặt trong tiến trình ban đầu khi reloader hoạt động.
    if not os.environ.get("WERKZEUG_RUN_MAIN"):
        # url = "http://localhost:5000"
        webbrowser.open_new_tab(url)
        print(f"Opening {url} in your default browser...")
        opened_browser = True
    
    if run_once:
        initial_image_thread = threading.Thread(target=initial_image_setup_task)
        initial_image_thread.daemon = True
        initial_image_thread.start()
        initial_image_thread.join(timeout=5.0)
        if not image_initialized:
            print("Warning: Initial image setup took too long or failed. Proceeding with a fallback.")
            with image_lock: # Ensure some image exists
                if not np.any(current_image0) or current_image0.size == 0:
                    current_image0 = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
                    update_img()
            image_initialized = True
            
        # Khởi tạo và bắt đầu luồng cập nhật AGV tự động
        agv_updater_thread = threading.Thread(target=auto_update_agv_position_task)
        agv_updater_thread.daemon = True # Đảm bảo luồng này sẽ thoát khi chương trình chính thoát
        agv_updater_thread.start()

    # if not os.environ.get("WERKZEUG_RUN_MAIN"): # Open browser only in main Werkzeug process setInterval loadPointList drawPathLabel
    #     # webbrowser.open_new_tab("http://localhost:5000") # Can be annoying in dev             function loadCurrentState() {

    #     print("Flask server starting. Open http://localhost:5000 in your browser.") nextPointNumericId updateSpecialPoints

    app.run(debug=True, host=host, port=port, use_reloader=True)

    # modalOriginalPointName             function loadPointList() { deletePointFromModal
