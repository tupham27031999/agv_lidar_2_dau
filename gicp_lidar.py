import open3d as o3d
import numpy as np
import cv2
import time

# Hàm để chuyển đổi dữ liệu LIDAR thành PointCloud của Open3D
def lidar_to_point_cloud(points):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

# Hàm để thực hiện downsampling bằng Voxel Grid Filter
def downsample_point_cloud(point_cloud, voxel_size):
    return point_cloud.voxel_down_sample(voxel_size)

# Hàm để lấy các điểm trong points2 có khoảng cách đến điểm (x_goc, y_goc) nhỏ hơn distan
def filter_points(points, x_goc, y_goc, distan):
    distances = np.sqrt((points[:, 0] - x_goc)**2 + (points[:, 1] - y_goc)**2)
    mask = distances < distan
    return points[mask]

def draw_points_on_image(points, image, color):
    for point in points:
        cv2.circle(image, (int(point[0]), int(point[1])), 1, color, -1)


def gicp(points1, points2, threshold=10, trans_init=np.eye(4), radius=0.1, max_nn=30, max_iteration=200, voxel_size=1):
    ''' 
        voxel_size_old = 0.1 | đang test là 2: tạo 1 khung vuông 2x2 và gộp các điểm trong khung vuông
        max_iteration: số lần lặp tối đa
        max_nn + radius: số điểm tối đa max_nn nằm trong bán kính radius được xét (radius = 0.1(old) or 1)

        radius=1, max_nn=30, max_iteration=1000, voxel_size=2  --> tb 2.74
        radius=0.1, max_nn=30, max_iteration=1000, voxel_size=0.1 --> tb 2.79

        Tham số threshold chính là khoảng cách xa nhất mà một cặp điểm có thể được coi là một "cặp tương ứng hợp lệ" (inlier).
        có thể điều chỉnh threshold = 5
        
    '''
    # Chuyển đổi dữ liệu LIDAR thành PointCloud của Open3D
    source_pcd = lidar_to_point_cloud(points1)
    target_pcd = lidar_to_point_cloud(points2)

    # Thực hiện downsampling cho source_pcd và target_pcd
    source_pcd = downsample_point_cloud(source_pcd, voxel_size)
    target_pcd = downsample_point_cloud(target_pcd, voxel_size)


    # Tính toán các vector pháp tuyến cho PointCloud
    source_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
    target_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))

    # Tính toán các ma trận hiệp phương sai cho PointCloud
    source_pcd.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
    target_pcd.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))

    # Thiết lập các tham số cho GICP
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration)

    # Sử dụng GICP để căn chỉnh các điểm quét
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        criteria)
    
    # return reg_p2p.inlier_rmse, reg_p2p.transformation
    # In ra ma trận chuyển đổi
    transformation = reg_p2p.transformation

    rmse = reg_p2p.inlier_rmse
    # Trích xuất ma trận xoay và ma trận tịnh tiến
    r = transformation[:3, :3]
    t = transformation[:3, 3]

    
    
    # return rmse, r, t

def transform_points(points, rotation_matrix, translation_vector):
    """
    Hàm để xoay và tịnh tiến danh sách điểm.
    
    Parameters:
    points (numpy array): Danh sách điểm đầu vào (Nx3).
    rotation_matrix (numpy array): Ma trận xoay (3x3).
    translation_vector (numpy array): Ma trận tịnh tiến (3,).
    
    Returns:
    numpy array: Danh sách điểm đã xoay và tịnh tiến (Nx3).
    """
    # Chuyển đổi danh sách điểm thành numpy array nếu cần
    points = np.asarray(points)
    # Xoay và tịnh tiến các điểm
    transformed_points = np.dot(points, rotation_matrix.T) + translation_vector
    
    return transformed_points
def inverse_transform_points(points, rotation_matrix, translation_vector):
    """
    Hàm để xoay và tịnh tiến ngược danh sách điểm.
    
    Parameters:
    points (numpy array): Danh sách điểm đầu vào (Nx3).
    rotation_matrix (numpy array): Ma trận xoay (3x3).
    translation_vector (numpy array): Ma trận tịnh tiến (3,).
    
    Returns:
    numpy array: Danh sách điểm đã xoay và tịnh tiến ngược (Nx3).
    """
    points = np.asarray(points)
    # Xoay và tịnh tiến ngược các điểm
    inverse_transformed_points = np.dot(points - translation_vector, rotation_matrix)
    return inverse_transformed_points
# luu gia tri san saved_scans
def detect_loop_closure(current_scan, saved_scans, threshold = 1):
    for i, saved_scan in enumerate(saved_scans):
        rmse, r, t = gicp(saved_scan, current_scan)
        if rmse < threshold:
            return i, r, t
    return None, None, None

# def ket_hop_2_anh(img1,img2):
#     # Tạo ảnh 1 màu xám (150, 150, 150) với kênh alpha
#     img1 = np.full((img_height, img_width, 4), (150, 150, 150, 255), dtype=np.uint8)

#     # Thêm 100 điểm màu đen và trắng bất kỳ vào ảnh 1
#     for _ in range(50):
#         x, y = random.randint(0, img_width-1), random.randint(0, img_height-1)
#         img1[y, x] = (0, 0, 0, 255)  # Màu đen với kênh alpha
#     for _ in range(50):
#         x, y = random.randint(0, img_width-1), random.randint(0, img_height-1)
#         img1[y, x] = (255, 255, 255, 255)  # Màu trắng với kênh alpha

#     # Tạo ảnh 2 màu xám (150, 150, 150) với kênh alpha
#     img2 = np.full((img_height, img_width, 4), (150, 150, 150, 255), dtype=np.uint8)

#     # Thêm 100 điểm màu đen và trắng bất kỳ vào ảnh 2
#     for _ in range(50):
#         x, y = random.randint(0, img_width-1), random.randint(0, img_height-1)
#         img2[y, x] = (0, 0, 0, 255)  # Màu đen với kênh alpha
#     for _ in range(50):
#         x, y = random.randint(0, img_width-1), random.randint(0, img_height-1)
#         img2[y, x] = (255, 255, 255, 255)  # Màu trắng với kênh alpha

#     # Tạo ma trận xoay để xoay ảnh 2 góc 30 độ và tinh tiến (1, 2)
#     center = (img_width // 2, img_height // 2)
#     angle = 30
#     scale = 1.0
#     rotation_matrix = cv2.getRotationMatrix2D(center, angle, scale)
#     rotation_matrix[0, 2] += 1  # Tinh tiến theo trục x
#     rotation_matrix[1, 2] += 2  # Tinh tiến theo trục y

#     # Xoay và tinh tiến ảnh 2
#     img2_rotated = cv2.warpAffine(img2, rotation_matrix, (img_width, img_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(150, 150, 150, 255))

#     # Hợp nhất hai ảnh, ưu tiên màu đen
#     merged_img = np.where(np.all(img1[:, :, :3] == [0, 0, 0], axis=-1, keepdims=True), img1, img2_rotated)
#     merged_img = np.where(np.all(img2_rotated[:, :, :3] == [0, 0, 0], axis=-1, keepdims=True), img2_rotated, merged_img)