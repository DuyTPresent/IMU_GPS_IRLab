import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Hàm để đọc dữ liệu từ tệp CSV
def read_csv_file(filename):
    timestamps, x_values, y_values, z_values = [], [], [], []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Bỏ qua dòng tiêu đề
        for row in reader:
            timestamps.append(float(row[0]))
            x_values.append(float(row[1]))
            y_values.append(float(row[2]))
            #z_values.append(float(row[3]))
    return timestamps, x_values, y_values, z_values

# Đọc dữ liệu từ tệp CSV
gps_timestamps, gps_x, gps_y, gps_z = read_csv_file('/home/d/ros_irl_cpp/src/eskf_localization/gps_1.csv')
fused_timestamps, fused_x, fused_y, fused_z = read_csv_file('/home/d/ros_irl_cpp/src/eskf_localization/state_1.csv')

# Tạo subplot 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Vẽ điểm từ dữ liệu GPS
ax.scatter(gps_x, gps_y, c='r', marker='*', label='GPS Data')
#ax.scatter(gps_x, gps_y, gps_z, c='r', marker='o', label='GPS Data')

# Vẽ điểm từ dữ liệu GPS đã fusion
ax.scatter(fused_x, fused_y, c='b', marker='*', label='Fused GPS Data')
#ax.scatter(fused_x, fused_y, fused_z, c='b', marker='o', label='Fused GPS Data')

# Đặt nhãn cho các trục
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
#ax.set_zlabel('Z Label')

# Thêm chú thích vào đồ thị
plt.legend()

# Hiển thị đồ thị
plt.show()
