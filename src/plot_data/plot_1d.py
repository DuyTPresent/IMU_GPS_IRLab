import csv
import matplotlib.pyplot as plt

def read_csv_file(filename):
    timestamps, x_values, y_values = [], [], []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Bỏ qua dòng tiêu đề
        for row in reader:
            timestamps.append(float(row[0]))
            x_values.append(float(row[1]))
            y_values.append(float(row[2]))
    return timestamps, x_values, y_values

# Đọc dữ liệu từ tệp CSV cho GPS
#gps_timestamps, gps_x, gps_y = read_csv_file('/home/d/ros_irl_cpp/src/eskf_localization/gps.csv')

# Đọc dữ liệu từ tệp CSV cho dữ liệu đã fusion
fused_timestamps, fused_x, fused_y = read_csv_file('/home/d/ros_irl_cpp/src/eskf_localization/error_1.csv')

# Vẽ đồ thị 1D cho biến x theo thời gian
plt.figure(figsize=(10, 6))

# Vẽ dữ liệu từ GPS
#plt.plot(gps_timestamps, gps_x, label='GPS X', color='blue', linestyle='-')

# Vẽ dữ liệu đã fusion
plt.plot(fused_timestamps, fused_x, label='Fused GPS X', color='red', linestyle='--')

# Đặt tiêu đề và nhãn trục
plt.title('GPS vs. Fused Data (X Value)')
plt.xlabel('Timestamp')
plt.ylabel('X Value')

# Hiển thị chú thích
plt.legend()

# Hiển thị đồ thị
plt.show()
