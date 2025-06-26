
import csv
import matplotlib.pyplot as plt

# Mở file CSV và đọc dữ liệu
with open('/home/d/ros_irl_cpp/src/eskf_localization/gps.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Bỏ qua dòng tiêu đề
    data = list(reader)

# Mở file CSV và đọc dữ liệu
with open('/home/d/ros_irl_cpp/src/eskf_localization/state.csv', 'r') as file1:
    reader_1 = csv.reader(file1)
    next(reader_1)  # Bỏ qua dòng tiêu đề
    data_1 = list(reader_1)

# Chuyển dữ liệu sang các list riêng lẻ cho các cột
#time = [float(row[0]) for row in data]
#x_values = [float(row[1]) for row in data]
y_values = [float(row[2]) for row in data]
#z_values = [float(row[3]) for row in data]

#x_values_1 = [float(row[1]) for row in data_1]
y_values_1 = [float(row[2]) for row in data_1]

# Vẽ đồ thị
plt.figure(figsize=(10, 6))
#plt.plot(x_values, label='GPS')
plt.plot(y_values, label='GPS')
#plt.plot(time, z_values, label='Z')

#plt.plot(x_values_1, label='Fuse')
plt.plot(y_values_1, label='Fuse')


plt.xlabel('Time')
plt.ylabel('Value')
plt.title('IMU_GPS_FUSE')
plt.legend()
plt.grid(True)
plt.show()