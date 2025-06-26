
import csv
import matplotlib.pyplot as plt

# Mở file CSV và đọc dữ liệu
with open('/home/d/ros_irl_cpp/src/eskf_localization/error_1.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Bỏ qua dòng tiêu đề
    data = list(reader)

# Chuyển dữ liệu sang các list riêng lẻ cho các cột
# time = [float(row[0]) for row in data]
x_values = [float(row[1]) for row in data]
y_values = [float(row[2]) for row in data]
z_values = [float(row[3]) for row in data]



# Vẽ đồ thị
plt.figure(figsize=(10, 6))
plt.plot(x_values, label='Error_Position_X')
plt.plot(y_values, label='Error_Position_Y')
# plt.plot(z_values, label='Error_Position_Z')

plt.xlabel('Time')
plt.ylabel('Value_ERROR')
plt.title('Error Data')
plt.legend()
plt.grid(True)
plt.show()