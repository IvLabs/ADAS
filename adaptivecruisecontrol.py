import matplotlib.pyplot as plt

safe_distance = 5
current_distance = 50
current_velocity =0
error = -safe_distance +current_distance
front_car_acc =-5
integral = 0
previous_error = 0
time = 0
time_step = 0.1
current_car_acceleration=0
Kp = 2
Ki = 0.5
Kd = 5

time_data = []
velocity_data = []
distance_data = []
error_data = []

sim = 60
while time <= sim:
    error = -safe_distance + current_distance
    integral += error * time_step
    derivative = (error - previous_error) / time_step

    control = Kp * error + Ki * integral + Kd * derivative
    current_car_acceleration=front_car_acc-control
   
    current_velocity += current_car_acceleration * time_step

    current_distance += current_velocity * time_step

    previous_error = error
    time += time_step

    time_data.append(time)
    velocity_data.append(current_velocity)
    distance_data.append(current_distance)
    error_data.append(error)
    




plt.subplot(3, 2, 1)
plt.plot(time_data, velocity_data)
plt.xlabel('time')
plt.ylabel('relative velocity')

plt.subplot(3, 2, 2)
plt.plot(time_data, distance_data)
plt.xlabel('time')
plt.ylabel('distance')

plt.subplot(3, 2, 3)
plt.plot(time_data, error_data)
plt.xlabel('time')
plt.ylabel('error')

plt.tight_layout()
plt.show()
