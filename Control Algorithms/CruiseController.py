import matplotlib.pyplot as plt
import numpy as np



M = 1100  
drag_coefficient = 0.5  
setpoint_vel = 20

dt =1  
t_max = 90 # in sec

# Initializing variables 
time = np.arange(0, t_max, dt)
vel = np.zeros(len(time))
acceleration = np.zeros(len(time))

Kp = 320
Ki = 0.6
Kd = 0.8
# PID controller param(tuning)

error = np.zeros(len(time))   #at each time

integral_e = 0
derivative_e=0

# Looping for dt
for i in range(1, len(time)):
    error[i] = setpoint_vel - vel[i-1]
    
    integral_e += error[i] * dt
    derivative_e += (error[i] - error[i-1]) / dt

    engineforce  = Kp * error[i] + Ki * integral_e + Kd * derivative_e
    
    acceleration[i] =(engineforce /M) - ((drag_coefficient * vel[i-1]) /M)
  

    vel[i] = vel[i-1] + acceleration[i] * dt    # Updating velocity

rise_time = 0
for i in range(len(time)):
    if vel[i] >= 0.9 * setpoint_vel:
        rise_time = time[i]
        break

overshoot = ((max(vel) - setpoint_vel) / setpoint_vel) * 100

print(f"Rise time: {rise_time:}")
print(f"Overshoot: {overshoot:.2f} %")


plt.plot(time, vel)
plt.plot(time, setpoint_vel * np.ones(len(time)), label="Set Point vel")
plt.xlabel("Time(s)")
plt.ylabel("velocity(m/s)")
plt.legend()
plt.show()


