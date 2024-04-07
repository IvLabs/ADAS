import matplotlib.pyplot as plt

m = 1000
b = 10
setSpeed = 10
kp = 450
ki = 10
kd = 0


e = setSpeed
vel = 0
i = 0
e1 = setSpeed
v1 = 0
earr = []
varr = []

while abs(e) > 0.01 or abs(e1) > 0.01:
    e = setSpeed - vel
    i += e
    d = e - e1

    u = kp * e + kd * d + ki * i
    e1 = e
    print(e)

    vel = (u + m * vel) / (m + b)
    earr.append(e)
    varr.append(vel)

plt.plot(range(len(earr)), earr)
plt.xlabel("Time")
plt.ylabel("Error in Velocity")
plt.show()
