import matplotlib.pyplot as plt

dmax = 5
kp = 100
ki = 0.4
kd = 1000
b = 0.6
m = 1000

vel = 2
a = 0
s = 100

e = s - dmax
i = 0
d = 0


e1 = s - dmax
earr = []
sarr = []
varr = []

while abs(e) > 0.1 or abs(e1) > 0.1:
    e = s - dmax
    i += e
    d = e - e1

    u = kp * e + kd * d + ki * i

    if e > 100 or e < -100:
        break

    e1 = e
    print("Errore >>", e)
    print("Errord >>", d)
    print("Errori >>", i)

    a = (u - b * vel) / m
    s -= vel + (a / 2)
    vel = vel + a
    print("Vel >>", vel)
    print("acc >>", a)
    print("dis >>", s)
    earr.append(e)
    sarr.append(s)
    varr.append(vel)

plt.plot(range(len(sarr)), sarr)
plt.plot(range(len(varr)), varr)

plt.xlabel("Time")
plt.ylabel("Distance")
plt.show()
plt.plot(range(len(varr)),vel)
plt.xlabel("Time")
plt.ylabel("velovity")
plt.show()


