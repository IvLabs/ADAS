import numpy as np
from scipy.interpolate import make_interp_spline
import matplotlib.pyplot as plt 
from math import atan2,atan,sin,cos
import numpy as np

x = np.array([1, 2, 3, 4, 5, 6, 7])
y = np.array([20, 25, 10, 12, 39, 48, 50])
L=1

X_Y_Spline = make_interp_spline(x, y)
X_ = np.linspace(x.min(), x.max(), 500)
Y_ = X_Y_Spline(X_)
print(Y_)
plt.plot(X_, Y_)

opPosx,opPosy=(1,20)
n=1
xp,yp=[],[]

kdd=.5
ld_min=0.1
ld_max=0.5
speed=0.1

while n<len(Y_)-1:
    while (abs(opPosx-X_[n])>0.1) and (abs(opPosy-Y_[n])>0.1):
        print("s2")
        alpha=atan2(Y_[n]-opPosy,X_[n]-opPosx)
        ld = np.clip(kdd * speed, ld_min, ld_max)
        print(kdd*speed)
        delta=atan(2*L*sin(alpha)/ld)
        opPosx+=speed*cos(delta)
        opPosy+=speed*sin(delta)
        xp.append(opPosx)
        yp.append(opPosy)
        print((opPosx,opPosy),delta)
    n+=1
    print("n=",n)
    print(X_[n],Y_[n])

print("nnn")
plt.plot(xp, yp)   
plt.show()
    
    
