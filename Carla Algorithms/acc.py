import glob
import os
import sys
import numpy as np

import random
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
def main():
#loading the world
    client=carla.Client('localhost',2000)
    world=client.get_world()
    client.load_world('Town04')
    bp_lib=world.get_blueprint_library()
#spawning the ego vehicle
    vehicle_bp=bp_lib.find('vehicle.tesla.model3')
    transform=carla.Transform(carla.Location(x=-60, y=9.848137, z=11.667718),carla.Rotation(yaw=0))
    ego=world.spawn_actor(vehicle_bp,transform)
    ego.apply_control(carla.VehicleControl(throttle=0.3,steer=0.0))
#initializing the spectator
    spectator=world.get_spectator()
    transform=carla.Transform(ego.get_transform().transform(carla.Location(x=-4,z=3)),ego.get_transform().rotation)
    spectator.set_transform(transform)
#spawning the lead vehicle
    vehicle_bp1 = random.choice(bp_lib.filter('vehicle.*.*'))
    transform1=carla.Transform(carla.Location(x=-40, y=9.848137, z=11.667718),carla.Rotation(yaw=0))
    veh=world.try_spawn_actor(vehicle_bp1,transform1)
    veh.apply_control(carla.VehicleControl(throttle=0.5,steer=0.0))
    print(ego.get_transform().location)
#control loop
    mass=1000
    kp = 500
    ki = 10
    kd = 100
    safed=20
    vf=7
    t=range(0,101)
    vc=[]
    rd=[]
    e0=0
    e1=0
    esum=0
    t1=0.1
    for i in t:
        if i%5==0:
            t1=t1+0.05
            veh.apply_control(carla.VehicleControl(throttle=np.fmin(t1,1.0),steer=0.0))
        vel=ego.get_velocity()
        vc.append(vel.x)
        egox=ego.get_transform().location.x
        vehx=veh.get_transform().location.x
        dif=abs(vehx-egox)
        rd.append(dif)
        e1=rd[i]-safed
        esum=+e1

        p=kp*e1
        ine=ki*esum
        d=kd*(e1-e0)

        u=(p+ine+d)/mass
        if(u<0):
            ego.apply_control(carla.VehicleControl(throttle=0.0,steer=0.0))
        elif(u>6.8):
            ego.apply_control(carla.VehicleControl(throttle=1.0,steer=0.0))
        else:
            a=u/6.8
            ego.apply_control(carla.VehicleControl(throttle=a,steer=0.0))
        e0=e1
        time.sleep(0.5)
    print(rd)
if __name__ == '__main__':
    main()
