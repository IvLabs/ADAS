import glob
import os
import sys
import numpy as np
import math

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

def get_points(client, ego):
    tm = client.get_trafficmanager(8000)
    tm_port=tm.get_port()
    ego.set_autopilot(True,tm_port)
    tm.ignore_lights_percentage(ego,100)

    points=[]
    for i in range(21):
        loc=ego.get_location()
        points.append((loc.x,loc.y,loc.z))
        time.sleep(1)
    b=ego.destroy()
    a=np.array(points)
    return a

def pure_pursuit_control(ego,waypoints):
    L = 3
    Kdd = 10.0
    t=0
    ego.apply_control(carla.VehicleControl(throttle=0.3,steer=0.0))
    while t<20:
        loc=ego.get_location()
        vel=ego.get_velocity()
        vf = np.sqrt(vel.x**2 + vel.y**2)
        vf = np.fmax(np.fmin(vf, 2.5), 0.1)
        dxl, dyl = [], []
        for i in range(len(waypoints)):
            dx = abs(loc.x - waypoints[i][0])
            dxl.append(dx)
            dy = abs(loc.y - waypoints[i][1])
            dyl.append(dy)
        dist = np.hypot(dxl, dyl)
        idx = np.argmin(dist)+3
        if idx < len(waypoints):
            tx = waypoints[idx][0]
            ty = waypoints[idx][1]
        else:
            tx = waypoints[-1][0]
            ty = waypoints[-1][1]
        ld=Kdd*vf
        alpha=np.arctan2(ty-loc.y,tx-loc.x)-np.radians(ego.get_transform().rotation.yaw)
        delta = math.atan2(2*L*np.sin(alpha), ld)
        delta = np.fmax(np.fmin(delta, 1.0), -1.0)
        ego.apply_control(carla.VehicleControl(throttle=0.3,steer=delta))
        time.sleep(2)
        print(delta)
        t+=1
def main():
    print('hello carla world')
#loading the world
    client=carla.Client('localhost',2000)
    world=client.get_world()
    client.load_world('Town03')
    bp_lib=world.get_blueprint_library()
#spawning the ego vehicle
    vehicle_bp=bp_lib.find('vehicle.tesla.model3')
    spawn=world.get_map().get_spawn_points()
    eg=world.spawn_actor(vehicle_bp,spawn[0])
#initializing the spectator
    spectator=world.get_spectator()
    transform=carla.Transform(eg.get_transform().transform(carla.Location(x=-4,z=3)),eg.get_transform().rotation)
    spectator.set_transform(transform)
#enabling traffic manager & getting waypoints
    points=get_points(client,eg)
    print(points)
    
#control loop    
    ego=world.spawn_actor(vehicle_bp,spawn[0])
    pure_pursuit_control(ego,points)

    
    
    
if __name__=='__main__':
    main()
