# ADAS: Advanced Driving Assistance System_🚗_______

# Description

ADAS includes technologies that assist drivers with the safe operation of a vehicle. It uses automated technology, such as sensors and cameras, to detect nearby obstacles or driver errors, and respond accordingly.This project aims to develop the Autonomy with three main layers: Controls, Route Planning & Mapping, and Perception. 
Thus this system is designed to provide autonomous driving capabilities for vehicles.
![Screenshot 2024-04-07 174343](https://hackmd.io/_uploads/BJ5zAWelC.png)
In this project, we implemented a controller in Python and used it to drive a car autonomously around a track in Carla Simulator. The output of the controller will be the vehicle throttle, brake and steering angle commands. The throttle and brake come from the Longitudinal speed control and the steering comes from the Lateral Control.


## System Architecture

ADAS can enable various levels of autonomous driving.
Level 0 – NO Driving Automation. [Cruise Control]
Level 1 – Driver Assist. [Adaptive Cruise Control]
Level 2 – Partial Driving Automation [Lane Following]
The ADAS system follows a modular architecture, with each layer communicating through well-defined interfaces. The high-level architecture is depicted in the following diagram:


# 1 . Controls Layer

The Controls layer is responsible for the linear and lateral movement of the vehicle.

### Cruise Control and Adaptive Cruise Control

We have implemented a cruise control and adaptive cruise control system using a PID (Proportional-Integral-Derivative) controller for linear control. The PID controller adjusts the vehicle's speed based on the desired speed and the distance to the vehicle in front.

```python
# PID Controller for Linear Control
kp = 0.5  # Proportional gain
ki = 0.1  # Integral gain
kd = 0.2  # Derivative gain

def pid_control(target_speed, current_speed, dt):
    error = target_speed - current_speed
    integral += error * dt
    derivative = (error - previous_error) / dt
    control_output = kp * error + ki * integral + kd * derivative
    previous_error = error
    return control_output
```

#### Results for Cruise Control
>* Setting the car to maintain a constant speed without any human efforts.
>* Uses the PID controller for acceleration calculation.

 
Error v/s Time            |  Velocity v/s Time 
:-------------------------:|:-------------------------:
![image](https://hackmd.io/_uploads/r1gwoyXk0.png)  |  ![image](https://hackmd.io/_uploads/Hyv4hkQyC.png)
![73613341-3e26e400-45fd-11ea-8985-d059e340796e](https://hackmd.io/_uploads/rJCRHkleR.png)


#### Results for Adaptive Cruise Control
> Varies the speed to maintain a safe following distance and stay within speed limits using the PID controller

Error v/s Time            |  Velocity v/s Time 
:-------------------------:|:-------------------------:
![image](https://hackmd.io/_uploads/SJFLRyQJ0.png) |  ![image](https://hackmd.io/_uploads/Bysw0km10.png)


### Lateral Control and Lane Following

For lateral control and lane following, we have implemented a pure pursuit controller. This algorithm calculates the steering angle required to follow a desired path or trajectory.

```python
# Pure Pursuit Controller for Lateral Control
def pure_pursuit_control(vehicle_state, waypoints):
    look_ahead_distance = 10.0  # Distance to look ahead on the path
    closest_point, nearest_distance = get_closest_point_on_path(vehicle_state.position, waypoints)
    look_ahead_point = get_look_ahead_point(closest_point, waypoints, look_ahead_distance)
    steering_angle = get_steering_angle(vehicle_state, look_ahead_point)
    return steering_angle
```

These controllers have been tested and validated using the Carla simulation environment.

#### Results for Pure Pursuite
> * Tracks and Traces the path from selected waypoints.
>* Selects a point from a tunable look-ahead distance and calculates an appropriate steer angle.
>* Look-ahead distance depends on the car velocity.

![image](https://hackmd.io/_uploads/Hk-XVMgg0.png)


## Implementation on Carla


## Carla Simulator
![image](https://hackmd.io/_uploads/ryhbBl71R.png)

CARLA is an open-source autonomous driving simulator. It was built from scratch to serve as a modular and flexible API to address a range of tasks involved in the problem of autonomous driving, providing a realistic urban environment and high-fidelity physics simulation.
We have utilized the Carla Simulator for testing and validation of our ADAS system. 

For Installation procedure and detailed operations:

https://hackmd.io/@Shreyas321/B1p-Ha48a/edit



### Launching Carla

1. In the terminal, go to the `Carla_Simulator` folder.
2. Deactivate the Conda environment: `conda deactivate`
3. Run the shell file: `./CarlaUE4.sh`
4. The default Carla window will pop up on the screen.

### Simulation

1. Python APIs are used to perform simulations on Carla.
2. The `Carla_Simulator/pythonAPI/examples` folder contains example API scripts for better understanding of Carla.
3. To run an API script, open this folder in the terminal, deactivate the Conda environment, and run the `python name.py` command to execute the required API.

### PythonAPI

>These are the files used to manipulate Carla and perform simulations on it.

#### Basic Structure of API

##### Client

- We need to connect the Carla server to an open port. The client controls the simulator using the `client` and `world` objects.

```python
client = carla.Client('localhost', 2000)
world = client.get_world()
```

The `client` object serves to maintain the client's connection. We can set any available port; port 2000 is set as the default.

- We need a map for the simulation. The `world` object provides access to map elements.

```python
client.load_world('Town05')
```

- We can use a spectator view to observe the surroundings. We can use the mouse to control pitch and yaw, and QASD to control its movement.

```python
spect = world.get_spectator()
transform = spect.get_transform()
location = transform.location
spect.set_transform(transform1)  # This command will set the spectator to the given transform.
```

###### Spawning

- First, we have to take the blueprint library to load an actor.

```python
bplib = world.get_blueprint_library()
```

- In this library, we can find the actor we want.

```python
vehicle_bp = bplib.find('vehicle.tesla_model3')  # It will find the tesla_model3 in the library.
```

- Now, we require spawn points to spawn the vehicles.

```python
spawn_points = world.get_map().get_spawn_points()  # It will search for available locations to spawn vehicles on the map and give a list of spawn points.
```

- Now, we can spawn our vehicle in Carla.

```python
ego = world.spawn_actor(vehicle_bp, spawn_points[0])  # We can use world.try_spawn_actor() to spawn the vehicle without throwing an error if a collision occurs at the spawn points and continue running the script.
```

- To spawn other actors like sensors, we follow the same procedure, but we need to attach them to a vehicle.

```python
camera_bp = bplib.find('sensor.camera.rgb')
camera = world.spawn_actor(camera_bp, spawn_points[0], attach_to=ego)
```

###### Vehicle

Vehicles are a special type of actor in Carla. They incorporate special components that simulate the physics of wheeled vehicles. This is achieved by applying different types of controls.

- `carla.vehicle_control` provides input for driving commands such as throttle, brake, steer, etc.

```python
ego.apply_control(carla.VehicleControl(throttle=1, steer=0))
```

- We can set the vehicle to auto-pilot using the following command:

```python
ego.set_autopilot(True)
```

- Getter and setter functions:
    - `ego.get_transform` - Returns the transform object of the ego vehicle as output (location and orientation).
    - `ego.get_transform().location` - Gives the location of the ego vehicle from the transform object.
    - `ego.get_velocity()` - Gives the velocity of the vehicle in the x, y, z directions.
    - `ego.get_acceleration()` - Gives the acceleration of the vehicle.
    - `ego.set_target_velocity()` - Sets the car to the desired velocity.
    - `ego.set_location()` - Teleports the actor to the given location.
    - `ego.set_transform()` - Teleports the actor to the given transform (location + rotation).


### cruise controller output


https://github.com/IvLabs/ADAS/assets/119414628/d79c35f3-3a7e-492a-940b-980d75e76154
 
### Adaptive cruise controller output


https://github.com/IvLabs/ADAS/assets/119414628/f34b9a76-f4ad-4af1-a53f-06b5f02ef83a

### Pure Persuit output



https://github.com/IvLabs/ADAS/assets/119414628/a42fede6-9653-486d-846d-d138ea49f1a9






