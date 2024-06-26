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

### cruise controller output


https://github.com/IvLabs/ADAS/assets/119414628/d79c35f3-3a7e-492a-940b-980d75e76154
 
### Adaptive cruise controller output


https://github.com/IvLabs/ADAS/assets/119414628/f34b9a76-f4ad-4af1-a53f-06b5f02ef83a

### Pure Pursuit output



https://github.com/IvLabs/ADAS/assets/119414628/a42fede6-9653-486d-846d-d138ea49f1a9






