# FireBird-SolidBoost-25
# Background:
My design for Solid-Boost (Solid Rocketry Competition) held at Indian Institute of Technology - Banaras Hindu University (IIT-BHU) as part of their TechNex Tech festival

## Requirements for the design:
* Below 1kg
* F class motor (Upto 80N-s of thrust)
* 500m max apogee
* < 30m radius landing
# Design
## Structure
Fully 3d printed

## Propulsion
This rocket  designed to be used with simple KNO3 (Potassium Nitrate) + C12H22O12 (Sugar) in 65:35 ratio.

## Electronics
### Brains:
* Uses teensy 4.1 for sensor fusion and kalman filtering for maximum certainty
* write telemetry to a flash storage module
* Sends telemetry to an Arduino nano to write there for redundancy in telemetry at different frequencies for analysis

### Sensors:
* MPU9250 - for Inertial navigation + Gyroscope for orientation + Magentometer for earth orientation & positioning for waypoints guidance
* NEO-7M GPS - 

## Control
Uses a multiple phase approach:
* Powered Ascent - TVC gimbal (Pitch + Yaw) + Fins (Roll)
* Unpowered Ascent - Fins (Pitch, Yaw and Roll)
* Powered descent - Same as above
* Airbrake - Fins maximize AoA for slowing down
* deploy parachute at 30m

# Advantages:
* Very high controllability
* Very precise

# Disadvantages:
* Very complex code
* Is not a full working concept, needs a lot of iteration
* Better to start over on another one
* Too much 3d printing, more machines components would make it better
* Structural strength is absolute shite, not really airworthy
* Definitely a great learning experience


# Future improvements
* Actually implement a full Kalman filter for all states
* Use quaternions instead of Euler angles
* Make PCB for electronics (minimize weight)
* Implement forward control surfaces
* Make a larger motor
* Stick a camera on it
* Make a gimbal-able nozzle for the motor

# Outcome
* We were placed 2nd in the competition, beaten by a team of our own institution.
