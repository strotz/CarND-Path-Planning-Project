## path planning project

The goal of the project is to implement path planning module that safely navigates the vehicle in highway traffic. It includes dealing with vehicle speed and acceleration, changing lanes, avoiding collision with other vehicles.

### map of the world

The map of the highway is defines via collection of waypoints that allow us approximately convert from frenet (s, d) to cartesian (x, y) and back. Class world is defined to implement such operations. 

Due to the cyclic nature of the map, S coordinate has a maximum value. In order to ensure continuation of the planning, as soon as car S coordinate reaches certain value, its and other vehicles coordinates shifted.

### project genesis

#### timing

In the beginning, I've implemented "keep lane" car behavior with 2 timing profiles (constant speed, acceleration). Acceleration profile had different duration based on current and target speed. Such approach has nice properties (smooth speed and acceleration, manageable jerk) since it covers complete operation. 

Yet, such operations could be relatively long and it reduces speed of system reaction. Sometimes other vehicles can shift lanes and place itself into already planned trajectory. So, later timing calculation were limited to chunks by same size (0.5 seconds). It reduced probability of collision with "cut-off" vehicles. 

#### trajectory

Due to the nature of the map waypoint model, constant D coordinate will not grantee movement without position jerks. As recommended in class, spline library is used for trajectory generation in (X,Y) space. Next sequence of operations implemented to generate model:
1. Originated sparse trajectory in frenet
2. Converted to absolute XY
3. Converted to car XY
4. Spline computed

To generate path from model. First, linear distance calculated by timing profile based on delay (each 20 milliseconds). Then linear distance 
  

        