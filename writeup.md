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
2. Converted to map XY
3. Converted to car XY
4. Spline computed

To generate path from model. First, distance in frenet coordinates calculated by timing profile based on delay from start of prediction (each 20 milliseconds). Then distance transferred to car X coordinate. Spline used to obtain Y component in car system. And finally transferred to map XY coordinates (see prediction::calculate_trajectory and trajectory::get_position_at).

#### it is all about future

Two things it is important to mention regarding this project. Result of the path planning usually is quite coarse. Result passed to control module (for example PID or MPC) and those modules are responsible for fine tuning of the trajectory. For this project, path planner takes partial responsibility of control module and suppose to generate fine trajectory - series of XY coordinates. To avoid sharp changes of the direction or large jerk, in addition to the methods described above, continuation ot the path is used. I.e. simulator provides back sequence of positions that calculated during previous planning step and not used by simulator yet. So to achive continuation path planner suppose to start next step planning not from current position of the vehicle, but from result of previous step. 

Since each step start calculation not from actual car position, but from result of previous prediction, discrepancies between predicted position and reality occurred very quickly. Luckely, simulator provided end_path_s and end_path_d feedback, that enabled correction of prediction error.

#### states of FSM

1. KeepLane - maintain car's D position and adjust velocity of the vehicle based on the speed either of car ahead or speed limit
2. SlowDown - special case of KeepLine designed to reduce speed of the ego when "cut-off" car is detected
3. ChangeLaneRight and ChangeLaneLeft - switch lane to achive higher velocity and distance to the car aher (leader)       

#### states of FSM

1.    
  

        