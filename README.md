# Robotics in C++ ISR
### Charlie Mawn

## How to run

(Assuming a working ROS2 environment setup, if not, follow the guide [here](https://comprobo25.github.io/How%20to/setup_your_environment))

```bash
cd ros2_ws/src
git clone git@github.com:c-mawn/robo_behavior_cpp.git
cd ~/ros2_ws
colcon build --packages-select robo_behaviors person_follower
source install/setup.bash
```

These steps will set up the code from these packages on your local machine. In this repo there are 2 packages: `robo_behaviors` and `person_follower`. You can build both packages together (which is done above), but they are two seperate packages to be run.

### Person Follower

To run the `person_follower`, first connect to the Neato Simulator in Gazebo:

```bash
ros2 launch neato2_gazebo neato_gauntlet_world.py
```

Then, in a new terminal, run the person follower:

```bash
ros2 run person_follower pfollow
```
### Path Follower

To run the path follower, first connect to the Neato Simulator in Gazebo:

```bash
ros2 launch neato2_gazebo neato_gauntlet_world.py
```

Then, in a new terminal, run the person follower:

```bash
ros2 run robo_behaviors path_follow
```

