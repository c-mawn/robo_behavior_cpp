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

## Demo

### Person Following 
<iframe width="1020" height="630" src="https://www.youtube.com/embed/7QHE4I6VIrI?si=vJoSfSoesd86VSzt" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen>Demo Video of Person Following</iframe>

#### Implementation

To implement person following behavior, I utilized the Lidar on the neato, to track the nearest objest (Imagining that the closest object is a person). To do this, I took in the data from each lidar scan as a subscriber, then filtered out all the extra data (anything farther than 1 meter, or closer than 50 cm). I then take the average location of remaining lidar points, to find a destination. I publish commands to the neato to drive towards that destination using PID. 

### Path Following Demo
<iframe width="1020" height="630" src="https://www.youtube.com/embed/aoCDR-20jis?si=WJxjKf3AWofz-Lkx" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen>Demo Vide of Path Following</iframe>

#### Implementation
To implement a path following controller, I assumed that the goal poses were given (which could be from something like A*, but I didn't do that here), then calculate the path from the current pose to the goal pose. I then publish commands to tell the Neato to drive to the next goal pose. 

## About

This project was a part of my independent student research (isr), Fall 2025 at Olin College. 

My goals with this project were to
- Get a better understand of C++
- Learn how to use C++ with ROS2
- Create a project that would be fun to show others

The hardest part of this project for me was learning C++. I had essentially no C++ experience before doing this, so I really had to learn it from the ground up. I was already familiar with ROS2 before this project, which I believe gave me a really good jumping off point for learning C++. The best method for learning C++ I used for this was writing the basic outline of a pubsub line by line. It helped me map out all the ROS2 stuffs that I knew from python over to C++. It helped me figure out how objects are referenced, how functions take inputs, and how to use a pointer. Overall, having a project to learn C++ was much better than trying to learn it from a random website (like how I usually learn new languages). 

In the future, I would like to keep working on this, and I have a few ideas of how to continue. 
- I would like to refactor a lot of this code now that I am getting a grasp of C++. I have a lot of really messy stuff in here, which I would like to clean up. Some of which is already being done. 
- I would like to use this package as a learning environment for myself in the future. I have a small amount of experience in a lot of different sections of robotics, and I want to get better at those by stitching them into this code. For example, I would love to get more experience implementing computer vision stuff, localization stuff, path planning, SLAM based things, and more complex controls theories.



