# Simultaneous Localization and Mapping (SLAM) using particle filters

![alt text](./results/slam.gif)

This project aims to simultaneously localize a walking humanoid robot and map an unknown indoor environment using odometry data, IMU orientation and a 2D laser range scanner (LIDAR). A particle filter based approach is taken to achieve the objective. Particle filters effectively model a probability distribution as a set of discrete states. Each particle has an associated weight which describes its confidence in its current estimate of pose according to the observation. As time progresses, the particles survive in proportion with how consistent its lidar scans are. [This video](https://www.youtube.com/watch?v=aUkBa1zMKv4) provides a good introduction to particle filters. 

In the gif above, the yellow dots represent all the particles and the red dot represents the particle with the highest weight. The map is a log odds map where values closer to 1 (white) represent obstacles and values closer to 0 (black) represent free space.

This project was done as a part of ESE650 Learning in Robotics, University of Pennsylvania in the spring of 2015.

More information can be found in the [report](./report/project4.pdf)

#### THOR robot
![alt text](./robot.JPG "thor")

## Results

![alt text](./results/slam0_result.png)
![alt text](./results/slam2_result.png)

## References

Dr. Daniel Lee's ESE 650 lecture notes
