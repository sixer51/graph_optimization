# graph_optimization
This package bases on g2o package and realizes the comparision of original pose and pose after graph optimization.

## How to use
(1)install g2o package following instruction in https://github.com/uoip/g2opy  
(2)`source devel/setup.bash` and `catkin_make` as what we usually do in ros  
(3)`roslaunch graph_optimization graph_optimization.launch`  
Then two .g2o file which correspond to original pose graph and pose graph after graph optimization will be saved in graph_optimization/g2o_graph/. The name of them can be modified by ros param:`output_path` and `output_path_after_optimization`. 

## Bag File Content
The bag file provided in this package records a husarion robot starting from 2m away from the wall(facing the wall) and moving towards the wall with a speed of 0.2m/s for 5 seconds. Two meter is the distance from the front end of the robot to the wall. The final stop position is 0.9m away from the wall which can be seen as ground truth. 

## Implementation
### 1. Graph Optimization 
This is the first class based on following template.  
https://github.com/RainerKuemmerle/g2o/wiki/File-Format-SLAM-2D  
It is modified from 3D version to 2D version which means we only consider the x, y, theta of robot and x, y of landmark. In add_vertex and add_edge funtion, the abilities of adding landmark vertex and adding edge between landmark and pose are added.

### 2. State Esitimzation
This class initializes node "graph_optimization" and subscribes to "cmd_vel", "pose" and "scan". The "cmd_vel" topic is used to decide if we should start adding vertices and edge. Optimizer starts contructing graph when it is not zero and stop when it get back to zero, which corresponds to robot moving and stop. 
#### (1) add vertices  
I add the first landmark vertex when initialing the class. The pose is set as [2, 0], which is the wall position. Whenever a pose message is received, a new pose vertex will be added into the graph. Their position x and y is directly recorded in pose message, but theta need to be calculated by tf.transformation.eulerfromquarternion.
#### (2) add edges
An edge between poses and an edge between pose and landmark are added when a new pose message is received. The measurement between two poses can be comculated by current pose subtracting last pose. The order of vertices id should be current pose id, last pose id. The measurement between pose and landmark is the minimum distance in the front side of the laser. x is set to be that distance and y is set to be 0. The order of vertices id should be current pose id, landmark id.
#### (3) save .g2o file and optimization
The .g2o file can be save with the save function of GraphOptimization object. If the path is just set to be the name of the g2o file, it might be saved in the temporay folder in ~/.ros. For this assignment, I save them in g2o_graph folder in this package as the default path. After saving the original graph, we can optimize it with optimize function.
#### (4) plot result
After getting the original and after optimization poses, they can be plotted and compared with matplotlib tool. The time interval as not constant, but the real time step between two poses message.

## Performance and Evaluation
After running the launch file, 58 vertices inculding one landmark vertex and 113 edges are added into the graph. Then a 50- iteration optimization is runned. The results of optimized poses can be seen as follows.  
Though the final error of x is smaller after graph optimization, the trend of all x, y, theta and error of y and theta are obviously worse than the original one. On my option, the most possible reason is that in the first second, there is no pose position update which might because of the delay of wheel odometry, but a lot of poses messages are published and be added as vertices in that second. To get the best fit of graph, much error is introduced in it. If we start building the graph from the second second, the final result should be much better.
![pose_x](https://github.com/sixer51/graph_optimization/blob/master/results/x.png)
![pose_y](https://github.com/sixer51/graph_optimization/blob/master/results/y.png)  
![pose_theta](https://github.com/sixer51/graph_optimization/blob/master/results/theta.png)  

## Reference
The bag file is recorded by Mingi Jeong.
