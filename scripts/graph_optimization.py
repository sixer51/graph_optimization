#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import g2o
import tf

# g2o graph optimizer
class GraphOptimization(g2o.SparseOptimizer):
    def __init__(self):
        super(GraphOptimization, self).__init__()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverCholmodSE2())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super(GraphOptimization, self).set_algorithm(solver)

    # optimize the graph
    def optimize(self, max_iterations=20):
        super(GraphOptimization, self).initialize_optimization()
        # super(GraphOptimization, self).save('graph.g2o')
        super(GraphOptimization, self).set_verbose(True)
        super(GraphOptimization, self).optimize(max_iterations)

    # add vertex to graph
    def add_vertex(self, id, pose, typeit, fixed=False):
        if typeit == 1: v_se2 = g2o.VertexSE2()  # pose
        else: v_se2 = g2o.VertexPointXY()  # landmark

        v_se2.set_id(id)
        v_se2.set_estimate(pose)
        v_se2.set_fixed(fixed)
        super(GraphOptimization, self).add_vertex(v_se2)
        print("add vertex", id)

    # add edge between poses or pose and landmark
    def add_edge(self, vertices, measurement, typeit, robust_kernel=None):
        if typeit == 1: 
            edge = g2o.EdgeSE2()  # between poses
            information=np.identity(3)
        else: 
            edge = g2o.EdgeSE2PointXY()  # between pose and landmark
            information=np.identity(2)

        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(measurement)  # relative pose
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super(GraphOptimization, self).add_edge(edge)
        print("add edge", vertices)

    # get the pose of the vertex
    def get_pose(self, id):
        return self.vertex(id).estimate()
        

# create rosnode and use graph optimizer to optimize estimated pose
class StateEstimate:
    def __init__(self):
        self.graph_optimizer = GraphOptimization()
        rospy.init_node("graph_optimization")
        self.posesub = rospy.Subscriber("pose", PoseStamped, self.pose_callback)
        self.scansub = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.velsub = rospy.Subscriber("cmd_vel", Twist, self.vel_callback)
        self.is_moving = False
        self.id = 0
        self.last_pose = [0, 0, 0]

        # add landmark vertex
        self.graph_optimizer.add_vertex(self.id, [2, 0], 2)
        self.id += 1
        self.distance = 2

        self.t = []
        self.init_time = rospy.get_time()
        self.x = []
        self.y = []
        self.theta = []

        self.get_initial_pose = False

    # callback function of pose topic
    def pose_callback(self, msg):
        if not self.get_initial_pose: 
            self.get_initial_pose = True
            self.initial_pose_x = msg.pose.position.x
            self.initial_pose_y = msg.pose.position.y
            rotation = (msg.pose.orientation.x, msg.pose.orientation.y, 
                    msg.pose.orientation.z, msg.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.initial_pose_theta = euler[2]
            self.init_time = rospy.get_time()


        self.t.append(rospy.get_time() - self.init_time)
        x = msg.pose.position.x - self.initial_pose_x
        y = msg.pose.position.y = self.initial_pose_y
        rotation = (msg.pose.orientation.x, msg.pose.orientation.y, 
                    msg.pose.orientation.z, msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(rotation)
        theta = euler[2] - self.initial_pose_theta
        self.x.append(x)
        self.y.append(y)
        self.theta.append(theta)

        # add pose vertex
        pose = g2o.SE2(x, y, theta)
        self.graph_optimizer.add_vertex(self.id, pose, 1)

        # add edge between poses
        if self.id != 1:
            relative_pose = [x - self.last_pose[0], y - self.last_pose[1], 
                            theta - self.last_pose[2]]
            measurement = g2o.SE2(relative_pose[0], relative_pose[1], relative_pose[2])
            self.graph_optimizer.add_edge([self.id, self.id - 1], measurement, 1)

        # add edge between pose and landmark
        measurement = [self.distance, 0]
        self.graph_optimizer.add_edge([self.id, 0], measurement, 2)

        self.last_pose = [x, y, theta]
        self.id += 1

    # callback function of scan topic
    def scan_callback(self, msg):
        for i in range(-5, 5):
            if msg.ranges[i] > 0 and msg.ranges[i] < self.distance: 
                self.distance = msg.ranges[i]
    
    # callback function of cmd_vel topic
    # to get the movement status of the robot
    def vel_callback(self, msg):
        if msg.linear.x != 0: self.is_moving = True
        else: self.is_moving = False

    # main funtion to do state estimation
    def main(self):
        while not self.is_moving: continue
        print("start moving")
        while self.is_moving: continue
        rospy.sleep(3)
        self.graph_optimizer.save(rospy.get_param('/graph_optimization/output_path'))
        self.graph_optimizer.optimize(50)
        self.graph_optimizer.save(rospy.get_param('/graph_optimization/output_path_after_optimization'))

        x_after_op = []
        y_after_op = []
        theta_after_op = []

        for i in range(1, self.id):
            trans = self.graph_optimizer.get_pose(i).translation()
            x_after_op.append(trans[0])
            y_after_op.append(trans[1])
            theta_after_op.append(self.graph_optimizer.get_pose(i).rotation().angle())

        print(self.x, self.y, self.theta, self.t)
        print(theta_after_op)

        plt.figure()
        self.plot("x", self.x, x_after_op)
        plt.figure()
        self.plot("y", self.y, y_after_op)
        plt.figure()
        self.plot("theta", self.theta, theta_after_op)

        plt.show()


    def plot(self, value, original, afterop):
        plt.plot(self.t, original, label = 'original')
        plt.plot(self.t, afterop, '*', label = 'after optimization')
        if value == "x":
            plt.plot(self.t, [0]*len(self.t), 'r', linestyle='solid', label = 'ground truth')
            plt.plot(self.t, [1.1]*len(self.t), 'r', linestyle='solid')
            plt.text(3, 0.4, "pose error: "+str(round(1.1 - original[-1], 4)))
            plt.text(3, 0.2, "after optimization pose error: "+str(round(1.1 - afterop[-1], 4)))
        else:
            plt.plot(self.t, [0]*len(self.t), 'r', linestyle='solid', label = 'ground truth')
            plt.text(3, 0.2, "pose error: "+str(round(original[-1], 4)))
            plt.text(3, 0.1, "after optimization pose error: "+str(round(afterop[-1], 4)))
        plt.title(value)
        plt.xlabel('time(s)')
        if value == "theta": plt.ylabel('angle(rad)')
        else: plt.ylabel('distance(m)')
        plt.legend(loc='upper left')


if __name__ == "__main__":
    robot = StateEstimate()
    robot.main()
    rospy.spin()