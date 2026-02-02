#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetEntityState
import numpy as np
import math

class RobotEnv(Node):
    def __init__(self):
        super().__init__('robot_env')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.reset_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        self.odom = None
        self.scan = None
        self.goal = np.array([9.0, 7.0])
        
        self.observation_shape = 4
        self.n_active_features = 1
        self.action_space = type('obj', (object,), {'n': 2})()

    def odom_cb(self, msg):
        self.odom = msg

    def scan_cb(self, msg):
        self.scan = msg

    def get_yaw(self, q):
        sy_cp = 2.0 * (q.w * q.z + q.x * q.y)
        cy_cp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sy_cp, cy_cp)

    def get_state(self):
        if self.odom is None or self.scan is None:
            return np.zeros(self.observation_shape)
        
        p = self.odom.pose.pose.position
        yaw = self.get_yaw(self.odom.pose.pose.orientation)
        
        dist = np.hypot(self.goal[0] - p.x, self.goal[1] - p.y)
        angle = math.atan2(self.goal[1] - p.y, self.goal[0] - p.x) - yaw
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        
        obs = np.array([
            dist, 
            angle, 
            min(self.scan.ranges) if self.scan.ranges else 10.0,
            self.odom.twist.twist.linear.x
        ])
        return obs

    def step(self, action):
        v = float(np.clip((action[0] + 1.0) * 0.25, 0.0, 0.5))
        w = float(np.clip(action[1], -1.0, 1.0))
        
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)
        
        state = self.get_state()
        reward, done = self.calculate_reward(state)
        return state, reward, done, False, {}

    def calculate_reward(self, state):
        d, a, s, _ = state
        if d < 0.3: return 1000.0, True
        if s < 0.2: return -500.0, True
        
        r = (1.0 / (d + 0.5)) + (math.cos(a) * 0.5) - 0.1
        return r, False

    def reset(self):
        self.cmd_pub.publish(Twist())
        
        req = SetEntityState.Request()
        req.state.name = 'robot'
        req.state.pose.position.x = 0.0
        req.state.pose.position.y = 0.0
        
        if self.reset_client.wait_for_service(timeout_sec=1.0):
            self.reset_client.call_async(req)
            
        return self.get_state(), {}