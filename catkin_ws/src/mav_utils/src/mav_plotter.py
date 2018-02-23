#!/usr/bin/env python
from __future__ import division

import rospy
import tf

import numpy as np

from state_plotter.Plotter import Plotter

from geometry_msgs.msg import TwistStamped, Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

class PlotWrapper:
    """
    PlotWrapper class for connecting ROS topics to the Plotter object
    """
    def __init__(self, update_freq=30, use_degrees=True):

        # Store parameters
        self.use_degrees = use_degrees

        # Setup the plotter window
        self.plotter = Plotter()

        # Define plot names
        plots =  [['n', 'n_gps'],      ['e', 'e_gps'],        ['h', 'h_gps', 'h_c'],
                 'phi',     'theta',  'psi',
                 'u',       'v',        'w',
                 ['p', 'p_gyro'],       ['q', 'q_gyro'],      ['r', 'r_gyro'],
                 'ax',     'ay',        'az',
                 ['Va', 'Va_c'], ['chi', 'chi_c', 'chi_gps'], ['p_diff', 'p_abs'] ]

        self.wind = np.zeros(3)
        self.crab = 0.0

        # Add plots to the window
        for p in plots:
            self.plotter.add_plot(p)

        # Add legends
        self.plotter.add_legend('n')
        self.plotter.add_legend('e')
        self.plotter.add_legend('h')
        self.plotter.add_legend('p')
        self.plotter.add_legend('q')
        self.plotter.add_legend('r')
        self.plotter.add_legend('Va')
        self.plotter.add_legend('chi')

        self.tf_listener = tf.TransformListener()

        # Define input vectors for easier input
        self.plotter.define_input_vector('neh',    ['n', 'e', 'h'])
        self.plotter.define_input_vector('lin_velocity',    ['u', 'v', 'w'])
        self.plotter.define_input_vector('ang_velocity',    ['p', 'q', 'r'])
        self.plotter.define_input_vector('euler', ['phi', 'theta', 'psi'])
        self.plotter.define_input_vector('neh_gps', ['n_gps', 'e_gps', 'h_gps'])
        self.plotter.define_input_vector('ang_gyro', ['p_gyro', 'q_gyro', 'r_gyro'])
        self.plotter.define_input_vector('acc', ['ax', 'ay', 'az'])

        # Subscribe to relevant ROS topics
        rospy.Subscriber('twist', TwistStamped, self.velocity_cb_)
        rospy.Subscriber('imu', Imu, self.imu_cb_)
        rospy.Subscriber('Va_c', Float32, self.va_cb_)
        rospy.Subscriber('gps_neh', Vector3Stamped, self.gps_neh_cb_)
        rospy.Subscriber('gps_chi', Float32, self.gps_chi_cb_)
        rospy.Subscriber('gps_vg', Float32, self.gps_vg_cb_)
        rospy.Subscriber('h_c', Float32, self.h_cb_)
        rospy.Subscriber('chi_c', Float32, self.chi_cb_)
        rospy.Subscriber('p_diff', Float32, self.p_diff_cb_)
        rospy.Subscriber('p_static', Float32, self.p_static_cb_)

        self.va_c = None
        self.h_c = None
        self.chi_c = None

        # Update the plots
        rate = rospy.Rate(update_freq)
        while not rospy.is_shutdown():
            self.tick()
            self.plotter.update_plots()
            rate.sleep()

    def p_diff_cb_(self, msg):
        t = rospy.Time.now().to_sec()
        self.plotter.add_measurement('p_diff' ,msg.data, t)

    def p_static_cb_(self, msg):
        t = rospy.Time.now().to_sec()
        self.plotter.add_measurement('p_abs' ,msg.data, t)

    def velocity_cb_(self, msg):
        # Extract time
        t = msg.header.stamp.to_sec()

        # Handle position measurements
        twist = msg.twist
        self.plotter.add_vector_measurement('lin_velocity',
                [twist.linear.x, twist.linear.y, twist.linear.z], t)

        self.plotter.add_vector_measurement('ang_velocity',
                [twist.angular.x, twist.angular.y, twist.angular.z], t, rad2deg=self.use_degrees)

        self.crab = np.arctan2(twist.linear.y, twist.linear.x)

        V_g = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        Va = np.linalg.norm(V_g - self.wind)

        self.plotter.add_measurement('Va', Va, t)

    def imu_cb_(self, msg):
        t = msg.header.stamp.to_sec()

        gyro = msg.angular_velocity
        acc = msg.linear_acceleration

        self.plotter.add_vector_measurement('acc',
                [acc.x, acc.y, acc.z], t)

        self.plotter.add_vector_measurement('ang_gyro',
                [gyro.x, gyro.y, gyro.z], t, rad2deg=self.use_degrees)

    def va_cb_(self, msg):
        # Extract time
        t = rospy.Time.now().to_sec()
        self.plotter.add_measurement('Va_c', msg.data, t)
        self.va_c = msg.data

    def gps_neh_cb_(self, msg):
        t = msg.header.stamp.to_sec()
        
        self.plotter.add_vector_measurement('neh_gps',
                [msg.vector.x, msg.vector.y, msg.vector.z], t)

    def gps_chi_cb_(self, msg):
        # Extract time
        t = rospy.Time.now().to_sec()
        self.plotter.add_measurement('chi_gps', msg.data, t, rad2deg=self.use_degrees)

    def gps_vg_cb_(self, msg):
        pass

    def h_cb_(self, msg):
        # Extract time
        t = rospy.Time.now().to_sec()
        self.plotter.add_measurement('h_c', msg.data, t)
        self.h_c = msg.data

    def chi_cb_(self, msg):
        # Extract time
        t = rospy.Time.now().to_sec()
        self.plotter.add_measurement('chi_c', msg.data, t, rad2deg=self.use_degrees)
        self.chi_c = msg.data

    def pose(self):
        try:
            # Extract time
            t = self.tf_listener.getLatestCommonTime("world_ned", "base_link")

        except:
            return

        # Handle position measurements
        position, quaternion = self.tf_listener.lookupTransform("world_ned", "base_link", t)
        position[2] = -position[2]
        t = t.to_sec()
        self.plotter.add_vector_measurement('neh', position, t)

        # Use ROS tf to convert to Euler angles from quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # Add angles and angular velocities
        self.plotter.add_vector_measurement('euler', euler, t, rad2deg=self.use_degrees)

        self.plotter.add_measurement('chi', euler[2] + self.crab, t, rad2deg=self.use_degrees)

    def tick(self):
        self.pose()

        t = rospy.Time.now().to_sec()
        if self.va_c is not None:
            self.plotter.add_measurement('Va_c', self.va_c, t)
        if self.h_c is not None:
            self.plotter.add_measurement('h_c', self.h_c, t)
        if self.chi_c is not None:
            self.plotter.add_measurement('chi_c', self.chi_c, t, rad2deg=self.use_degrees)




if __name__ == '__main__':
    rospy.init_node('mav_plotter', anonymous=False)

    try:
        obj = PlotWrapper()
    except rospy.ROSInterruptException:
        pass
