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
        plots =  [['n', 'n_g'], ['e', 'e_g'], ['h', 'h_g'],
                 ['Va', 'Va_g'], ['chi', 'chi_g'], ['traj']]

        # Add plots to the window
        for p in plots:
            self.plotter.add_plot(p)

        # Add legends
        self.plotter.add_legend('n')
        self.plotter.add_legend('e')
        self.plotter.add_legend('h')
        self.plotter.add_legend('chi')
        self.plotter.add_legend('Va')

        self.tf_listener = tf.TransformListener()

        # Define input vectors for easier input
        self.plotter.define_input_vector('neh',    ['n', 'e', 'h'])
        self.plotter.define_input_vector('neh_g',    ['n_g', 'e_g', 'h_g'])

        # Subscribe to relevant ROS topics
        rospy.Subscriber('ned_g', Vector3Stamped, self.ned_g_cb_)
        rospy.Subscriber('twist', TwistStamped, self.velocity_cb_)
        rospy.Subscriber('chi_g', Float32, self.chi_g_cb_)
        rospy.Subscriber('Va_g', Float32, self.va_g_cb_)

        self.crab = 0.0
        self.wind = np.zeros(3)

        # Update the plots
        rate = rospy.Rate(update_freq)
        while not rospy.is_shutdown():
            self.tick()
            self.plotter.update_plots()
            rate.sleep()

    def velocity_cb_(self, msg):
        t = msg.header.stamp.to_sec()
        twist = msg.twist
        self.crab = np.arctan2(twist.linear.y, twist.linear.x)

        V_g = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        Va = np.linalg.norm(V_g - self.wind)

        self.plotter.add_measurement('Va', Va, t)

    def ned_g_cb_(self, msg):
        t = msg.header.stamp.to_sec()
        vector = msg.vector
        self.plotter.add_vector_measurement('neh_g', [vector.x, vector.y, -vector.z], t)
        self.plotter.add_measurement('traj', vector.y, vector.x)

    def va_g_cb_(self, msg):
        # Extract time
        t = rospy.Time.now().to_sec()
        self.plotter.add_measurement('Va_g', msg.data, t)

    def chi_g_cb_(self, msg):
        # Extract time
        t = rospy.Time.now().to_sec()
        self.plotter.add_measurement('chi_g', msg.data, t, rad2deg=self.use_degrees)

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
        self.plotter.add_measurement('chi', euler[2] + self.crab, t, rad2deg=self.use_degrees)

    def tick(self):
        self.pose()



if __name__ == '__main__':
    rospy.init_node('mav_plotter', anonymous=False)

    try:
        obj = PlotWrapper()
    except rospy.ROSInterruptException:
        pass
