#!/usr/bin/python2
# coding=UTF-8
"""This is goint to be a Python wrapper for KUKA youBot ROS interface."""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from brics_actuator.msg import JointPositions, JointValue, JointVelocities


class YouBot(object):

    """youBot class.

    Has Base and Arm atributes.
    """

    def __init__(self):
        """Class constructor."""
        rospy.init_node('rospyoubot')
        self.arm = Arm()
        self.base = Base()
        self.freq = 10

class Base(object):

    """Control youBot Base."""

    def __init__(self):
        """Class constructor."""
        self.velocity = Twist()
        # self.velocity.linear.x = 0
        # self.velocity.linear.y = 0
        # self.velocity.angular.z = 0
        self.odometry = Odometry()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)

    def publish_state(self):
        """Publish info about base."""
        self.velocity_publisher.publish(self.velocity)

    def set_velocity(self, lin_x=0, lin_y=0, ang_z=0):
        """Set base velocity vector."""
        self.velocity.linear.x = lin_x
        self.velocity.linear.y = lin_y
        self.velocity.angular.z = ang_z
        self.velocity_publisher.publish(self.velocity)

    def update_odometry(self, data):
        """Update odometry every time message is recieved."""
        self.odometry = data

    def get_odometry(self):
        """Return list with odometry.

        get_odometry() -> [x, y, z, rotation x, rotation y, rotation z]
        """
        return self.odometry.pose.covariance

class Arm(object):

    """Control youBot Arm."""

    def __init__(self):
        """Class constructor."""
        self.gripper = Gripper()
        self.joints_positions = JointPositions()
        self.joints_velocities = JointVelocities()
        self.joints_positions_publisher = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions)
        self.joints_velocities_publisher = rospy.Publisher('/arm_1/arm_controller/velocity_command', JointVelocities)

    def set_joints_angles(self, angles):
        """Set arm joints to defined angles.

        angles -> list of joints angles
        """
        assert len(angles) <= 5
        self.joints_positions.positions = []
        for i in range(len(angles)):  # I know it's unpythonic, sorry
            tmp = JointValue()
            tmp.timeStamp = rospy.Time.now()
            tmp.joint_uri = 'arm_joint_{}'.format(i+1)
            tmp.unit = 'rad'
            tmp.value = angles[i]
            self.joints_positions.positions.append(tmp)
        self.joints_positions_publisher.publish(self.joints_positions)

    def set_joints_velocities(self, velocities):
        assert len(velocities) <= 5
        self.joints_velocities.velocities = []
        for i in range(len(velocities)):  # I know it's unpythonic, sorry
            tmp = JointValue()
            tmp.timeStamp = rospy.Time.now()
            tmp.joint_uri = 'arm_joint_{}'.format(i+1)
            tmp.unit = 's^-1 rad'
            tmp.value = velocities[i]
            self.joints_velocities.velocities.append(tmp)
        self.joints_velocities_publisher.publish(self.joints_velocities)

class Gripper(object):

    """Gripper class."""

    def __init__(self):
        """Gripper constructor."""
        self.gripper_position = JointPositions()
        self.gripper_position_publisher = rospy.Publisher('arm_1/gripper_controller/position_command', JointPositions)

    def set_gripper_state(self, open_gripper=True):
        """Open/close gripper."""
        self.gripper_position.positions = []
        if open_gripper:
            # Open gripper
            tmp_gripper_position_r = JointValue()
            tmp_gripper_position_r.joint_uri = 'gripper_finger_joint_r'
            tmp_gripper_position_r.value = 0.011499
            tmp_gripper_position_r.unit = 'm'
            tmp_gripper_position_r.timeStamp = rospy.Time.now()
            self.gripper_position.positions.append(tmp_gripper_position_r)
            tmp_gripper_position_l = JointValue()
            tmp_gripper_position_l.joint_uri = 'gripper_finger_joint_l'
            tmp_gripper_position_l.value = 0.011499
            tmp_gripper_position_l.unit = 'm'
            tmp_gripper_position_l.timeStamp = rospy.Time.now()
            self.gripper_position.positions.append(tmp_gripper_position_l)
        else:
            # Close gripper
            tmp_gripper_position_r = JointValue()
            tmp_gripper_position_r.joint_uri = 'gripper_finger_joint_r'
            tmp_gripper_position_r.value = 0
            tmp_gripper_position_r.unit = 'm'
            tmp_gripper_position_r.timeStamp = rospy.Time.now()
            self.gripper_position.positions.append(tmp_gripper_position_r)
            tmp_gripper_position_l = JointValue()
            tmp_gripper_position_l.joint_uri = 'gripper_finger_joint_l'
            tmp_gripper_position_l.value = 0
            tmp_gripper_position_l.unit = 'm'
            tmp_gripper_position_l.timeStamp = rospy.Time.now()
            self.gripper_position.positions.append(tmp_gripper_position_l)
        self.gripper_position_publisher.publish(self.gripper_position)
