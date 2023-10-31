import rospy
import tf.transformations
import numpy as np

from geometry_msgs.msg import PoseStamped, Pose
from franka_msgs.msg import FrankaState

import sys
# adding to the system path
sys.path.insert(0, '/home/prashanth/Thesis/Imitation-Learning/')

import utils


pose = Pose()

def wait_for_initial_pose():
    msg = rospy.wait_for_message("panda_teleop/follower_state_controller/franka_states",
                                 FrankaState)  # type: FrankaState

    initial_quaternion = \
        tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / \
        np.linalg.norm(initial_quaternion)
    pose.orientation.x = initial_quaternion[0]
    pose.orientation.y = initial_quaternion[1]
    pose.orientation.z = initial_quaternion[2]
    pose.orientation.w = initial_quaternion[3]
    pose.position.x = msg.O_T_EE[12]
    pose.position.y = msg.O_T_EE[13]
    pose.position.z = msg.O_T_EE[14]

# rospy.init_node("teleop_botterneck_pose1")
# wait_for_initial_pose()
# print(pose)

import tf_conversions
bottleneck_pose = tf_conversions.fromMsg(pose)

# import sys
# sys.path.insert(0, '/home/prashanth/Thesis/Imitation-Learning/')
# from Robot import kdl_utils
# bottleneck_pose_vertical = kdl_utils.create_vertical_pose_from_x_y_z_theta(bottleneck_pose.p[0], bottleneck_pose.p[1], bottleneck_pose.p[2], bottleneck_pose.M.GetRPY()[2])
# print(bottleneck_pose_vertical)
import rospy

class Robot:
    def __init__(self, topic='panda_teleop/follower_state_controller/franka_states'):
        self.robot_topic = topic
        self.cart_pos = None
        self.cart_vel = None
        self.joint_pos = None
        self.jacobian = None

        self.robot_sub =  rospy.Subscriber(self.robot_topic, FrankaState, self.state_callback, tcp_nodelay=True)
        # self._init_subs()

    def _init_subs(self):
        self.robot_sub =  rospy.Subscriber(self.robot_topic, FrankaState, self.state_callback)

    
    def state_callback(self, msg):
        initial_quaternion = \
            tf.transformations.quaternion_from_matrix(
                np.transpose(np.reshape(msg.O_T_EE,
                                        (4, 4))))
        initial_quaternion = initial_quaternion / \
            np.linalg.norm(initial_quaternion)

        pose = Pose()
        pose.orientation.x = initial_quaternion[0]
        pose.orientation.y = initial_quaternion[1]
        pose.orientation.z = initial_quaternion[2]
        pose.orientation.w = initial_quaternion[3]
        pose.position.x = msg.O_T_EE[12]
        pose.position.y = msg.O_T_EE[13]
        pose.position.z = msg.O_T_EE[14]

        self.cart_pos = pose
        # print(pose)

    def wait_for_initial_pose(self):
        msg = rospy.wait_for_message(self.robot_topic, FrankaState)
        initial_quaternion = \
            tf.transformations.quaternion_from_matrix(
                np.transpose(np.reshape(msg.O_T_EE,
                                        (4, 4))))
        initial_quaternion = initial_quaternion / \
            np.linalg.norm(initial_quaternion)

        pose = Pose()
        pose.orientation.x = initial_quaternion[0]
        pose.orientation.y = initial_quaternion[1]
        pose.orientation.z = initial_quaternion[2]
        pose.orientation.w = initial_quaternion[3]
        pose.position.x = msg.O_T_EE[12]
        pose.position.y = msg.O_T_EE[13]
        pose.position.z = msg.O_T_EE[14]

        return pose



if __name__ == "__main__":
    
    rospy.init_node("teleop_botterneck_pose")
    robot = Robot()
    utils.set_up_terminal_for_key_check()

    poses = []
    trajectory = []

    print('Press \
        \n "b" to record bottelneck pose \
        \n "m" to move to weighted bottleneck pose \
        \n "r" to record trajectory \
        \n "s" to save \
        \n "x" to exit')
    
    try:
        while not rospy.is_shutdown():
            # Record the bottleneck poses for varying network delays
            if utils.check_for_key('b'):
                print('Record the current pose as bottleneck pose')
                pose = robot.wait_for_initial_pose()
                poses.append(pose)
                print('Total no of bottlenect poses: {} '.format(len(poses)))

            # Get weighted average from the poses
            if utils.check_for_key('m'):
                delays = [0, 0.1, 0.2, 0.3, 0.4, 0.5]
                wrighted_pose = None
                for i, pose in enumerate(poses):
                    bottleneck_pose = tf_conversions.fromMsg(pose)
                    # w_pose = min(delays) / delays[i] * pose
                    pass


            # Record Interaction trajectory for Ergodic exploration
            ros_rate = rospy.Rate(30)
            if utils.check_for_key('t'):
                print('Record trajectory started, press "x" to stop')
                while not rospy.is_shutdown(): 
                    pose = robot.cart_pos
                    trajectory.append(pose)
                    ros_rate.sleep()

                    if utils.check_for_key('x'):
                        print('Stopping trajectory recording')
                        print('Total no of traj points: {} '.format(len(trajectory)))
                        break

            if utils.check_for_key('s'):
                print('Saving data')
                np.save('./data/poses.npy', poses)
                np.save('./data/trajectory.npy', trajectory)

            if utils.check_for_key('x'):
                print('Shuttingdown ros')
                utils.reset_terminal()
                rospy.signal_shutdown('Keyboard Interupt')
                break
            # Repeat for different network latency
    
    except Exception:
            utils.reset_terminal()
    finally :
        utils.reset_terminal() 

   