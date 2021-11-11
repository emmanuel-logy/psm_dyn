#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Pose
from ambf_client import Client
from dvrk_fk.srv import joint_pos

# psm1_obj = None
# i = 0

# def	dvrk_fk_cb(msg):
# 	global psm1_obj
# 	if i < 1:
# 		print("dvrk_fk_cb called")
# 		# Now you can use the psm1_obj to set and get its position, rotation,
# 		# Pose etc. If the object has joints, you can also control them
# 		# in either position control mode or open loop effort mode. You can even mix and
# 		# match the joints commands
# 		psm1_obj.set_pos(msg.position.x, msg.position.y, msg.position.z) # Set the XYZ Pos in obj's World frame
#
# 		# Convert quats to Euler Angles
# 		quaternion = (  msg.orientation.x,
# 									msg.orientation.y,
# 									msg.orientation.z,
# 									msg.orientation.w)
# 		euler = tf.transformations.euler_from_quaternion(quaternion)
# 		psm1_obj.set_rpy(euler[0], euler[1], euler[2]) # Set the Fixed RPY in World frame
# 		rospy.sleep(5) # Sleep for a while to see the effect of the command before moving on
#
# 		# Other methods to control the obj position include
# 		# psm1_obj.set_pose(pose_cmd) # Where pose_cmd is of type Geometry_msgs/Pose
# 		# psm1_obj.set_rot(quaterion) # Where quaternion is a list in the order of [qx, qy, qz, qw]
# 		# Finally all the position control params can be controlled in a single method call
# 		# psm1_obj.pose_command(px, py, pz, roll, pitch, yaw, *jnt_cmds)
#
# 		# We can just as easily get the pose information of the obj
# 		# cur_pos = psm1_obj.get_pos() # xyx position in World frame
# 		# print("cur_pos: ", cur_pos)
# 		# cur_rot = psm1_obj.get_rot() # Quaternion in World frame
# 		# cur_rpy = psm1_obj.get_rpy() # Fixed RPY in World frame
# 		# print("cur_rpy: ", cur_rpy)
#
# 		# Similarly you can directly control the wrench acting on the obj by
# 		# psm1_obj.set_force(5, -5, 10) # Set the force in the World frame
# 		# psm1_obj.set_torque(0, 0, 0.8) # Set the torque in the World frame
# 		# rospy.sleep(5) # Sleep for a while to see the effect of the command before moving on
#
# 		# Similarly you can directly control the wrench acting on the obj by
# 		# The key difference is that it's the user's job to update the forces
# 		# and torques in a loop otherwise the wrench in cleared after an internal
# 		# watchdog timer expires if a new command is not set. This is for safety
# 		# reasons where a user shouldn't set a wrench and then forget about it.
# 		# for i in range(0, 50):
# 		#	psm1_obj.set_force(5, -5, 10) # Set the force in the World frame
# 		#	psm1_obj.set_torque(0, 0, 0.8) # Set the torque in the World frame
# 		#	rospy.sleep(0.001) # Sleep for a while to see the effect of the command before moving on
#
# 		# We can get the number of children and joints connected to this body as
# 		# num_joints = psm1_obj.get_num_joints() # Get the number of joints of this object
# 		# children_names = psm1_obj.get_children_names() # Get a list of children names belonging to this obj
#
# 		# print(num_joints)
# 		# print(children_names)
#
# 		# If the obj has some joints, we can control them as follows
# 		# if num_joints > 1:
# 		#	psm1_obj.set_joint_pos(0, 0.5) # The the joints at idx 0 to 0.5 Radian
# 		#	psm1_obj.set_joint_effort(1, 5) # Set the effort of joint at idx 1 to 5 Nm
# 		#	rospy.sleep(2) # Sleep for a while to see the effect of the command before moving on


def handle_psm_joint_cmds(req):
	# Create a instance of the client
	_client = Client()

	# Connect the client which in turn creates callable objects from ROS topics
	# and initiates a shared pool of threads for bidrectional communication
	_client.connect()

	# You can print the names of objects found
	# print(_client.get_obj_names())

	# Lets say from the list of printed names, we want to get the
	# handle to an object names "Torus"
	psm1_obj = _client.get_obj_handle('/ambf/env/psm1/baselink')

	print("**************: ", type(req))
	psm1_obj.set_joint_pos(0, req.j1) # The the joints at idx 0 to 0.5 Radian
	psm1_obj.set_joint_pos(1, req.j2) # The the joints at idx 0 to 0.5 Radian
	psm1_obj.set_joint_pos(2, req.j3) # The the joints at idx 0 to 0.5 Radian
	psm1_obj.set_joint_pos(3, req.j4) # The the joints at idx 0 to 0.5 Radian
	psm1_obj.set_joint_pos(4, req.j5) # The the joints at idx 0 to 0.5 Radian
	psm1_obj.set_joint_pos(5, req.j6) # The the joints at idx 0 to 0.5 Radian
	psm1_obj.set_joint_pos(6, req.j7) # The the joints at idx 0 to 0.5 Radian
	rospy.sleep(2)

	# num_joints = psm1_obj.get_num_joints() # Get the number of joints of this object
	# print(num_joints)
	#
	# input("Press J1")
	# for i in range(0,10,1):
	# 	psm1_obj.set_joint_pos(0, 0.1+i/10) # The the joints at idx 0 to 0.5 Radian
	# 	rospy.sleep(1)
	# input("Press J2")
	# for i in range(0,10,1):
	# 	psm1_obj.set_joint_pos(1, 0.1+i/10) # The the joints at idx 0 to 0.5 Radian
	# 	rospy.sleep(1)
	# input("Press J3")
	# for i in range(0,10,1):
	# 	psm1_obj.set_joint_pos(2, 0.1+i/10) # The the joints at idx 0 to 0.5 Radian
	# 	rospy.sleep(1)
	# input("Press J4")
	# for i in range(0,10,1):
	# 	psm1_obj.set_joint_pos(3, 0.1+i/10) # The the joints at idx 0 to 0.5 Radian
	# 	rospy.sleep(1)
	# input("Press J5")
	# for i in range(0,10,1):
	# 	psm1_obj.set_joint_pos(4, 0.1+i/10) # The the joints at idx 0 to 0.5 Radian
	# 	rospy.sleep(1)
	# input("Press J6")
	# for i in range(0,10,1):
	# 	psm1_obj.set_joint_pos(5, 0.1+i/10) # The the joints at idx 0 to 0.5 Radian
	# 	rospy.sleep(1)
	# input("Press J7")
	# for i in range(0,10,1):
	# 	psm1_obj.set_joint_pos(6, 0.1+i/10) # The the joints at idx 0 to 0.5 Radian
	# 	rospy.sleep(1)
	# input("Press J8")
	# for i in range(0,10,1):
	# 	psm1_obj.set_joint_pos(7, 0.1+i/10) # The the joints at idx 0 to 0.5 Radian
	# 	rospy.sleep(1)
	# subscribe to topic from FK cpp node
	# rospy.Subscriber("/dvrk_fk", Pose, dvrk_fk_cb)

	# Lastly to cleanup
	_client.clean_up()
	return True


if __name__ == '__main__':
	try:
		rospy.init_node('psm_ros_server', anonymous=True)
		s = rospy.Service('psm_ros_server', joint_pos, handle_psm_joint_cmds)
		print("Ready to send cmds to PSM")
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
