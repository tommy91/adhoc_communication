#!/usr/bin/python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from adhoc_communication.srv import ChangeMCMembership, SendString
from adhoc_communication.msg import RecvString


topic_new_robot = "/adhoc_communication/new_robot"
topic_remove_robot = "/adhoc_communication/remove_robot"
topic_status = "/adhoc_communication/protelisStatus"

mc_prefix = "mc_robot_"
robots = [
	{
		'name': "robot_0",
		'mc_name': mc_prefix + "0",
		'positions': [(0,0),(0,0),(0,0)]
	},
	{
		'name': "robot_1",
		'mc_name': mc_prefix + "1",
		'positions': [(0,5),(0,10),(0,5)]
	}
]
# 	{
# 		'name': "robot_2", 
# 		'mc_name': mc_prefix + "2",
# 		'positions': [(0,-5),(0,-2),(0,-5)]
# 	},
# 	{
# 		'name': "robot_3",
# 		'mc_name': mc_prefix + "3",
# 		'positions': [(5,0)]
# 	},
# 	{
# 		'name': "robot_4",
# 		'mc_name': mc_prefix + "4",
# 		'positions': [(-5,0)]
# 	}
# ]

publishers = []
subscribers = []
neighbors = []
num_robots = len(robots);


def sendStatus(dest, data):
	service_name = "robot_%i/adhoc_communication/send_string"%num_robot
	print "robot_" + str(num_robot) + ": sending its status"
	send_status_srv = rospy.ServiceProxy(service_name, SendString)
	res = send_status_srv(dest,topic_status,data)
	print "robot_" + str(num_robot) + ": service response:\n" + str(res.status)


def joinMcGroup(robot_name, mc_group_name):
	service_name = "/" + robot_name + "/adhoc_communication/join_mc_group"
	print robot_name + ": trying to join " + mc_group_name + " group"
	join_mc_group_srv = rospy.ServiceProxy(service_name, ChangeMCMembership)
	res = join_mc_group_srv(mc_group_name,1)	
	print robot_name + ": service response:\n" + str(res.status)


def callbackNewRobot(data, args):
	current_robot_number = args[0]
	current_robot_name = "robot_" + str(current_robot_number)
	new_neighbor = data.data
	print current_robot_name + ": heard new neighbor " + new_neighbor
	if new_neighbor in neighbors[current_robot_number]:
		print current_robot_name + ": " + new_neighbor + " is already a neighbor of " + current_robot_name
	else:
		print current_robot_name + ": adding new neighbor " + new_neighbor
		neighbors[current_robot_number].append(new_neighbor)
		# Joining the mc group of the neighbor 
		neighbor_mc_group = "mc_" + new_neighbor
		joinMcGroup(current_robot_name, neighbor_mc_group)
		
		

def callbackRemoveRobot(data, args):
	current_robot_number = args[0]
	current_robot_name = "robot_" + str(current_robot_number)
	neighbor_to_remove = data.data
	print current_robot_name + ": received neighbor to remove " + neighbor_to_remove
	if neighbor_to_remove not in neighbors[current_robot_number]:
		print current_robot_name + ": " + neighbor_to_remove + " is not neighbor of " + current_robot_name
	else:
		print current_robot_name + ": removing neighbor " + neighbor_to_remove
		neighbors[current_robot_number].remove(neighbor_to_remove)
		# No need to leave the mc group of the removed neighbor
		# because I already lose the connection with him
		# so I'll be removed automatically by him


def callbackRecvString(data, args):
	current_robot_number = args[0]
	current_robot_name = "robot_" + str(current_robot_number)
	print current_robot_name + ": received data from " + data.src_robot + ":\n" + data.data


def waitEnterPress():
	while True:
		i = raw_input('Press enter to continue..\n')
		break


print "Initializing Positions and Publishers"
rospy.init_node("Simple_simulator_2")

# Setting publishers and subscribers
for num_robot in range(0,num_robots):
	pubs = {}
	position_topic = "robot_%i/base_pose_ground_truth"%num_robot
	pubs['pos'] = rospy.Publisher(position_topic,Odometry,queue_size=10,latch=False)
	publishers.append(pubs)
	topic_new_robot_complete = "/robot_" + str(num_robot) + topic_new_robot
	topic_remove_robot_complete = "/robot_" + str(num_robot) + topic_remove_robot
	topic_status_complete = "/robot_" + str(num_robot) + topic_status
	subscribers.append(rospy.Subscriber(topic_new_robot_complete, String, callbackNewRobot, (num_robot,)))
	subscribers.append(rospy.Subscriber(topic_remove_robot_complete, String, callbackRemoveRobot, (num_robot,)))
	subscribers.append(rospy.Subscriber(topic_status_complete, RecvString, callbackRecvString, (num_robot,)))
	neighbors.append([])

iteration_count = 0

try:
	
	while True:
	
		waitEnterPress()
	
		for num_robot in range(0,num_robots):
			msg = Odometry()
			msg.header.stamp = rospy.Time.now()
			(x,y) = robots[num_robot]['positions'][iteration_count]
			print "robot_" + str(num_robot) + ": publishing position (" + str(x) + ", " + str(y) + ")"
			msg.pose.pose.position = Point(x, y, 0)
			publishers[num_robot]['pos'].publish(msg)
		iteration_count += 1
		
		waitEnterPress()

		for num_robot in range(0,num_robots):
			dest = mc_prefix + str(num_robot)
			data = "status from robot_" + str(num_robot)
			sendStatus(dest, data)		
	
except Exception as e:
	print "\nException has been caught: " + str(e)
	print "Stopping subscribers"
	for subscriber in subscribers:
		subscriber.unregister()
	print "Exit, bye"
