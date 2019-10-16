#!/usr/bin/python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix 
from adhoc_communication.srv import ChangeMCMembership, SendString, GetID
from adhoc_communication.msg import RecvString


namespace_prefix = "/prj_device_"
name_prefix = "robot_"
adhoc_node_name = "adhoc_communication"
mavros_node_name = "mavros"
topic_new_robot = "new_robot"
topic_remove_robot = "remove_robot"
topic_status = "protelisStatus"


mc_prefix = "mc_robot_"
robots = [
	{
		'name': name_prefix + "1",
		'mc_name': mc_prefix + "1",
		'positions': [(0,0,1),(0,0,1),(0,0,1)]
	},
	{
		'name': name_prefix + "2",
		'mc_name': mc_prefix + "2",
		'positions': [(0,5,1),(0,20,1),(0,5,1)]
	}
]
# 	{
# 		'name': name_prefix + "3", 
# 		'mc_name': mc_prefix + "3",
# 		'positions': [(0,-5,1),(0,-2,1),(0,-5,1)]
# 	},
# 	{
# 		'name': name_prefix + "4",
# 		'mc_name': mc_prefix + "4",
# 		'positions': [(5,0,1)]
# 	},
# 	{
# 		'name': name_prefix + "5",
# 		'mc_name': mc_prefix + "5",
# 		'positions': [(-5,0,1)]
# 	}
# ]


publishers = []
subscribers = []
neighbors = {}
num_robots = len(robots);


def getAdhocPrefix(num_robot):
	return namespace_prefix + str(num_robot) + "/" + adhoc_node_name + "/"

def getMavrosPrefix(num_robot):
	return namespace_prefix + str(num_robot) + "/" + mavros_node_name + "/"


def getRobotName(num_robot):
	return name_prefix + str(num_robot)


def getID(num_robot):
	service_name = getAdhocPrefix(num_robot) + "get_id"
	robot_name = getRobotName(num_robot)
	print robot_name + ": getting its ID"
	get_id_srv = rospy.ServiceProxy(service_name, GetID)
	res = get_id_srv()
	print robot_name + ": service response: " + str(res.id)


def sendStatus(dest, data, num_robot):
	service_name = getAdhocPrefix(num_robot) + "send_string"
	robot_name = getRobotName(num_robot)
	print robot_name + ": sending its status"
	send_status_srv = rospy.ServiceProxy(service_name, SendString)
	res = send_status_srv(dest,topic_status,data)
	print robot_name + ": service response:\n" + str(res.status)


def joinMcGroup(num_robot, mc_group_name):
	service_name = getAdhocPrefix(num_robot) + "join_mc_group"
	robot_name = getRobotName(num_robot)
	print robot_name + ": trying to join " + mc_group_name + " group"
	join_mc_group_srv = rospy.ServiceProxy(service_name, ChangeMCMembership)
	res = join_mc_group_srv(mc_group_name,1)	
	print robot_name + ": service response:\n" + str(res.status)


def callbackNewRobot(data, args):
	current_robot_number = args[0]
	current_robot_name = getRobotName(current_robot_number)
	new_neighbor = data.data
	print current_robot_name + ": heard new neighbor " + new_neighbor
	if new_neighbor in neighbors[current_robot_name]:
		print current_robot_name + ": " + new_neighbor + " is already a neighbor of " + current_robot_name
	else:
		print current_robot_name + ": adding new neighbor " + new_neighbor
		neighbors[current_robot_name].append(new_neighbor)
		# Joining the mc group of the neighbor 
		neighbor_mc_group = "mc_" + new_neighbor
		joinMcGroup(current_robot_number, neighbor_mc_group)
		

def callbackRemoveRobot(data, args):
	current_robot_number = args[0]
	current_robot_name = getRobotName(current_robot_number)
	neighbor_to_remove = data.data
	print current_robot_name + ": received neighbor to remove " + neighbor_to_remove
	if neighbor_to_remove not in neighbors[current_robot_name]:
		print current_robot_name + ": " + neighbor_to_remove + " is not neighbor of " + current_robot_name
	else:
		print current_robot_name + ": removing neighbor " + neighbor_to_remove
		neighbors[current_robot_name].remove(neighbor_to_remove)
		# No need to leave the mc group of the removed neighbor
		# because I already lose the connection with him
		# so I'll be removed automatically by him


def callbackRecvString(data, args):
	current_robot_number = args[0]
	print getRobotName(current_robot_number) + ": received data from " + data.src_robot + ":\n" + data.data


def waitEnterPress():
	while True:
		i = raw_input('Press enter to continue..\n')
		break


print "Initializing Positions and Publishers"
rospy.init_node("Simple_simulator_2")

# Setting publishers and subscribers
for num_robot in range(1, num_robots + 1):
	pubs = {}
	position_topic = getMavrosPrefix(num_robot) + "global_position/global"
	pubs['pos'] = rospy.Publisher(position_topic,NavSatFix,queue_size=10,latch=False)
	publishers.append(pubs)
	topic_new_robot_complete = getAdhocPrefix(num_robot) + topic_new_robot
	topic_remove_robot_complete = getAdhocPrefix(num_robot) + topic_remove_robot
	topic_status_complete = getAdhocPrefix(num_robot) + topic_status
	subscribers.append(rospy.Subscriber(topic_new_robot_complete, String, callbackNewRobot, (num_robot,)))
	subscribers.append(rospy.Subscriber(topic_remove_robot_complete, String, callbackRemoveRobot, (num_robot,)))
	subscribers.append(rospy.Subscriber(topic_status_complete, RecvString, callbackRecvString, (num_robot,)))
	neighbors[getRobotName(num_robot)] = []

iteration_count = 0

waitEnterPress()
for num_robot in range(1, num_robots + 1):
	getID(num_robot)

try:
	
	while True:
	
		waitEnterPress()
	
		for num_robot in range(1, num_robots + 1):
			msg = NavSatFix()
			msg.header.stamp = rospy.Time.now()
			(x,y,z) = robots[num_robot-1]['positions'][iteration_count]
			print getRobotName(num_robot) + ": publishing position (" + str(x) + ", " + str(y) + ", " + str(z) + ")"
			msg.latitude = x
			msg.longitude = y
			msg.altitude = z
			publishers[num_robot-1]['pos'].publish(msg)
		iteration_count += 1
		
		waitEnterPress()

		for num_robot in range(1, num_robots + 1):
			dest = mc_prefix + str(num_robot)
			data = "status from " + getRobotName(num_robot)
			sendStatus(dest, data, num_robot)		
	
except Exception as e:
	print "\nException has been caught: " + str(e)
	print "Stopping subscribers"
	for subscriber in subscribers:
		subscriber.unregister()
	print "Exit, bye"
