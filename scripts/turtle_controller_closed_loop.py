#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
import json
import math

class PID:
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D

        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
		
    def reset(self):
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
    
    def update(self, error):
        self.error = error
        self.integral = self.integral + error
        self.derivative = error - self.prev_error
        self.prev_error = error
        return self.P * self.error + self.I * self.integral + self.D * self.derivative
   
class TurtleController:
	def __init__(self):
		self.pub_cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		self.sub_pose = rospy.Subscriber('/turtle1/pose', Pose, self.onPose)
		self.subTasks = rospy.Subscriber('/turtle_command', String, self.onTasks)
		self.pose = Pose()
		
		self.anglePID = PID(3, 0, 1)
		self.distancePID = PID(1.4, 0, 0)

		self.msg = Twist()

	def onPose(self, data):                                                         
		self.pose = data
		if self.pose.theta < 0:
			self.pose.theta += 2*math.pi
	
	def onTasks(self, msg):
		if (len(msg.data) == 0):
			return
		tasks = self.parseJson(msg.data)
		print(tasks)
		for task in tasks:
			if hasattr(self, task['action']) and callable(getattr(self, task['action'])):
				method_to_call = getattr(self, task['action'])
				method_to_call(task['params'])

	def parseJson(self, inp_string):
		inp_string = inp_string.replace('\'', '"')
		inp_json = json.loads(inp_string)
		return inp_json

	def move(self, args):
		rospy.loginfo(f"Turtle Moving at speed of {args['speed']} in the {args['direction']} direction for {args['distance']} units")
		vel = Twist()
		t0 = rospy.Time.now().to_sec()
		dist_traveled = 0
		while (dist_traveled < float(args['distance'])):
			vel.linear.x = float(args['speed']) * pow(-1, args['direction'] == 'backward')
			self.pub_cmd_vel.publish(vel)
			t1 = rospy.Time.now().to_sec()
			dist_traveled = (t1 - t0) * float(args['speed'])
		vel.linear.x = 0
		self.pub_cmd_vel.publish(vel)

	def rotate(self, args):
		rospy.loginfo(f"Turtle Rotating at angular speed of {args['angular_speed']} in the {args['direction']} direction for {args['angle']} radians")
		vel = Twist()
		t0 = rospy.Time.now().to_sec()
		angleTraveled = 0
		vel.angular.z = float(args['angular_speed']) * pow(-1, args['direction'] == 'clockwise')
		while (angleTraveled < float(args['angle'])):
			self.pub_cmd_vel.publish(vel)
			t1 = rospy.Time.now().to_sec()
			angleTraveled = (t1 - t0) * float(args['angular_speed'])
		vel.angular.z = 0
		self.pub_cmd_vel.publish(vel)

	def rotateByTheta(self, args):
		if 'direction' not in args :
			args['direction'] = 'anticlockwise'
		rospy.loginfo(f"Turtle Rotating by theta {args['theta']} {args['direction']}")
		if args['direction'] == 'anticlockwise':
			goal_theta = (self.pose.theta + float(args['theta']) + 2*math.pi)%(2*math.pi)
		else:
			goal_theta = (self.pose.theta - float(args['theta']) + 2*math.pi)%(2*math.pi)
		vel = Twist()
		angle_distance = goal_theta - self.pose.theta
		while abs(angle_distance) >= 0.005:
			angle_distance = goal_theta - self.pose.theta
			if args['direction'] == 'anticlockwise':
				vel.angular.z = self.anglePID.update(abs(angle_distance))
			else :
				vel.angular.z = self.anglePID.update(-abs(angle_distance))
			self.pub_cmd_vel.publish(vel)
		vel.angular.z = 0
		self.pub_cmd_vel.publish(vel)
		
	def moveByDistance(self, args):
		rospy.loginfo(f"Turtle Moving by {args['distance']}")
		goal_x = self.pose.x + float(args['distance']) * math.cos(self.pose.theta)
		goal_y = self.pose.y + float(args['distance']) * math.sin(self.pose.theta)
		vel = Twist()
		distance = math.sqrt(math.pow(goal_x - self.pose.x, 2) + math.pow(goal_y - self.pose.y, 2))
		while abs(distance) >= 0.005:
			distance = math.sqrt(math.pow(goal_x - self.pose.x, 2) + math.pow(goal_y - self.pose.y, 2))
			vel.linear.x = self.distancePID.update(distance)
			self.pub_cmd_vel.publish(vel)
		vel.linear.x = 0
		self.pub_cmd_vel.publish(vel)

	def goToGoal(self, args):
		rospy.loginfo(f"Turtle Moving to x:{args['goal_x']} y:{args['goal_y']} theta:{args['goal_theta']}")
		self.anglePID.reset()
		self.distancePID.reset()
		self.R = math.sqrt(math.pow(self.pose.x - args['goal_x'] , 2) + math.pow(self.pose.y - args['goal_y'] , 2))
		self.xim = self.pose.x + self.R*math.cos(self.pose.theta)
		self.yim = self.pose.y + self.R*math.sin(self.pose.theta)

		self.C = math.sqrt(math.pow(self.xim - args['goal_x'] , 2) + math.pow(self.yim - args['goal_y'] , 2))
		if self.xim > args['goal_x']:
			self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
		else:
			self.alpha = 2*math.pi*math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))

		while self.R > 0.05 or self.alpha>0.005:
			self.R = math.sqrt(math.pow(self.pose.x - args['goal_x'] , 2) + math.pow(self.pose.y - args['goal_y'] , 2))

			self.xim = self.pose.x + self.R*math.cos(self.pose.theta)
			self.yim = self.pose.y + self.R*math.sin(self.pose.theta)

			self.C = math.sqrt(math.pow(self.xim - args['goal_x'] , 2) + math.pow(self.yim - args['goal_y'] , 2))
			
			if self.xim > args['goal_x']:
				self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
			else:
				self.alpha = 2*math.pi*math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))

			self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))

			self.angle_err = self.anglePID.update(self.alpha)
			self.distance_err = self.distancePID.update(self.R)
			self.msg.angular.z = self.angle_err
			self.msg.linear.x = self.distance_err

			self.pub_cmd_vel.publish(self.msg)

		self.msg.linear.x=0
		self.msg.angular.z=0
		self.pub_cmd_vel.publish(self.msg)
		
		# Angle correction
		angle_err_final = (float(args['goal_theta']) - self.pose.theta + 2*math.pi)%(2*math.pi)
		self.rotateByTheta({ 'theta': angle_err_final, 'direction': 'clockwise'})

		
def main():
    TurtleController()
    rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('turtlesim', anonymous=True)
		main()
	except rospy.ROSInterruptException:
		pass