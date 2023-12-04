#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from turtle_gpt.srv import GPTPrompt

def main():
    publisher = rospy.Publisher("turtle_command", String, queue_size=1)
    while not rospy.is_shutdown():
        inp = input("Enter the command for turtlesim (Enter q to exit): ")
        if inp == 'q':
            break

        rospy.wait_for_service('/GPTPrompt')
        GPTService = rospy.ServiceProxy('/GPTPrompt', GPTPrompt)
        res = GPTService(inp)
        result = String()
        result.data = res.response

        # result = String()
        # result.data = '''[
        #     {"action": "goToGoal", "params": { "goal_x" : 1, "goal_y": 1, "goal_theta": 1.57  }},
        #     {"action": "goToGoal", "params": { "goal_x" : 5, "goal_y": 9, "goal_theta": 2  }}
        # ]'''

        publisher.publish(result)


if __name__ == "__main__":
    rospy.init_node('PromptNode',anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        pass