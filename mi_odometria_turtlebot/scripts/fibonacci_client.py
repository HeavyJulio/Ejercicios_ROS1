#!/usr/bin/env python
import rospy
import actionlib

import primer_ejemplo.msg

def fibonacci_client(name):
    client = actionlib.SimpleActionClient(name, primer_ejemplo.msg.FibonacciAction)
    client.wait_for_server()

    goal = primer_ejemplo.msg.FibonacciGoal()
    goal.order = 20

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()


if __name__ == '__main__':
    rospy.init_node('fibonacci_client')
    result = fibonacci_client('fibonacci')

    result_str = 'Result: '
    for n in result.sequence:
        result_str += str(n) + ', '
    rospy.loginfo(result_str)