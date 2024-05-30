#!/usr/bin/env python
from unicodedata import name

from matplotlib.pyplot import autoscale
import rospy
import actionlib

import primer_ejemplo.msg


class FibonacciAction:
    _feedback = primer_ejemplo.msg.FibonacciFeedback()
    _result = primer_ejemplo.msg.FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
                            self._action_name, 
                            primer_ejemplo.msg.FibonacciAction, 
                            execute_cb=self.execute_cb, 
                            auto_start=False
                            )
        self._as.start()

    def execute_cb(self, goal : primer_ejemplo.msg.FibonacciGoal):
        r = rospy.Rate(1)
        success = True

        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        rospy.loginfo(f'{self._action_name}: Executing, creating fibonacci sequence of order {goal.order} with seeds 0, 1')

        for i in range(1, goal.order):
            if self._as.is_preempt_requested():
                rospy.loginfo(f'{self._action_name} Preempted')
                self._as.set_preempted()
                success = False
                break
            last0 = self._feedback.sequence[i]
            last1 = self._feedback.sequence[i-1]
            self._feedback.sequence.append(last0 + last1)

            self._as.publish_feedback(self._feedback)

            r.sleep()

        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo(f'{self._action_name}: Succeeded')
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('fibonacci_server')
    server = FibonacciAction('fibonacci')
    rospy.spin()