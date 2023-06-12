#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from exam_prep.srv import demo_srv, demo_srvRequest, demo_srvResponse
from actionlib import SimpleActionServer, SimpleActionClient
from exam_prep.msg import multinacciAction, multinacciGoal, multinacciFeedback, multinacciResult


class demo_node():
    def __init__(self, pub_topic, sub_topic):

        # simple pub and sub
        self.pub = rospy.Publisher(pub_topic, String, queue_size=1)
        self.sub = rospy.Subscriber(sub_topic, String, self.pub_sub_cb)

        # simple service server and client
        self.srv = rospy.Service("multiply_ints", demo_srv, self.handle_srv)
        rospy.wait_for_service("multiply_ints")
        self.client = rospy.ServiceProxy("multiply_ints", demo_srv)

        # action server and client
        self.act_srv = SimpleActionServer("multinacci", multinacciAction, self.handle_act, auto_start=False)
        self.act_client = SimpleActionClient("multinacci", multinacciAction)
        
    def pub_sub_cb(self, string):
        # string = String(string)     # this line is not necessary but it allows vscode to autocomplete the methods + variables
        # print the recieved message and republish to another topic
        rospy.loginfo("%s", string.data)
        self.pub.publish(string)

    def handle_srv(self, req : demo_srvRequest):
        # the "type hints" let me use the fields from the class but this feature wont work in python2
        result = req.a * req.b
        res = demo_srvResponse()
        res.result = result
        return result
    
    def client_srv(self, a, b):
        req = demo_srvRequest(a, b)
        res = self.client.call(req)
        # res = self.client(a, b)   # couldnt get this to work, just use the "call" method
        rospy.loginfo("%s", res.result)

    def handle_act(self, goal):
        feedback = multinacciFeedback()
        result = multinacciResult()
        seq = [1,2]
        for i in range(1, goal.order):
            if self.act_srv.is_preempt_requested():
                self.act_srv.set_preempted()
                return
            seq.append(seq[i]*seq[i-1])
            feedback.sequence = seq
            self.act_srv.publish_feedback(feedback)
        result = feedback
        self.act_srv.set_succeeded(result)

    def client_act(self, order):
        goal = multinacciGoal(order)
        self.act_client.send_goal(goal)
        self.act_client.wait_for_result()
        result = self.act_client.get_result()
        rospy.loginfo("%s", str(result))



if __name__ == "__main__":
    rospy.init_node("demo_node")
    n = demo_node("publish", "subscribe")
    rospy.spin()