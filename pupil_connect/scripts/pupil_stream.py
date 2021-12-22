#!/usr/bin/env python


import rospy
from cv_bridge import CvBridge, CvBridgeError

from pupil_msgs.msg import frame, gaze, pupil
from sensor_msgs.msg import Image

from msgpack import loads
import zmq
import cv2
import numpy as np


class PupilStreamer:

    FRAME_FORMAT = "bgr"

    # TODO: Receive argument from ros console for ip and port. Make launch file + parameter file?
    def __init__(self, topics, ip="127.0.0.1", pupil_port="50020"):
        self.pupil_pub = None
        self.gaze_pub = None
        self.frame_pub = None
        self.context = zmq.Context()
        self.addr = ip
        self.req_port = pupil_port
        self.req = self.context.socket(zmq.REQ)
        self.req.connect("tcp://{}:{}".format(self.addr, self.req_port))
        self.req.send_string("SUB_PORT")
        self.sub_port = self.req.recv_string()
        self.sub = self.context.socket(zmq.SUB)
        self.sub.connect("tcp://{}:{}".format(self.addr, self.sub_port))
        self.topics = topics
        self.cv_bridge = CvBridge()

    def subscribe(self):
        for topic in self.topics:
            self.sub.setsockopt_string(zmq.SUBSCRIBE, topic)
            if topic == 'frame':
                self.frame_pub = rospy.Publisher('pupil_frame', frame, queue_size=260)
            elif topic == 'gaze':
                self.gaze_pub = rospy.Publisher('pupil_gaze', gaze, queue_size=120)
            elif topic == 'pupil':
                self.pupil_pub = rospy.Publisher("pupil_pupil", pupil, queue_size=120)

    def publish(self):
        zmq_topic = self.sub.recv_string()
        zmq_message = self.sub.recv_multipart()

        payload = loads(zmq_message[0], raw=False)
        if len(zmq_message) == 2:
            extra_frame = [zmq_message[1]]
            payload["image_data"] = extra_frame

        if zmq_topic.startswith("gaze"):
            pass
        elif zmq_topic.startswith("pupil."):
            pass
        if zmq_topic.startswith("frame.world"):

            if payload["format"] != self.FRAME_FORMAT:
                print(f"different frame format ({payload['format']});")

            msg = frame()
            msg.topic = payload["topic"]
            msg.width = payload["width"]
            msg.height = payload["height"]
            msg.index = payload["index"]
            msg.timestamp = payload["timestamp"]
            msg.format = payload["format"]
            cv_image = np.frombuffer(payload["image_data"][0], dtype=np.uint8).reshape(payload["height"], payload["width"], 3)
            msg.image = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.frame_pub.publish(msg)



# Why use all the string versions of the socket commands?
# For non-ambiguous utf-8 encoding! Just stick to it
# context = zmq.Context()
# # open a req port to talk to pupil
# addr = "127.0.0.1"  # remote ip or localhost
# req_port = "50020"  # same as in the pupil remote gui
# req = context.socket(zmq.REQ)
# req.connect("tcp://{}:{}".format(addr, req_port))
# # ask for the sub port
# req.send_string("SUB_PORT")
# sub_port = req.recv_string()
#
# sub = context.socket(zmq.SUB)
# sub.connect("tcp://{}:{}".format(addr, sub_port))
#
#
# #sub.setsockopt_string(zmq.SUBSCRIBE, "pupil.")
# #sub.setsockopt_string(zmq.SUBSCRIBE, 'gaze')
# # sub.setsockopt_string(zmq.SUBSCRIBE, 'notify.')
# # sub.setsockopt_string(zmq.SUBSCRIBE, 'logging.')
# # or everything:
# sub.setsockopt_string(zmq.SUBSCRIBE, '')
# sub.setsockopt_string(zmq.SUBSCRIBE, 'frame.eye.1')

# Note to self:
# Unlike all the other messages, the frames receive 3 parts from the socket.
# First is topic, second is meta info and third the raw data.

# while True:
#     try:
#         zmq_topic = sub.recv_string()
#         zmq_hello = sub.recv_multipart()
#         # zmq_topic = zmq_hello[0].decode('UTF-8') # might be better to use recv_string for topic and then use recv_multipart for rest?
#         zmq_msg = loads(zmq_hello[0], raw=False)  # is a dictionary
#         if len(zmq_hello) is 2:
#             # zmq_msg["image_data"] = zmq_hello[2]
#             pass
#         print("\n{}: {}".format(zmq_topic, zmq_msg))
#
#         # topic = sub.recv_string()
#         # msg = sub.recv()
#         # msg = loads(msg, raw=False)
#         # print("\n{}: {}".format(topic, msg))
#     except KeyboardInterrupt:
#         break

if __name__ == '__main__':
    try:
        rospy.init_node('pupil_stream', anonymous=True)
        rate = rospy.Rate(500)
        pupil_topics = ["frame"]
        pupil_stream = PupilStreamer(pupil_topics)
        pupil_stream.subscribe()

        while not rospy.is_shutdown():
            pupil_stream.publish()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
