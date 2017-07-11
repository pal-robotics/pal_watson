#!/usr/bin/env python


import rospy
import actionlib
from pal_watson_visual_recognition.visual_recognition import PalWatsonVR, get_api_key
from pal_watson_visual_recognition.msg import ClassifyImageAction
import pal_interaction_msgs.msg
from sensor_msgs.msg import Image


class WatsonServer:
    """
    Watson action server that will recognize an image and use TTS to describe
    what has been recognized

    Subscribes to the image topic to capture an image when requested
    """
    def __init__(self, api_key, threshold=0.75):
        self.threshold = 0.75
        self.last_img = None
        self.pal_watson = PalWatsonVR(api_key)
        self.action_server = actionlib.SimpleActionServer("watson/classify_image", ClassifyImageAction, execute_cb=self.classify_cb, auto_start=False)
        self.image_sub = rospy.Subscriber("image", Image, self.image_cb)
        self.tts_client = actionlib.SimpleActionClient("tts",
                                                        pal_interaction_msgs.msg.TtsAction)
        rospy.loginfo("Waiting for TTS Server")
        self.tts_client.wait_for_server()
        rospy.loginfo("Connected to TTS Server")
        self.action_server.start()
        rospy.loginfo("Visual recognition server started")

    def classify_cb(self, goal):
        if not self.last_img:
            error = "No image received on topic {}".format(self.image_sub.resolved_name)
            rospy.logerr(error)
            self.action_server.set_aborted(text=error)
            return
        answer = self.pal_watson.classify_msg(self.last_img)
        if not answer.has_key('images') or len(answer['images']) == 0:
            error = "Invalid answer received from watson, missing image classification"
            rospy.logerr(error)
            self.action_server.set_aborted(text=error)
            return
        classifiers = answer['images'][0]['classifiers']
        recognized_classes = []
        for classifier in classifiers:
            for a_class in classifier['classes']:
                if a_class['score'] > self.threshold:
                    rospy.loginfo("{} score {}".format(a_class['class'], a_class['score']))
                    recognized_classes.append(a_class['class'])

        if recognized_classes:
            text = "I recognized: " + ", ".join(recognized_classes)
        else:
            text = "I didn't recognize anything"
        rospy.loginfo(text)
        self.say(text)
        self.action_server.set_succeeded()

    def say(self, text):
        goal = pal_interaction_msgs.msg.TtsGoal()
        goal.rawtext.lang_id = "en_GB"
        goal.rawtext.text = text
        self.tts_client.send_goal(goal)

    def image_cb(self, msg):
        self.last_img = msg


def main():
    rospy.init_node("watson_visual_recog_server")
    api_key = get_api_key()
    if not api_key:
        rospy.logerr("No api_key provided")
        return

    server = WatsonServer(api_key=api_key)
    rospy.spin()

if __name__ == "__main__":
    main()
