#!/usr/bin/env python

import unittest
import rospy
import rospkg
from pal_watson_visual_recognition.visual_recognition import PalWatsonVR, get_api_key
class TestWatsonVR(unittest.TestCase):

    def __init__(self, *args):
        super(TestWatsonVR, self).__init__(*args)
        api_key = get_api_key()
        self.assertIsNotNone(api_key)
        self.watson = PalWatsonVR(api_key=api_key)

    def test_recognition(self):
        rpkg = rospkg.RosPack()
        img_path = rpkg.get_path("pal_watson_visual_recognition") + "/test/test_image.jpg"
        answer = self.watson.classify_file_path(img_path)
        self.assertTrue(answer.has_key('images'))
        self.assertEqual(len(answer['images']), 1)


if __name__ == '__main__':
    import rosunit
    rospy.init_node("test_watson_vr_api")
    rosunit.unitrun('test_watson_vr_api', 'test_watson_vr_api', TestWatsonVR)

