from watson_developer_cloud import VisualRecognitionV3
from pal_python.ros_image_utils import RosImageTmpFile
import os
import rospy

def get_api_key():
    api_key = rospy.get_param("~api_key", None)
    if not api_key:
        default_api_key_file = os.getenv("HOME") + "/.pal/watson_visual_recog_api_key.txt"
        api_key_file = rospy.get_param("~api_key_file", default_api_key_file)
        if not os.path.isfile(api_key_file):
            rospy.logerr("No api_key param exists and file \"{}\" does not exist, can't operate without a key".format(api_key_file))
            return None
        else:
            with open(api_key_file, "r") as f:
                api_file_contents = f.read()
                api_key = api_file_contents.splitlines()[0]
    return api_key


class PalWatsonVR:
    """
    Wrapper for Watson VisualRecognition API, performs common ROS conversions
    and should provide an immutable API in case Watson API changes in the future
    """
    def __init__(self, api_key):
        # If I don't hard code this URL, it fails when run on some computers
        url = 'https://access.alchemyapi.com/visual-recognition/api'
        self.api = VisualRecognitionV3('2016-05-20', api_key=api_key, url=url)

    def classify_file(self, image_file):
        return self.api.classify(images_file=image_file)

    def classify_file_path(self, image_path):
        with open(image_path, "r") as f:
            return self.api.classify(images_file=f)

    def classify_msg(self, image_msg):
        with RosImageTmpFile(image_msg) as f:
            return self.classify_file(image_file=f)


