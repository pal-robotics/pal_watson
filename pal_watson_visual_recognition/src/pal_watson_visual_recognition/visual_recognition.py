from watson_developer_cloud import VisualRecognitionV3
from pal_python.ros_image_utils import RosImageTmpFile

class PalWatsonVR:
    """
    Wrapper for Watson VisualRecognition API, performs common ROS conversions
    and should provide an immutable API in case Watson API changes in the future
    """
    def __init__(self, api_key):
        self.api = VisualRecognitionV3('2016-05-20', api_key=api_key)

    def classify_file(self, image_file):
        return self.api.classify(images_file=image_file)

    def classify_msg(self, image_msg):
        with RosImageTmpFile(image_msg) as f:
            return self.classify_file(image_file=f)


