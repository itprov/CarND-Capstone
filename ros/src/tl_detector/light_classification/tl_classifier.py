from __future__ import division

import cv2
import numpy as np
import glob
import tensorflow as tf
from PIL import Image
from keras import backend as K
from keras.models import load_model
from keras.layers import Input
from styx_msgs.msg import TrafficLight

from yolo3.model import yolo_eval

class TLClassifier(object):
    def __init__(self):
        # Initialize YOLO detection parameters
        self.detection_threshold = 0.35
        self.iou_threshold = 0.45
        self.in_width = 416
        self.in_height = 416
        self.scale = 1/255
        self.num_classes = 3 # Classes: RED = 0, YELLOW = 1, GREEN = 2

        model_path = '/capstone/ros/src/tl_detector/light_classification/traffic_lights-tiny-model.h5'

        # YOLOv3-tiny anchors
        anchors = [10.,14.,  23.,27.,  37.,58.,  81.,82.,  135.,169.,  344.,319.]
        self.anchors = np.array(anchors).reshape(-1, 2)

        self.sess = K.get_session()
        self.model = load_model(model_path, compile=False)
        self.input_image_shape = K.placeholder(shape=(2, ))

        self.boxes, self.scores, self.classes = yolo_eval(self.model.output,
                        self.anchors, self.num_classes, self.input_image_shape,
                        score_threshold=self.detection_threshold,
                        iou_threshold=self.iou_threshold)
        self.graph = tf.get_default_graph()

    def letterbox_image(self, image, size):
        '''resize image with unchanged aspect ratio using padding'''
        iw, ih = image.size
        w, h = size
        scale = min(w/iw, h/ih)
        nw = int(iw*scale)
        nh = int(ih*scale)

        image = image.resize((nw,nh), Image.BICUBIC)
        new_image = Image.new('RGB', size, (128,128,128))
        new_image.paste(image, ((w-nw)//2, (h-nh)//2))
        return new_image

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Process image to prepare input
        image = Image.fromarray(image)
        boxed_image = self.letterbox_image(image, (self.in_width, self.in_height))
        image_data = np.array(boxed_image, dtype='float32')
        image_data *= self.scale
        # Add batch dimension
        image_data = np.expand_dims(image_data, 0)

        with self.graph.as_default():
            out_boxes, out_scores, out_classes = self.sess.run(
                [self.boxes, self.scores, self.classes],
                feed_dict={
                    self.model.input: image_data,
                    self.input_image_shape: [image.size[1], image.size[0]],
                    K.learning_phase(): 0
                })

        state = TrafficLight.UNKNOWN
        # In case of multiple traffic lights are detected in an image,
        # ensure a majority of the detected states match
        # Order of preference in case of more than 1 states detected equally:
        # Red, Yellow, Green
        if len(out_classes) > 0:
            state = np.argmax(np.bincount(out_classes))

        return state
