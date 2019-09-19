#!/usr/bin/env python
#-*-coding: utf-8 -*-
import numpy as np
import cv2
import sys
import os

import six.moves.urllib as urllib
import tarfile
import tensorflow as tf

import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TwistStamped

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util




def callback(image_msg):
    global image_np
    bridge = CvBridge()
    image_np1 = bridge.imgmsg_to_cv2(image_msg)
    image_np = image_np1[:,:,:3].copy()

data2=None
def callback_kinect(data):
    global data2
    data2=data


def read_depth(width, height):
    global data2
    data_out = pc2.read_points(data2, field_names = ("x", "y", "z"), skip_nans=False, uvs=[[width,height]])
    return next(data_out)



rospy.init_node('object_detect',anonymous=True)
rospy.Subscriber('/camera/rgb/image_rect_color',Image ,callback)
rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback_kinect,queue_size=3)
coordinat_park = rospy.Publisher("/park_geometry",TwistStamped,queue_size=3)
coordinat_yaya = rospy.Publisher("/yaya_geometry",TwistStamped,queue_size=3)


#Object detection imports
sys.path.append("..")




#Model preparation 
#What model to download.
MODEL_NAME = 'traffic_v1_inference_graph_ssd'
#Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'
#List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('config', 'labelmap.pbtxt')
NUM_CLASSES = 6


#Load a (frozen) Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')


#Loading label map
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

with detection_graph.as_default():
  with tf.Session(graph=detection_graph) as sess:
    while not rospy.is_shutdown():
      global image_np
      #ret, image_np=cap.read()
      rows = 480#image_np.shape[0]
      cols = 640#image_np.shape[1]
      
      image_np_expanded = np.expand_dims(image_np, axis=0)
      image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
      # Each box represents a part of the image where a particular object was detected.
      boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
      # Each score represent how level of confidence for each of the objects.
      # Score is shown on the result image, together with the class label.
      scores = detection_graph.get_tensor_by_name('detection_scores:0')
      classes = detection_graph.get_tensor_by_name('detection_classes:0')
      num_detections = detection_graph.get_tensor_by_name('num_detections:0')
      
      # Actual detection.
      (boxes, scores, classes, num_detections) = sess.run(
          [boxes, scores, classes, num_detections],
          feed_dict={image_tensor: image_np_expanded})

     
      for i in range(num_detections):        
        if scores[:,i] > 0.5:
            for bbox in boxes[:,i,:]:

                x = bbox[1] * cols
                y = bbox[0] * rows
                right = bbox[3] * cols
                bottom = bbox[2] * rows
                
                cv2.rectangle(image_np, (int(x), int(y)), (int(right), int(bottom)), (125, 255, 51), thickness=5)

                index = np.squeeze(classes).astype(np.int32)[i]
                index_name = category_index[index]['name']

                text = index_name + ": " + str(round(np.squeeze(scores)[i],2))+"%"
                cv2.putText(image_np,text,(int(x),int(y-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(125, 255, 51),1)#,cv2.LINE_AA)



                if index_name == 'park':
                    derinlik_x=int((x+right)/2)
                    derinlik_y=int((y+bottom)/2)
                    derinlik_park = read_depth(derinlik_x, derinlik_y)
                    msg_park=TwistStamped()
                    msg_park.header.frame_id=index_name
                    print (msg_park.header.frame_id)
                    msg_park.twist.linear.x = derinlik_park[0]
                    msg_park.twist.linear.y = derinlik_park[1]
                    msg_park.twist.linear.z = derinlik_park[2]
                    msg_park.twist.angular.x = derinlik_x
                    msg_park.twist.angular.y = derinlik_y
                    msg_park.twist.angular.z = 0
                    coordinat_park.publish(msg_park)
                    cv2.circle(image_np, (derinlik_x,derinlik_y), 5, (0,0,255), -1)


                elif index_name == 'yaya':
                    derinlik_x=int((x+right)/2)
                    derinlik_y=int((y+bottom)/2)
                    derinlik_yaya = read_depth(derinlik_x, derinlik_y)
                    msg_yaya=TwistStamped()
                    msg_yaya.header.frame_id=index_name
                    print (msg_yaya.header.frame_id)
                    msg_yaya.twist.linear.x = derinlik_yaya[0]
                    msg_yaya.twist.linear.y = derinlik_yaya[1]
                    msg_yaya.twist.linear.z = derinlik_yaya[2]
                    msg_yaya.twist.angular.x = derinlik_x
                    msg_yaya.twist.angular.y = derinlik_y
                    msg_yaya.twist.angular.z = 0
                    coordinat_yaya.publish(msg_yaya)


                    cv2.circle(image_np, (derinlik_x,derinlik_y), 5, (0,0,255), -1)
                
                
                
      #image = cv2.resize(image_np, None, fx=(1.0/2.0), fy=(1.0/2.0), interpolation=cv2.INTER_AREA)
      cv2.imshow('object detection', image_np) 
      if cv2.waitKey(25) & 0xFF == 27:
       	cv2.destroyAllWindows()
       	break
        


