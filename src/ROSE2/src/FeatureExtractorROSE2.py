#!/usr/bin/env python
import rospy
import tf
from rose2.msg import ROSEFeatures
from rose2.msg import ROSE2Features
from jsk_recognition_msgs.msg import PolygonArray
import rose2.srv
from util import MsgUtils as mu
from rose_v2_repo import minibatch, parameters
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from PIL import Image as Img
from nav_msgs.msg import OccupancyGrid
import warnings
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

"""
Subscribers: /features_ROSE (rose2/ROSEFeatures)
Publishers: /features_ROSE2 (rose2/ROSE2Features)
            /extended_lines (visualization_msgs/Marker)
            /edges (visualization_msgs/MarkerArray)
            /rooms (jsk_recognition_msgs/PolygonArray)
Service:    /features_ROSESrv (nav_msgs/ROSE2Features)
params:     /pub_once -> publish once or or keep publishing at a certain rate
            #rose2 PARAMS
            /spatialClusteringLineSegmentsThreshold 
            /lines_th1 #real number 0 to 1 (0 keeps all the lines)
            /lines_distance
            /edges_th #real number 0 to 1
            #voronoi params
            #find more rooms with voronoi method (slower)
            /rooms_voronoi -> True if you want to use this method
            /filepath
            /voronoi_closeness
            /voronoi_blur
            /voronoi_iterations

This node has to be started after ROSE node (with param pub_once=True) which gives a filtered map and its main directions.
ROSE2 node applies ROSE2 method to the filtered map, finds walls and rooms of the map.
After computing the result, ROSE2 publishes ROSE2Features: edges, extended lines, rooms and contours 

"""
class FeatureExtractorROSE2:
    def __init__(self):
        rospy.init_node('ROSE2')
        rospy.loginfo("[ROSE2] waiting for features...")

        ### ROS PARAMS ###
        self.pubOnce = rospy.get_param("ROSE/pub_once2", True)
        self.spatialClusteringLineSegmentsThreshold = rospy.get_param("ROSE/spatialClusteringLineSegmentsThreshold", 5)
        #extended lines parameters
        self.lines_th1 = rospy.get_param("ROSE/lines_th1", 0.1)
        self.distance_extended_segment = rospy.get_param("ROSE/lines_distance", 20)
        # Edges parameters
        self.threshold_edges = rospy.get_param("ROSE/edges_th", 0.1)
        #voronoi params
        self.rooms_voronoi = rospy.get_param("ROSE/rooms_voronoi", True)
        self.filepath = rospy.get_param("ROSE/filepath", "/home/oseasy/YOEO_ws/maps/")
        self.voronoi_closeness = rospy.get_param("ROSE/voronoi_closeness", 10) #10
        self.blur = rospy.get_param("ROSE/voronoi_blur", 8)
        self.iterations = rospy.get_param("ROSE/voronoi_iterations", 5)

        #store ros params into param obj
        self.param_obj = parameters.ParameterObj()
        self.param_obj.voronoi_closeness = self.voronoi_closeness
        self.param_obj.blur = self.blur
        self.param_obj.iterations = self.iterations
        self.param_obj.spatialClusteringLineSegmentsThreshold = self.spatialClusteringLineSegmentsThreshold
        self.param_obj.distance_extended_segment = self.distance_extended_segment
        self.param_obj.th1 = self.lines_th1
        self.param_obj.threshold_edges = self.threshold_edges

        #features storing variables
        self.features = None #rose2/ROSEFeatures
        self.features2 = None #rose2/ROSE2Features
        self.cleanMap = None #np.array
        self.originalOccMap = None #np.array
        self.origin = None #map origin
        self.res = None #map res

        self.pubRate = 1

        #Flow control variables
        self.publish = False # True if this node has new features to publish

        #SUBSCRIBER
        rospy.Subscriber('features_ROSE', ROSEFeatures, self.processMap, queue_size=1, buff_size=2 ** 29)

        #PUBLISHERS
        self.pubFeatures = rospy.Publisher('features_ROSE2', ROSE2Features, queue_size=1)
        self.pubLines = rospy.Publisher('extended_lines', Marker, queue_size=1)
        self.pubEdges = rospy.Publisher('edges', MarkerArray, queue_size=1)
        self.pubRooms = rospy.Publisher('rooms', PolygonArray, queue_size=1)
        self.pub_final_map = rospy.Publisher('final_map', Image, queue_size=1)
        self.pub_semantic_grid = rospy.Publisher('semantic_grid', OccupancyGrid, queue_size=1)

        #Service
        rospy.Service('ROSE2Srv', rose2.srv.ROSE2, self.featuresSrv)



    def run(self):
        tf_broadcaster = tf.TransformBroadcaster()
        r = rospy.Rate(self.pubRate)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            # 发布map到robot1/map的tf变换
            tf_broadcaster.sendTransform(
            (0, 0, 0),                     # 平移信息
            (0, 0, 0, 1),                  # 旋转信息
            current_time,                  # 时间戳
            "map",                  # 子frame名称
            "robot1/map"                          # 父frame名称
            )
            rospy.sleep(0.1)  # 等待一段时间再发布下一个tf变换
            if self.publish:
                self.publishFeatures()
                if self.pubOnce:
                    self.publish = False
            r.sleep()

    #CALLBACK
    def processMap(self, features):
        self.features = features
        self.cleanMap = mu.fromOccupancyGridRawToImg(features.cleanMap)
        self.originalMap = mu.fromOccupancyGridToImg(features.originalMap)
        self.directions = list(features.directions)
        self.param_obj.comp = self.directions
        self.rose2 = minibatch.Minibatch()
        self.rose2.start_main(parameters, self.param_obj, self.cleanMap, self.originalMap, self.filepath, self.rooms_voronoi)
        self.makeFeaturesMsg()
        self.publish = True
        rospy.loginfo('[ROSE2] publishing edges, lines, contour and rooms')

    #SERVICE RESPONSE
    def featuresSrv(self, features):
        if(self.publish is False):
            self.processMap(features)
        return self.features2

    def makeFeaturesMsg(self):
        self.line_markers = Marker()
        self.edge_markers = MarkerArray()
        self.room_polygons = PolygonArray()
        self.ros_image = Image()
        edgesRviz = []
        roomsRviz = []
        linesMsg = []
        edgesMsg = []
        roomsMsg = []
        contourMsg = None
        self.origin = self.features.originalMap.info.origin
        self.res = self.features.originalMap.info.resolution
        extended_lines = self.rose2.extended_segments_th1_merged.copy()
        edges = self.rose2.edges_th1.copy()
        #Lines Marker and ExtendedLine messages
        self.line_markers = mu.make_lines_marker(extended_lines, self.origin, self.res)
        for l in self.rose2.extended_segments_th1_merged:
            linesMsg.append(mu.make_extendedline_msg(l))
        i = 0
        # Edge MarkerArray and Edge messages
        for e in edges:
            edgesRviz.append(mu.make_edge_marker(e, i, self.origin, self.res))
            edgesMsg.append(mu.make_edge_msg(e))
            i += 1
        #Room PolygonArray and Room messages
            
        if self.rose2.rooms_th1 is not None:
            labels = []
            for r in self.rose2.rooms_th1:
                roomsMsg.append(mu.make_room_msg(r))
                roomsRviz.append(mu.make_room_polygon(r, self.origin, self.res))
                labels.append(1)
            contourMsg = mu.make_contour_msg(self.rose2.vertices)
            self.room_polygons.polygons = roomsRviz
            self.room_polygons.header.frame_id = "map"
            self.room_polygons.labels = labels

            #---------------------final_map_process--------------------------#
            bridge = CvBridge()
            image_path = '/home/oseasy/YOEO_ws/maps/8b_rooms_th1.png'
            self.final_map = cv2.imread(image_path,cv2.IMREAD_UNCHANGED)
            if self.final_map is None:
                rospy.logerr('failed to read the final img')
            self.ros_image = bridge.cv2_to_imgmsg(self.final_map, encoding='passthrough')
            self.ros_image.header.frame_id = "map"
            self.final_map_2 = Img.open(image_path)
            self.pix_data_final_map2 = self.final_map_2.load()
            self.semantic_grid = mu.fromImgMapToOccupancyGrid(self.pix_data_final_map2,self.final_map_2,self.features.originalMap.info.origin,self.features.originalMap.info.resolution)

            self.semantic_grid.data = np.clip(self.semantic_grid.data, -128, 127).astype(np.int8).tolist()
           # print("Data min:", min(self.semantic_grid.data))
           # print("Data max:", max(self.semantic_grid.data))

        self.features2 = ROSE2Features()
        self.edge_markers.markers = edgesRviz
        self.features2.rooms = roomsMsg
        self.features2.contour = contourMsg
        self.features2.lines = linesMsg
        self.features2.edges = edgesMsg



    def publishFeatures(self):
        self.pubEdges.publish(self.edge_markers)
        self.pubLines.publish(self.line_markers)
        self.pubRooms.publish(self.room_polygons)
        self.pubFeatures.publish(self.features2)
        self.pub_final_map.publish(self.ros_image)
        self.pub_semantic_grid.publish(self.semantic_grid)
        

if __name__ == '__main__':
    f = FeatureExtractorROSE2()
    warnings.filterwarnings("ignore")
    f.run()
