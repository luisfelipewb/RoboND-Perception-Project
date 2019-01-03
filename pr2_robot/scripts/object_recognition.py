#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

import time

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:
    t1 = time.time()

    # Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    #pcl.save(pcl_data, '01_original.pcd')
    cloud_filtered = pcl_data

    # PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(0.6,1.2)
    cloud_filtered = passthrough.filter()
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('y')
    passthrough.set_filter_limits(-0.5,0.5)
    cloud_filtered = passthrough.filter()
    #pcl.save(cloud_filtered, '02_passthrough_filtered.pcd')
    #pcl_passthrough_pub.publish(pcl_to_ros(cloud_filtered))

    # Statistical Outlier Filtering
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    outlier_filter.set_std_dev_mul_thresh(1.2)
    cloud_filtered = outlier_filter.filter()
    #pcl.save(cloud_filtered, '03_statistical_filtered.pcd')
    #pcl_statistical_pub.publish(pcl_to_ros(cloud_filtered))

    # Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    vox.set_leaf_size(0.005,0.005,0.005)
    cloud_filtered = vox.filter()
    #pcl.save(cloud_filtered, '04_voxel_downsampled.pcd')
    #pcl_downsampled_pub.publish(pcl_to_ros(cloud_filtered))

    # RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.02)

    # Extract inliers and outliers
    inliers, coefficients = seg.segment()
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    #pcl.save(cloud_table, '05_ransac_segmented_table.pcd')
    #pcl.save(cloud_objects, '05_ransac_segmented_objects.pcd')
    #pcl_objects_pub.publish(pcl_to_ros(cloud_objects))

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(10)
    #ec.set_MaxClusterSize(250)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    #pcl.save(cluster_cloud, '06_colored_cluster.pcd')

    # Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects_list = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        sample_cloud = pcl_to_ros(pcl_cluster)
        chists = compute_color_histograms(sample_cloud, using_hsv=False)
        normals = get_normals(sample_cloud)
        nhists = compute_normal_histograms(normals)

        # Compute the associated feature vector
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = sample_cloud
        detected_objects_list.append(do)

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects_list)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass
    t2 = time.time()
    print ("Callback elapsed time: ", str(t2-t1))

# function to load parameters and request PickPlace service
def pr2_mover(detected_object_list):

    # TODO: Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()

    yaml_dictionary_list =[]

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')
    test_scene_num.data = rospy.get_param('test_scene_num')

    # TODO: Parse parameters into individual variables
    for dropbox in dropbox_list_param:
        if dropbox['name'] == 'left':
            left_position = dropbox['position']
        if dropbox['name'] == 'right':
            right_position = dropbox['position']

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # Loop through the pick list
    for object_param in object_list_param:
        object_name.data = object_param['name']

        # Assign the arm to be used for pick_place
        # Create 'place_pose' for the object
        group = object_param['group']
        if group == 'green':
            arm_name.data = 'right'
            place_pose.position.x = right_position[0]
            place_pose.position.y = right_position[1]
            place_pose.position.z = right_position[2]
        elif group == 'red':
            arm_name.data = 'left'
            place_pose.position.x = left_position[0]
            place_pose.position.y = left_position[1]
            place_pose.position.z = left_position[2]
        else:
            print("Invalid group")

        # Get the PointCloud for a given object and obtain it's centroid
        for detected_object in detected_object_list:
            if detected_object.label == object_param['name']:
                points_arr = ros_to_pcl(detected_object.cloud).to_array()
                centroid = np.mean(points_arr, axis=0)[:3]
                pick_pose.position.x = np.asscalar(centroid[0])
                pick_pose.position.y = np.asscalar(centroid[1])
                pick_pose.position.z = np.asscalar(centroid[2])

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        dict_entry = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        yaml_dictionary_list.append(dict_entry)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)
            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    filename = "output_" + str(test_scene_num.data) + ".yaml"
    send_to_yaml(filename, yaml_dictionary_list)

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('object_recognition', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points/",pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    #pcl_passthrough_pub = rospy.Publisher("/pcl_passthrough", PointCloud2, queue_size=1)
    #pcl_statistical_pub = rospy.Publisher("/pcl_statistical", PointCloud2, queue_size=1)
    #pcl_downsampled_pub = rospy.Publisher("/pcl_downsampled", PointCloud2, queue_size=1)
    #pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('../training/model_500.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
