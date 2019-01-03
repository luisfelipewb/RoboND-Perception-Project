# Project: Perception Pick & Place

[//]: # (Image References)
[img_overview]: ./images/overview.png
[img_training_a]: ./images/training_a.png
[img_training_b]: ./images/training_b.png
[img_result_1]: ./images/result_1.png
[img_result_2]: ./images/result_2.png
[img_result_3]: ./images/result_3.png
[img_pipe_01]: ./images/pipe_01.png
[img_pipe_02]: ./images/pipe_02.png
[img_pipe_03]: ./images/pipe_03.png
[img_pipe_04]: ./images/pipe_04.png
[img_pipe_05]: ./images/pipe_05.png
[img_pipe_06]: ./images/pipe_06.png
[img_pipe_07]: ./images/pipe_07.png
[img_pick_5]: ./images/pick5.png
[img_confusion_500]: ./images/confusion_500.png
[img_confusion_500_norm]: ./images/confusion_500_norm.png
[img_]: ./images/X.png
[gif_pipeline]: ./images/pipeline.gif
[yaml1]: ./results/output_1.yaml
[yaml2]: ./results/output_2.yaml
[yaml3]: ./results/output_3.yaml


## Project Report

The project uses PR2 robot with an RGB-D camera in a ROS + Gazebo simulated environment. The main goal is to implement a perception pipeline to recognize a list of objects on top of a table. The information from the recognized objects is used to prepare a pick and place list that can be used by the robot to separate the items in two drop boxes. This report presents an explanation of the perception pipeline implemented, including some comments, considerations, and future work.

![][img_overview]

## Perception Pipeline

The perception pipeline is a series of steps to process information collected from sensors and allow the robot to understand the environment. In this project, the perception pipeline had the purpose to identify the objects on top of the table using an RGB-D camera.

![][gif_pipeline]

Each step of the perception pipeline implemented is briefly described below.

### Passthrough filter
In this project, we are only interested in the objects located on top of the table. To remove all other points the pass-through filter is used. Basically, it crops the point cloud in 3D space. The code was based on previous exercises and was necessary to adjust to the hight of the table. An additional passthrough filter was implemented in the z-axis to remove the corner of the dropboxes perceived by the camera.

```python
passthrough = cloud_filtered.make_passthrough_filter()
passthrough.set_filter_field_name('z')
passthrough.set_filter_limits(0.6,1.2)
cloud_filtered = passthrough.filter()

passthrough = cloud_filtered.make_passthrough_filter()
passthrough.set_filter_field_name('y')
passthrough.set_filter_limits(-0.5,0.5)
cloud_filtered = passthrough.filter()
```

Before            |  After           |
:----------------:|:----------------:|
![][img_pipe_01]  | ![][img_pipe_02] |

### Statistical Outlier Filtering
This filter is used to remove the noise from the camera. It is based on the statistical distribution of the average distance between neighbor points in the cloud. The threshold of `1.2` was used but also removed some valid data from the objects. One option would be to reduce to 1 or less, but since it did not interfere with the results, this small loss was accepted.

```python
outlier_filter = cloud_filtered.make_statistical_outlier_filter()
outlier_filter.set_mean_k(50)
outlier_filter.set_std_dev_mul_thresh(1.2)
cloud_filtered = outlier_filter.filter()

```

Before            |  After           |
:----------------:|:----------------:|
![][img_pipe_02]  | ![][img_pipe_03] |

### Voxel Grid Downsampling

The amount of points to be processed is high and demands high computation resources. In order to achieve better performance, the number of points in the cloud is reduced using Voxel Grid Downsampling. In this project, it was used `0.005m` leaf size. It allowed keeping a good resolution while achieving reasonable performance.

```python
vox = cloud_filtered.make_voxel_grid_filter()
vox.set_leaf_size(0.005,0.005,0.005)
cloud_filtered = vox.filter()
```
Before            |  After           |
:----------------:|:----------------:|
![][img_pipe_03]  | ![][img_pipe_04] |

### RANSAC Plane Segmentation
Next, we need to identify and remove the surface of the table. Since the geometric shape of the table is a simple plane, the RANSAC filter is a good option to identify it. In this project, the parameter for maxim distance was set to `0.02m`.

```python
seg = cloud_filtered.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_distance_threshold(0.02)
```

Before            |  After           |
:----------------:|:----------------:|
![][img_pipe_04]  | ![][img_pipe_05] |

### Euclidean Clustering (DBSCAN)

The number of objects changes for each scenario presented in this project. Moreover, all points belonging to the same object are geometrically close. Therefore Euclidean Clustering (also known as DBSCAN) is a good option for a segmenting algorithm. The `ClusterTolerance` parameter was set to `0.05` with a minimum of `10` points per cluster. Maximum cluster size was not defined, to prevent the algorithm to split the same object into multiple clusters.

```python
ec = white_cloud.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance(0.05)
ec.set_MinClusterSize(10)
ec.set_SearchMethod(tree)
cluster_indices = ec.Extract()
```

Before            |  After           |
:----------------:|:----------------:|
![][img_pipe_05]  | ![][img_pipe_06] |


### Classification

After each cluster was distinguished, it is time to identify which object it represents. For this purpose, the SVM algorithm is used. The classifier must be previously trained, as explained in the next section. To use the classifier model, two features need to be extracted from each object: The color histogram and the normal distribution. These features are used by the SVM to make a prediction. To implement this task, the [`scikit-learn`](https://scikit-learn.org/stable/) library was very handy. The version used was 0.19.2
```python
feature = np.concatenate((chists, nhists))
prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
label = encoder.inverse_transform(prediction)[0]
```

Before            |  After           |
:----------------:|:----------------:|
![][img_pipe_06]  | ![][img_pipe_07] |


### Perception Pipeline Results
A significant performance improvement was achieved by changing the order which the point cloud is filtered. By first using passthrough filter to select only the tabletop area, it was possible to drastically reduce the time needed to complete segmentation. This avoided calculation for statical outlier removal and voxel downsampling in several points such as the floor for example.

```shell
#Initial
Callback elapsed time: 17.1248950958
Callback elapsed time: 17.2118489742
Callback elapsed time: 17.3884019852
Callback elapsed time: 17.0263290405

#Using Passthrough filter first
Callback elapsed time: 8.66569495201
Callback elapsed time: 8.78525209427
Callback elapsed time: 9.42934083939
Callback elapsed time: 8.46423411369
```

For the three world scenarios, all objects were correctly identified.
When exploring different object positions, sometimes the glue was misclassified, especially when it was mostly hidden behind other objects, but it did not affected the project results.

World 1            |  World 2          | World 3           |
:-----------------:|:-----------------:|:-----------------:|
![][img_result_1]  | ![][img_result_2] | ![][img_result_3] |


## Training model 

As previously mentioned, a machine learning algorithm called Support Vector Machine (SVM) is used to recognize the objects. First, it is necessary to train the SVM model with enough data. This was done using the [`capture_features.py`](./pr2_robot/training/capture_features.py) code from on previous exercises. The script uses the `sensor_stick` functions to extract the features from the objects in several different positions, as illustrated below.

![][img_training_a] | ![][img_training_b] | 
:------------------:|:-------------------:|

Those features are stored in the `training_set.sav` file and then used to train the classifier using the `./train_svm.py`  script. The script was modified to pass the training data path and output filename as parameters, besides that, it was necessary to adjust the list of models and increase the number of samples. The steps are described below.

```shell
roslaunch sensor_stick training.launch
cd RoboND-Perception-Project/pr2_robot/training/
rosrun pr2_robot capture_features.py
./train_svm.py training_set.sav model.sav
```
When the training is complete, the file `model.sav` is generated and is ready to be used in the perception pipeline.

### Training Model Results
To compare results, the model was trained with a different number of samples: 10, 100 and 500. It is clearly visible that the accuracy increases when more training data is provided in the learning process. As the results below demonstrate, it was possible to reach an accuracy of 98% when the sample number was 500. The downside was a long time required. In my machine, it took 83 minutes to collect training data and more than 120 minutes to train the classifier.

I also tried to change the kernel type from `linear` to `poly`, but the results were less accurate. In this project, the linear kernel provided the best accuracy.

```
10
Features in Training Set: 80
Invalid Features in Training set: 0
Accuracy: 0.82 (+/- 0.76)
accuracy score: 0.825

100
Features in Training Set: 800
Invalid Features in Training set: 0
Accuracy: 0.95 (+/- 0.42)
accuracy score: 0.95375

500
Features in Training Set: 4000
Invalid Features in Training set: 8
Scores: [ 1.  0.  1. ...,  1.  1.  1.]
Accuracy: 0.98 (+/- 0.28)
accuracy score: 0.979458917836
```
![][img_confusion_500]



## Pick and Place setup

The last part of the project was to parse all information perceived from the world and provide instructions for the robot to perform the pick and place operation. The key concept in this session was to underwent how data is structured in the ROS environment and be able to manipulate it.

```python
#Variable type definition
test_scene_num = Int32()
object_name = String()
arm_name = String()
pick_pose = Pose()
place_pose = Pose()

```
It was also important to check and understand the information available in some ROS parameters.
```shell
wolf@wolf-pc:RoboND-Perception-Project/pr2_robot/scripts$ rosparam get /object_list
- {group: green, name: biscuits}
- {group: green, name: soap}
- {group: red, name: soap2}

wolf@wolf-pc:RoboND-Perception-Project/pr2_robot/scripts$ rosparam get /dropbox 
- group: red
  name: left
  position: [0, 0.71, 0.605]
- group: green
  name: right
  position: [0, -0.71, 0.605]
```

To make easier to generate the yaml output files, the `pick_place_project.launch` file was modified to creade a ROS parameter that can be read in the `object_recognition.py` code. 

### Pick and Place setup Results
By executing the `object_recognition.py` code in each world scene, the following output yaml files were generated:

1. ![`output_1.yaml`][yaml1]
2. ![`output_2.yaml`][yaml2]
3. ![`output_3.yaml`][yaml3]

The task of building the collision map will be left for a future effort. Nevertheless, invoking the `pick_place_routine` with the parsed parameters, the robot was able to pick a few items, despite being very clumsy.
![][img_pick_5]


## Conclusions and future work
This was a fun project that gave me a really good overview of several techniques used in a perception pipeline and how to work with point cloud data. Moreover, it was a good opportunity to have a hands-on experience with ROS and how to integrate perception and motion planning modules.

In a future opportunity, I intend to continue to implement the proposed challenges. Besides that, other improvements that can be made on current implementation include:

* Several code performance optimizations. Especially in the perception pipeline and parsing the parameters.
* Use HSV color representation instead of RGB for better results in a realistic environment.
* Separate the perception code and the pick place routine in different ROS nodes for better code organization.
* Create better training data using images partially occluded to facilitate identification when objects are behind others
