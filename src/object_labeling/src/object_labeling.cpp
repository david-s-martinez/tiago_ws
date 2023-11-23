#include <object_labeling/object_labeling.h>

ObjectLabeling::ObjectLabeling(
    const std::string& objects_cloud_topic_, 
    const std::string& camera_info_topic,
    const std::string& camera_frame) :
  is_cloud_updated_(false),
  has_camera_info_(false),
  objects_cloud_topic_(objects_cloud_topic_),
  camera_info_topic_(camera_info_topic),
  camera_frame_(camera_frame),
  K_(Eigen::Matrix3d::Zero())
{
}

ObjectLabeling::~ObjectLabeling()
{
}

bool ObjectLabeling::initalize(ros::NodeHandle& nh)
{
  //#>>>>TODO: subscribe to objects pointcloud published by the plane_segmentation_node
  //#>>>>TODO: subscribe to bounding boxes from yolo (object_labeling_node)
  //#>>>>TODO: subscribe to camera info from robot to obtain the camera matrix K

  //#>>>>TODO: publish the labled objects as PointCloudl type (see typedefs in header)

  // publish the LABELED object names as visulaization marker (http://wiki.ros.org/rviz/DisplayTypes/Marker)
  text_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/text_markers", 1);

  // init internal pointclouds for processing (again pcl uses pointers)
  object_point_cloud_.reset(new PointCloud);    // holds unlabled object point cloud
  labeled_point_cloud_.reset(new PointCloudl);  // holds labled object point cloud

  //#>>>>TODO: setup a mapping from class names the ones given by yolo (see yolo bounding_boxes message for classes)
  //#>>>>TODO: to lables / ids (number from 1 to n) used by the pointcloud
  //#>>>>Note: We will use 0 as 'unkown' type 
  dict_["sports ball"] = 1;
  // ... bananna, cup, apple, ...

  return true;
}

void ObjectLabeling::update(const ros::Time& time)
{
  // camera info and point cloud available
  if(is_cloud_updated_ && has_camera_info_)
  {
    is_cloud_updated_ = false;

    // label the objects in pointcloud based on 2d bounding boxes 
    if(!labelObjects(object_point_cloud_, labeled_point_cloud_))
      return;

    //#>>>>TODO: publish labeled_point_cloud_ to ros

    //#>>>>TODO: publish text_markers_ to ros

  }
}

bool ObjectLabeling::labelObjects(CloudPtr& input, CloudPtrl& output)
{
  //#>>>>GOAL: Split input pointcloud into seperate blobs, compute centroid,
  //#>>>>GOAL: project centorid into the camera image and match with bounding box
  //#>>>>GOAL: finally label the pointcloud with object type

  // First we need to cluster the input cloud into seperated clusters,
  // each of them represents an object on the table.

  //#>>>>TODO: Use EuclideanClusterExtraction to seperate the pointcloud into clusters
  //#>>>>Hint: https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html?highlight=EuclideanClusterExtraction
  
  // holds the extracted cluster indices (just a integer for identifiction)
  std::vector<pcl::PointIndices> cluster_indices;

  //#>>>>TODO: Iterate over each cluster and compute its centroid point (= mean)
  //#>>>>TODO: Push the centroid into the vector of centroids
  std::vector<Eigen::Vector3d> centroids;

  // Next we need to find the pixel coordinates of the centroids within the 2d
  // camera image. This projection is handled by the camera matrix
  // First, the centorids need to be transformed from the pointcloud frame into the
  // camera frame.

  //#>>>>TODO: Get the homogenous transformation matrix of the base frame with respect
  //#>>>>TODO: to the camera frame.
  //#>>>>Hint: look up the transformation through the tf tree: tfListener_.lookupTransform(...)
  //#>>>>TODO: Convert the tf::StampedTransform into an Eigen::Affine3d
  //#>>>>Hint: tf::transformTFToEigen(...) can do the job
  Eigen::Affine3d T_base_camera; // = ?;

  //#>>>>TODO: Transform the centorids into the camera frame by multiplying them 
  //#>>>>TODO: with the transformation that takes a point in the pointcloud frame and turns it
  //#>>>>TODO: into a point in the camera frame. Do this for all centroids.
  std::vector<Eigen::Vector3d> centroids_camera;

  //#>>>>TODO: Project the transformed centorids into the camera plane by using the camera matrix K
  //#>>>>Hint: Multiplying a 3d vector with the 3x3 camera matrix gives a vector in R^3
  //#>>>>Hint: To get pixel coordinates in R^2 you need to convert them to homogenous 2d coordinates
  //#>>>>Hint: ( = divide by the last component and drop the one in third component.)
  std::vector<Eigen::Vector2d> pixel_centroids; // = ?

  // Now the centorids of each cluster are given as pixel coordinates in the 2d image
  // plane of the camera. What remains is to find the bounding box that matches to each of 
  // those controids.

  //#>>>>TODO: Find the best match between pixel_centroids and detections_
  //#>>>>Hint: Use the euclidian distance between the pixel_centroids and the boundingbox centers
  //#>>>>Hint: For each bounding box find the closest cenroid 
  //#>>>>TODO: If a cluster cant be matched (no bounding boxes left) assign 0 as label

  std::vector<int> assigned_labels(cluster_indices.size(), 0);                  // lables of each centroid
  std::vector<std::string> assigned_classes(cluster_indices.size(), "unknown"); // class names of each centroid

  for(size_t i = 0; i < detections_.size(); ++i)
  {
    // get the bounding box we want to find the closest cenroid 
    const darknet_ros_msgs::BoundingBox& bounding_box = detections_[i];

    //#>>>>TODO: For all cenroids compute the distance to the boudning box
    //#>>>>TODO: select the clostes as match and get its index in pixel_centroids
    int match; // = ?

    // remember the label of match
    if(dict_.find(bounding_box.Class) != dict_.end())
    {
      assigned_labels[match] = dict_[bounding_box.Class]; // set match to defined class index
      assigned_classes[match] = bounding_box.Class;       // set match to class name
    }
  }

  // relabel the point cloud
  output->points.clear();
  output->header = input->header;

  PointTl pt;
  i = 0;
  cit = cluster_indices.begin();
  for(; cit != cluster_indices.end(); ++cit, ++i ) 
  {
    // relabel all the points inside cluster
    std::vector<int>::const_iterator it = cit->indices.begin();
    for(; it != cit->indices.end(); ++it ) 
    {
      PointT& cpt = input->points[*it];
      pt.x = cpt.x;
      pt.y = cpt.y;
      pt.z = cpt.z;
      pt.label = assigned_labels[i];    // Note: To test clustering without the matching stuff just use pt.lable = i
      output->points.push_back( pt );
    }
  }

  // create a text marker that displays the assigned class name (assigned_classes) 
  // at the 3d position of the corresponding centroid
  text_markers_.markers.resize(assigned_classes.size());
  for(size_t i = 0; i < assigned_classes.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = assigned_classes[i];
    marker.pose.position.x = centroids.col(i).x();
    marker.pose.position.y = centroids.col(i).y();
    marker.pose.position.z = centroids.col(i).z() + 0.1;
    marker.color.a = 1.0;
    marker.scale.z = 0.1;
    marker.id = i;
    marker.header.frame_id = input->header.frame_id;
    marker.header.stamp = ros::Time::now();
    text_markers_.markers[i] = marker;
  }

  return true;
}


void ObjectLabeling::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // convert to pcl
  is_cloud_updated_ = true;
  //#>>>>TODO: convert to pcl and store in object_point_cloud_
  //#>>>>Hint: pcl::fromROSMsg()
}

void ObjectLabeling::detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
{
  //#>>>>TODO: copy the YOLO bounding boxes
  detections_; // = ?;
}

void ObjectLabeling::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  // copy camera info
  has_camera_info_ = true;
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();

  //#>>>>TODO: copy the 3x3 camera matrix to K_
  //#>>>>Hint: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  /*for(size_t i = 0; i < 9; ++i)
  {
    K(i) = msg->K[i];
  }
  K_ = K.transpose();*/
}