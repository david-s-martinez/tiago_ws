
#include <plane_segmentation/plane_segmentation.h>

PlaneSegmentation::PlaneSegmentation(
    const std::string& pointcloud_topic, 
    const std::string& base_frame) :
  pointcloud_topic_(pointcloud_topic),
  base_frame_(base_frame),
  is_cloud_updated_(false)
{
}

PlaneSegmentation::~PlaneSegmentation()
{
}

bool PlaneSegmentation::initalize(ros::NodeHandle& nh)
{
  //#>>>>TODO: subscribe to the pointcloud_topic_ and link it to the right callback

  //#>>>>TODO: advertise the pointcloud for of the table plane

  //#>>>>TODO: advertise the pointcloud for the remaining points (objects)

  // Subscribe to the pointcloud_topic_ and link it to the right callback
  point_cloud_sub_ = nh.subscribe(pointcloud_topic_, 1, &PlaneSegmentation::cloudCallback, this);

  // Advertise the pointcloud for the table plane
  plane_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("table_plane", 1);

  // Advertise the pointcloud for the remaining points (objects)
  objects_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("objects", 1);


  // Most PCL functions accept pointers as their arguments, as such we first set
  // initalize these pointers, otherwise we will run into segmentation faults...
  raw_cloud_.reset(new PointCloud);
  preprocessed_cloud_.reset(new PointCloud);
  plane_cloud_.reset(new PointCloud);
  objects_cloud_.reset(new PointCloud);

  return true;
}

void PlaneSegmentation::update(const ros::Time& time)
{
  // update as soon as new pointcloud is available
  if(is_cloud_updated_)
  {
    is_cloud_updated_ = false;

    //#>>>>Note: To check preProcessCloud() you can publish its output for testing
    // apply all preprocessing steps
    if(!preProcessCloud(raw_cloud_, preprocessed_cloud_))
      return;

    // segment cloud into table and objects
    if(!segmentCloud(preprocessed_cloud_, plane_cloud_, objects_cloud_))
      return;

    //#>>>>TODO: publish both pointclouds obtained by segmentCloud()
    plane_cloud_pub_.publish(plane_cloud_);
    objects_cloud_pub_.publish(objects_cloud_);
  }
}

bool PlaneSegmentation::preProcessCloud(CloudPtr& input, CloudPtr& output)
{
  //#>>>>Goal: Subsample and Filter the pointcloud

  //#>>>>Note: Raw pointclouds are typically to dense and need to be made sparse
  //#>>>>TODO: Down sample the pointcloud using VoxelGrid, save result in ds_cloud
  //#>>>>Hint: See https://pcl.readthedocs.io/projects/tutorials/en/master/voxel_grid.html#voxelgrid 
  //#>>>>TODO: Set useful parameters
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(input);
  sor.setLeafSize(0.01, 0.01, 0.01);
  sor.filter(*output);

  // CloudPtr ds_cloud(new PointCloud);            // downsampled pointcloud
  
  //#>>>>Note: Its allways a good idea to get rid of useless points first (e.g. floor, ceiling, walls, etc.)
  //#>>>>TODO: Transform the point cloud to the base_frame of the robot. (A frame with z=0 at ground level)
  //#>>>>TODO: Transform the point cloud to the base_frame and store the result in transf_cloud
  //#>>>>Hint: use pcl_ros::transformPointCloud

  CloudPtr transf_cloud(new PointCloud);        // transformed pointcloud (expressed in base frame)

  // Transform the point cloud to the base_frame link.
  // pcl_ros::transformPointCloud(base_frame_, *ds_cloud, *transf_cloud, tfListener_);
  pcl_ros::transformPointCloud(base_frame_, *output, *transf_cloud, tfListener_);
  //#>>>>TODO: Trim points lower than some z_min to remove the floor from the point cloud.
  //#>>>>Hint: use pcl::PassThrough filter and save result in output
  //#>>>>Hint: https://pcl.readthedocs.io/projects/tutorials/en/master/passthrough.html#passthrough
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(transf_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.01, std::numeric_limits<float>::max());
  pass.filter(*output);

  return true;
}

bool PlaneSegmentation::segmentCloud(CloudPtr& input, CloudPtr& plane_cloud, CloudPtr& objects_cloud)
{
  //#>>>>Goal: Remove every point that is not an object from the objects_cloud cloud

  // We will use Ransac to segment the pointcloud, here we setup the objects we need for this
  // pcl::SACSegmentation<PointT> seg;
  // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  // //#>>>>TODO: set parameters of the SACS segmentation
  // seg.setOptimizeCoefficients(true);
  // seg.setModelType(pcl::SACMODEL_PLANE);
  // seg.setMethodType(pcl::SAC_RANSAC);
  // seg.setMaxIterations(100);
  // //#>>>>TODO: set correct model, play with DistanceThreshold and the free outlier probability
  // seg.setDistanceThreshold(0.01);
  // //#>>>>TODO: then segment the input point cloud
  // seg.setInputCloud(input);
  // seg.segment(*inliers, *coefficients);
  // //#>>>>Note: Checkout the pcl tutorials on plane segmentation

  // //#>>>>TODO: save inliers in plane_cloud 
  // //#>>>>Note: These sould be point that belong to the table 
  // pcl::copyPointCloud(*input, inliers->indices, *plane_cloud);

  // //#>>>>TODO: save outliers in the objects_cloud
  // //#>>>>Note: This should be the rest
  // pcl::ExtractIndices<PointT> extract;
  // extract.setInputCloud(input);
  // extract.setIndices(inliers);
  // extract.setNegative(true);
  // extract.filter(*objects_cloud);
  
  // Setup SACSegmentation
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  // Set parameters of the SAC segmentation
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.02);

  // Segment the input point cloud
  seg.setInputCloud(input);

  try {
    seg.segment(*inliers, *coefficients);
  } catch (const pcl::PCLException &e) {
    // Handle the exception (print an error message, etc.)
    ROS_ERROR_STREAM("Error in SACSegmentation: " << e.what());
    return false;
  }

  // Check if any inliers were found
  if (inliers->indices.empty()) {
    ROS_WARN("SACSegmentation found no inliers. No plane could be segmented.");
    return false;
  }

  // Save inliers in plane_cloud (points belonging to the table)
  pcl::copyPointCloud(*input, inliers->indices, *plane_cloud);

  // Save outliers in the objects_cloud (rest of the points)
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(true);

  try {
    extract.filter(*objects_cloud);
  } catch (const pcl::PCLException &e) {
    // Handle the exception (print an error message, etc.)
    ROS_ERROR_STREAM("Error in ExtractIndices: " << e.what());
    return false;
  }
  
  // Next, we further refine the the objects_cloud by transforming it into the coordinate frame
  // of the fitted plane. In this transformed frame we remove everything below the table plane and 
  // everything more than 20 cm above the table.
  // Basically, a table aligned bounding box

  // if the plane fit is correct it will result in the coefficients = [nx, ny, nz, d]
  // where n = [nx, ny, nz] is the 3d normal vector perpendicular to the plane
  // and d the distance to the origin
  if(coefficients->values.empty())
    return false;

  //#>>>>TODO: extract the normal vector 'n' perpendicular to the plane and the scalar distance 'd'
  //#>>>>TODO: to the origin from the plane coefficions.
  //#>>>>Note: As always we use Eigen to represent vectors and matices 
  //#>>>>Note: https://eigen.tuxfamily.org/dox/GettingStarted.html
  Eigen::Vector3f n(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  double d = coefficients->values[3];
  
  // Now we construct an Eigen::Affine3f transformation T_plane_base that describes the table plane 
  // with respect to the base_link frame using n and d

  //#>>>>TODO: Build the Rotation (Quaterion) from the table's normal vector n
  //#>>>>TODO: And the floor (world) normal vector: [0,0,1]
  //#>>>>Hint: Use Eigen::Quaternionf::FromTwoVectors()
  Eigen::Quaternionf Q_plane_base = Eigen::Quaternionf::FromTwoVectors(n, Eigen::Vector3f::UnitZ());


  //#>>>>TODO: Build the translation (Vector3) from the table's normal vector n and distance
  //#>>>>TODO: to the origin 
  Eigen::Vector3f t_plane_base = n * static_cast<float>(d);

  // Finally we create the Homogenous transformation of the table
  Eigen::Affine3f T_plane_base = Eigen::Affine3f::Identity();
  T_plane_base.rotate(Q_plane_base.toRotationMatrix());
  T_plane_base.translate(t_plane_base);
  
  //#>>>>TODO: Transform the objects_cloud into the table frame and store in transf_cloud
  //#>>>>Hint: Use the function pcl::transformPointCloud() with T_plane_base as input
  //#>>>>https://pcl.readthedocs.io/projects/tutorials/en/latest/matrix_transform.html
  CloudPtr transf_cloud(new PointCloud);
  pcl::transformPointCloud(*objects_cloud, *transf_cloud, T_plane_base.inverse());
  //#>>>>TODO: filter everything directly below the table and above it (z > 0.01 && < 0.15) 
  //#>>>>Hint: using pcl::PassThrough filter (same as before)
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(transf_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.01, 0.15);
  pass.filter(*objects_cloud);
  // CloudPtr filterd_cloud(new PointCloud);

  //#>>>>TODO: transform back to base_link frame using the inverse transformation
  //#>>>>TODO: and store result in objects_cloud. Object cloud should only contain points associated to objects
  //#>>>>Hint: Eigen::Affine3f has an inverse function
  // pcl::transformPointCloud(*objects_cloud, *transf_cloud, T_plane_base);
  objects_cloud = transf_cloud;

  return true;
}

void PlaneSegmentation::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // convert ros msg to pcl raw_cloud
  is_cloud_updated_ = true;

  //#>>>>TODO: Convert the msg to the internal variable raw_cloud_ that holds the raw input pointcloud 
  //#>>>>Hint: pcl::fromROSMsg() can do the job
  pcl::fromROSMsg(*msg, *raw_cloud_);
  is_cloud_updated_ = true;
}
