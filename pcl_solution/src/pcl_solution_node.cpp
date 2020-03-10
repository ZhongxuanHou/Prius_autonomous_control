#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/PointCloud2.h"
#include "vision_msgs/Detection3DArray.h"
#include "head_seven.hpp"

ros::Publisher pub;
// subscriber callback function
void callback(const sensor_msgs::PointCloud2& msg){
  vision_msgs::Detection3DArray output;
  //copy the header
  output.header = msg.header;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  //convert the ros message to desired pcl data type
  pcl::fromROSMsg(msg , *cloud);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.05f, 0.05f, 0.05f);
  vg.filter (*cloud_filtered);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.3);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      ROS_WARN_STREAM("Could not estimate a planar model for the given data");
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.5); // 2cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

 //extract 3D bounding box values from the cluster and publish them
  for (size_t i = 0; i < cluster_indices.size (); i++)
		{
			Eigen::Vector4f centroid, min, max;
      vision_msgs::Detection3D detection;
			pcl::compute3DCentroid (*cloud_filtered,cluster_indices[i], centroid);
      pcl::getMinMax3D (*cloud_filtered,cluster_indices[i],min,max);

      detection.bbox.center.position.x = centroid[0];
      detection.bbox.center.position.y = centroid[1];
      detection.bbox.center.position.z = centroid[2];
      detection.bbox.size.x = max[0] - min[0];
      detection.bbox.size.y = max[1] - min[1];
      detection.bbox.size.z = max[2] - min[2];
      detection.header = msg.header;
      output.detections.push_back(detection);
		}
    pub.publish(output);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv,"pcl_solution");

  ros::NodeHandle nd;

  //subscribing to point_cloud
  ros::Subscriber sub = nd.subscribe("/point_cloud",1,&callback);

  //publishing the detections
  pub = nd.advertise<vision_msgs::Detection3DArray>("/pcl_solution_node/detections", 1);

  ros::Rate rate(1);

  while(ros::ok()){
    ros::spin();
  }
}
