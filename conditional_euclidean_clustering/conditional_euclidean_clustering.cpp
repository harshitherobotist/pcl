#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
// Euclidean Cluster Extraction


ros::Publisher pub;

int 
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CentroidPoint<pcl::PointXYZ> centroid;

  pcl::io::loadPCDFile<pcl::PointXYZ> ("table_scene_lms400_objects.pcd", *cloud);

  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*


  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud); 


  std::vector<pcl::PointIndices> cluster_indices;      
   
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setInputCloud (cloud);         
  ec.setClusterTolerance (0.02);  
  ec.setMinClusterSize (100);     
  ec.setMaxClusterSize (25000);   
  ec.setSearchMethod (tree);      
  ec.extract (cluster_indices);   
  

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
     cloud_cluster->points.push_back (cloud->points[*pit]); 
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true; 


    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;


    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    for(size_t i = 0; i <cloud_cluster->points.size();++i)
          centroid.add (pcl::PointXYZ (cloud_cluster->points[i].x, cloud_cluster->points[i].y, cloud_cluster->points[i].z));
    pcl::PointXYZRGB c1;
    centroid.get (c1);
    // c1.r = 0;
    // c1.g = 0;
    // c1.b = 255;
    std::cout <<"Centroid  "<<j<<"   is  <"<<c1.x <<" ,  "<<c1.y<<" , "<<c1.z<<"   >"<<std::endl;
    pcl::visualization::CloudViewer viewer ("Result viewer");
    viewer.showCloud (cloud_cluster);
    while (!viewer.wasStopped ())
    {
    }
    j++;
  }

  return (0);
}