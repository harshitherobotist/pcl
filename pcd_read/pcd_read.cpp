#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("table_scene_lms400_objects.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00;
  float minRange = 0.0f;
  int borderSize = 1;
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  std::cout << rangeImage << "\n";
  
  pcl::visualization::RangeImageVisualizer range_image_widget ("RangeImage");
  range_image_widget.showRangeImage (rangeImage);

  std::cout << "Loaded "
            << cloud->width * cloud->height
            // << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for(size_t i = 0; i <cloud->points.size();++i)
          centroid.add (pcl::PointXYZ (cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    pcl::PointXYZ c1;
    centroid.get (c1);
    std::cout <<"Centroid is  <"<<c1.x <<" ,  "<<c1.y<<" , "<<c1.z<<"   >"<<std::endl;
    pcl::visualization::CloudViewer viewer ("Result viewer");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ())
    {
    }
  return (0);
}
