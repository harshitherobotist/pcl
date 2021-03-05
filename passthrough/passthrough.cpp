#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  pcl::PCDReader reader;

  reader.read<pcl::PointXYZ> ("table_scene_lms400_transformed.pcd", *cloud);

  // for (auto& point: *cloud)
  // {
  //   point.x = 1024 * rand () / (RAND_MAX + 1.0f);
  //   point.y = 1024 * rand () / (RAND_MAX + 1.0f);
  //   point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  // }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 0.5);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-1.5, 1.5);
  //pass.setFilterLimitsNegative (true);
  //pass.setFilterLimitsNegative (true);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.069, 0.074);
  pass.filter (*cloud_filtered);

  std::cerr << "Cloud after passthrough filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_passthrough.pcd", *cloud_filtered, false);


  std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& point: *cloud_filtered)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  return (0);
}
