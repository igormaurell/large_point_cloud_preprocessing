#define PCL_NO_PRECOMPILE
#include <iostream>
#include <string>
#include <chrono>
#include <limits>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include "point_types/point_types.h"

//filtering params {co = cut_off, vg = voxel grid, sor = statistical outlier removal}
std::vector<double> co_min;
std::vector<double> co_max;

std::vector<double> vg_params;

std::vector<double> sor_params;

//normal estimation params
double ne_param;

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     --co_min x, y, z      = minimum x, y and z values to use\n");
  print_info ("                     --co_max x, y, z      = maximum x, y and z values to use\n");
  print_info ("                     --vg x, y, z | x      = leaf size for voxel grid for x, y and z cordinates, if just x is passed, it is used for all cordinates\n");
  print_info ("                     --sor mean, std       = mean and std for a statistical outlier removal filter\n");
  print_info ("                     --ne radius       = radius of the sphere used to estimate the nomal of each point\n");
}

void readParameters(int argc, char** argv)
{
  parse_x_arguments (argc, argv, "--co_min", co_min);
  if(co_min.size() != 3)
  {
    if(co_min.size() != 0) print_error ("Cut off minimum must be specified with 3 numbers (%lu given).\n", co_min.size());
  }

  parse_x_arguments (argc, argv, "--co_max", co_max);
  if(co_max.size() != 3)
  {
    if(co_max.size() != 0) print_error ("Cut off maximum must be specified with 3 numbers (%lu given).\n", co_max.size());
  }
  
  parse_x_arguments (argc, argv, "--vg", vg_params);
  if(vg_params.size() == 1)
  {
    vg_params = std::vector<double>(3, vg_params[0]);
  }
  else if(vg_params.size() != 3)
  {
    if(vg_params.size() != 0) print_error ("Voxel Grid leaf size must be specified with either 1 or 3 numbers (%lu given).\n", vg_params.size());
  }

  parse_x_arguments (argc, argv, "--sor", sor_params);
  if(sor_params.size() != 2)
  {
    if(sor_params.size() != 0) print_error ("Statistical outlier removal maximum must be specified with 2 numbers (%lu given).\n", sor_params.size());
  }
  
  parse_argument (argc, argv, "--ne", ne_param);   
}

int
main (int argc, char** argv)
{
  print_info ("Pre-processing for large point clouds. For more information, use: %s -h\n",
              argv[0]);

  pcl::PointCloud<PointNormalFace>::Ptr cloud (new pcl::PointCloud<PointNormalFace>);
  pcl::PointCloud<PointNormalFace>::Ptr cloud_filtered_co (new pcl::PointCloud<PointNormalFace>);
  pcl::PointCloud<PointNormalFace>::Ptr cloud_filtered_vg (new pcl::PointCloud<PointNormalFace>);
  pcl::PointCloud<PointNormalFace>::Ptr cloud_filtered_sor (new pcl::PointCloud<PointNormalFace>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointNormalFace>::Ptr cloud_normals (new pcl::PointCloud<PointNormalFace>);

  if(argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  readParameters(argc, argv);

  auto start = std::chrono::steady_clock::now();

  auto start_local = std::chrono::steady_clock::now();
  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  std::string filename = argv[1];
  std::cerr << "Reading Point Cloud..." << std::endl;
  reader.read<PointNormalFace> (filename.c_str(), *cloud);
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;
  auto end_local = std::chrono::steady_clock::now();
  std::cerr << "The reading process took: " 
  << std::chrono::duration_cast<std::chrono::seconds>(end_local - start_local).count() 
  << " sec"<< std::endl << std::endl;


  if(co_min.size() == 3 || co_max.size() == 3) {
    if(co_min.size() != 3) co_min = std::vector<double>(3, -std::numeric_limits<double>::infinity());
    if(co_max.size() != 3) co_max = std::vector<double>(3, std::numeric_limits<double>::infinity());
    start_local = std::chrono::steady_clock::now();
    std::cerr << "Cut-off Filtering..." << std::endl;
    pcl::ConditionAnd<PointNormalFace>::Ptr range_cond (new
      pcl::ConditionAnd<PointNormalFace> ());
    range_cond->addComparison (pcl::FieldComparison<PointNormalFace>::ConstPtr (new
      pcl::FieldComparison<PointNormalFace> ("x", pcl::ComparisonOps::GT, co_min[0])));
    range_cond->addComparison (pcl::FieldComparison<PointNormalFace>::ConstPtr (new
      pcl::FieldComparison<PointNormalFace> ("x", pcl::ComparisonOps::LT, co_max[0])));
    range_cond->addComparison (pcl::FieldComparison<PointNormalFace>::ConstPtr (new
      pcl::FieldComparison<PointNormalFace> ("y", pcl::ComparisonOps::GT, co_min[1])));
    range_cond->addComparison (pcl::FieldComparison<PointNormalFace>::ConstPtr (new
      pcl::FieldComparison<PointNormalFace> ("y", pcl::ComparisonOps::LT, co_max[1])));
    range_cond->addComparison (pcl::FieldComparison<PointNormalFace>::ConstPtr (new
      pcl::FieldComparison<PointNormalFace> ("z", pcl::ComparisonOps::GT, co_min[2])));
    range_cond->addComparison (pcl::FieldComparison<PointNormalFace>::ConstPtr (new
      pcl::FieldComparison<PointNormalFace> ("z", pcl::ComparisonOps::LT, co_max[2])));
    // build the filter
    pcl::ConditionalRemoval<PointNormalFace> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    //condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (*cloud_filtered_co);
    std::cerr << "PointCloud after cut-off filtering: " << cloud_filtered_co->width * cloud_filtered_co->height << \
    " data points." << std::endl;
    end_local = std::chrono::steady_clock::now();
    std::cerr << "The cut-off filtering process took: " 
    << std::chrono::duration_cast<std::chrono::seconds>(end_local - start_local).count() 
    << " sec" << std::endl << std::endl;
  }
  else cloud_filtered_co = cloud;
  

  if(vg_params.size() == 3) {
    start_local = std::chrono::steady_clock::now();
    std::cerr << "Voxel Grid Filtering..." << std::endl;
    pcl::VoxelGrid<PointNormalFace> vg;
    vg.setInputCloud (cloud_filtered_co);
    vg.setLeafSize (vg_params[0], vg_params[1], vg_params[2]);
    vg.filter (*cloud_filtered_vg);
    std::cerr << "PointCloud after voxel grid filtering: " << cloud_filtered_vg->width * cloud_filtered_vg->height << \
    " data points." << std::endl;
    end_local = std::chrono::steady_clock::now();
    std::cerr << "The voxel grid filtering process took: " 
    << std::chrono::duration_cast<std::chrono::seconds>(end_local - start_local).count() 
    << " sec"<< std::endl << std::endl;  
  }
  else cloud_filtered_vg = cloud_filtered_co;


  // // Create the filtering object
  if(sor_params.size() == 2) {
    start_local = std::chrono::steady_clock::now();
    std::cerr << "Statistical Outlier Removal Filtering..." << std::endl;
    pcl::StatisticalOutlierRemoval<PointNormalFace> sor;
    sor.setInputCloud (cloud_filtered_vg);
    sor.setMeanK (sor_params[0]);
    sor.setStddevMulThresh (sor_params[1]);
    sor.filter (*cloud_filtered_sor);
    std::cerr << "PointCloud after statistical filtering: " << cloud_filtered_sor->width * cloud_filtered_sor->height << \
    " data points." << std::endl;
    end_local = std::chrono::steady_clock::now();
    std::cerr << "The statistical outlier removal process took: " 
    << std::chrono::duration_cast<std::chrono::seconds>(end_local - start_local).count() 
    << " sec"<< std::endl << std::endl;
  }
  else cloud_filtered_sor = cloud_filtered_vg;

  if(ne_param != 0) {
    start_local = std::chrono::steady_clock::now();
    std::cerr << "Normal Estimation..." << std::endl;
    pcl::search::KdTree<PointNormalFace>::Ptr tree (new pcl::search::KdTree<PointNormalFace> ());
    pcl::NormalEstimationOMP<PointNormalFace, pcl::Normal> ne;
    ne.setInputCloud(cloud_filtered_sor);
    ne.setSearchSurface(cloud_filtered_co);
    ne.setSearchMethod(tree);
    ne.setViewPoint(0, 0, 0);
    ne.setRadiusSearch (ne_param);
    ne.compute (*normals);
    pcl::concatenateFields(*cloud_filtered_sor, *normals, *cloud_normals);
    end_local = std::chrono::steady_clock::now();
    std::cerr << "The normal estimation process took: " 
    << std::chrono::duration_cast<std::chrono::seconds>(end_local - start_local).count() 
    << " sec"<< std::endl << std::endl;
  }


  start_local = std::chrono::steady_clock::now();
  std::cerr << "Writing Point Cloud..." << std::endl;
  std::string out_filename = argv[2];
  pcl::PCDWriter writer;
  writer.write<PointNormalFace> (out_filename.c_str(), *cloud_filtered_sor, false);
  // if(ne_param != 0) {
  //   out_filename = out_filename.substr(0, out_filename.size() - 4) + "-normal.pcd";
  //   writer.write<pcl::PointNormal> (out_filename.c_str(), *cloud_normals, false);
  // }
  end_local = std::chrono::steady_clock::now();
  std::cerr << "The writing process took: " 
  << std::chrono::duration_cast<std::chrono::seconds>(end_local - start_local).count() 
  << " sec"<< std::endl << std::endl;

  auto end = std::chrono::steady_clock::now();

  std::cerr << "The overall process took: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() 
  << " sec"<< std::endl;

  return (0);
}