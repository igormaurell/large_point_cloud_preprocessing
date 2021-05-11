#define PCL_NO_PRECOMPILE
#include <iostream>
#include <string>
#include <chrono>
#include <limits>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include "point_types/point_types.h"
#include "filters.h"
#include "normal_estimation.h"

//filters
std::vector<double> co_min;
std::vector<double> co_max;

std::vector<double> vg_params;

std::vector<double> sor_params;

//normal estimation params
double ne_param;
void normalEstimation(pcl::PCLPointCloud2::Ptr& input, pcl::PCLPointCloud2::Ptr& output, pcl::PCLPointCloud2::Ptr& search);

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void readParameters(int argc, char** argv);
void loadPCD(std::string filename, pcl::PCLPointCloud2& cloud);
void savePCD(std::string filename, pcl::PCLPointCloud2& cloud);
void printHelp (int, char **argv);

int
main (int argc, char** argv)
{
  print_info ("Pre-processing for large point clouds. For more information, use: %s -h\n",
              argv[0]);

  if(argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }
  auto start = std::chrono::steady_clock::now();
   
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2), out_filtered(new pcl::PCLPointCloud2), normals(new pcl::PCLPointCloud2),
                           out_normals(new pcl::PCLPointCloud2);
  std::string filename = argv[1];
  loadPCD(filename, *cloud);

  std::string s_fields(pcl::getFieldsList(*cloud)), w;
  std::stringstream ss(s_fields);
  std::vector<std::string> fields;
  while (ss >> w) {
    fields.push_back(w);
  }

  readParameters(argc, argv);

  PointCloud<PointXYZRGBNormalFace>::Ptr before_vg;
  as::Filters<PointXYZRGBNormalFace> filters(co_min, co_max, vg_params, sor_params);
  filters.setInputCloud(cloud);
  filters.filter(out_filtered, before_vg);
  as::NormalEstimation<PointXYZRGBNormalFace> ne(ne_param);
  ne.setInputCloud(out_filtered);
  ne.setSearchSurface(before_vg);
  bool save_normal = false;
  if(ne.compute(normals)) {
    pcl::concatenateFields(*out_filtered, *normals, *out_normals);
    save_normal = true;
  }
  else 
    out_normals = out_filtered;

  for(int i = 0; i < out_normals->fields.size(); ) {
    if(std::find(fields.begin(), fields.end(), out_normals->fields[i].name) != fields.end() || (save_normal &&
      (out_normals->fields[i].name == "normal_x" || out_normals->fields[i].name == "normal_y" || out_normals->fields[i].name == "normal_z" || out_normals->fields[i].name == "curvature"))) {
      
      i++;
    }
    else {
      out_normals->fields.erase(out_normals->fields.begin() + i);
    }
  }

  std::string out_filename = argv[2];
  savePCD(out_filename, *out_normals); 

  auto end = std::chrono::steady_clock::now();
  print_info("\nThe overall process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());

  return 0;
}

void
readParameters(int argc, char** argv)
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

void 
loadPCD(std::string filename, pcl::PCLPointCloud2& cloud)
{
  auto start_local = std::chrono::steady_clock::now();
  print_info ("Reading Point Cloud...\n");
  loadPCDFile(filename, cloud);
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList(cloud).c_str ());
  print_info("PointCloud before filtering: "); print_value("%d data points\n", cloud.width * cloud.height);
  auto end_local = std::chrono::steady_clock::now();
  print_info("The reading process took: "); print_value("%f sec\n", static_cast<std::chrono::duration<double>>(end_local - start_local).count());
}

void
savePCD(std::string filename, pcl::PCLPointCloud2& cloud) {
  auto start_local = std::chrono::steady_clock::now();
  print_info("\nWriting Point Cloud...\n");
  savePCDFile(filename, cloud);
  auto end_local = std::chrono::steady_clock::now();
  print_info("The writing process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end_local - start_local).count());
}

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