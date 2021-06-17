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
#include "preprocessing/filters.h"
#include "preprocessing/normal_estimation.h"
#include "preprocessing/normalization.h"

bool use_search_surface_param;

//filters params (co = cut-off, vg = voxel-grid, sor = statistical outlier removal)
std::vector<double> cb_min;
std::vector<double> cb_max;

std::vector<double> vg_params;

std::vector<double> sor_params;

//normal estimation params
double neomp_param;

//normalization
double reescale_param;
bool centralize_param;
bool align_param;
double noise_add_param;
double cube_reescale_param;

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void readParameters(int argc, char** argv);
void loadPCD(std::string filename, pcl::PCLPointCloud2& cloud);
void savePCD(const std::string &filename, const pcl::PCLPointCloud2 &output);
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
   
  pcl::PCLPointCloud2::Ptr cloud_pc2(new pcl::PCLPointCloud2), aux_pc2(new pcl::PCLPointCloud2);
  PointCloud<pcl::PointNormal>::Ptr cloud_normal(new PointCloud<pcl::PointNormal>);
  PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new PointCloud<pcl::PointXYZ>), search_xyz(new PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  std::string filename = argv[1];
  loadPCD(filename, *cloud_pc2);

  readParameters(argc, argv);

  if(use_search_surface_param) pcl::fromPCLPointCloud2(*cloud_pc2, *search_xyz);

  as::Filters<pcl::PCLPointCloud2> filters(cb_min, cb_max, vg_params, sor_params);
  filters.filter(cloud_pc2);

  bool has_normal = false;
  for(auto it = cloud_pc2->fields.begin(); it != cloud_pc2->fields.end(); it++) {
    if(it->name == "normal_x") has_normal = true;
  }

  pcl::fromPCLPointCloud2(*cloud_pc2, *cloud_xyz);
  if(!use_search_surface_param) search_xyz = cloud_xyz;
  as::NormalEstimation<pcl::PointXYZ> ne(neomp_param);

  if(ne.compute(cloud_xyz, search_xyz, normals)) {
    if(use_search_surface_param) search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::concatenateFields(*cloud_xyz, *normals, *cloud_normal);
    normals.reset(new pcl::PointCloud<pcl::Normal>);

    has_normal = true;
  }
  else if(has_normal) {
    if(use_search_surface_param) search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_pc2, *cloud_normal);
  }
  else {
    if(use_search_surface_param) search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_pc2, *cloud_xyz);
  }
  
  if(has_normal) {
    as::Normalization<pcl::PointNormal> normalization(reescale_param, centralize_param, align_param, noise_add_param, cube_reescale_param);
    normalization.normalize(cloud_normal);
    pcl::toPCLPointCloud2(*cloud_normal, *aux_pc2);
    cloud_normal.reset(new pcl::PointCloud<pcl::PointNormal>);
  }
  else {
    as::Normalization<pcl::PointXYZ> normalization(reescale_param, centralize_param, align_param, noise_add_param, cube_reescale_param);
    normalization.normalize(cloud_xyz);
    pcl::toPCLPointCloud2(*cloud_xyz, *aux_pc2);
    cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
  }
  
  pcl::PCLPointCloud2::Ptr out_pc2(new pcl::PCLPointCloud2);

  concatenateFields(*cloud_pc2, *aux_pc2, *out_pc2);
  cloud_pc2.reset(new pcl::PCLPointCloud2);
  aux_pc2.reset(new pcl::PCLPointCloud2);

  std::string out_filename = argv[2];
  savePCD(out_filename, *out_pc2); 

  auto end = std::chrono::steady_clock::now();
  print_info("\n\nThe overall process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());

  return 0;
}

void
readParameters(int argc, char** argv)
{
  use_search_surface_param = !find_switch (argc, argv, "--no_use_search_surface");

  parse_x_arguments (argc, argv, "--cb_min", cb_min);
  if(cb_min.size() != 3)
  {
    if(cb_min.size() != 0) print_error ("Cut off minimum must be specified with 3 numbers (%lu given).\n", cb_min.size());
  }

  parse_x_arguments (argc, argv, "--cb_max", cb_max);
  if(cb_max.size() != 3)
  {
    if(cb_max.size() != 0) print_error ("Cut off maximum must be specified with 3 numbers (%lu given).\n", cb_max.size());
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

  parse_argument (argc, argv, "--ne", neomp_param);

  parse_argument (argc, argv, "--reescale_factor", reescale_param);
  
  centralize_param = find_switch (argc, argv, "--centralize");

  align_param = find_switch (argc, argv, "--align");

  parse_argument (argc, argv, "--noise_limit", noise_add_param);

  parse_argument (argc, argv, "--cube_reescale_factor", cube_reescale_param);
}

void 
loadPCD(std::string filename, pcl::PCLPointCloud2& cloud)
{
  auto start_local = std::chrono::steady_clock::now();
  print_info ("\nReading Point Cloud...\n");
  loadPCDFile(filename, cloud);
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList(cloud).c_str ());
  print_info("PointCloud before filtering: "); print_value("%d data points\n", cloud.width * cloud.height);
  auto end_local = std::chrono::steady_clock::now();
  print_info("The reading process took: "); print_value("%f sec\n", static_cast<std::chrono::duration<double>>(end_local - start_local).count());
}

void
savePCD(const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  auto start_local = std::chrono::steady_clock::now();
  print_info("\n\nWriting Point Cloud...\n");
  PCDWriter w;
  w.writeASCII (filename, output);
  auto end_local = std::chrono::steady_clock::now();
  print_info("The writing process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end_local - start_local).count());
}

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                 ----no_use_search_surface       = to not use the original cloud as search surface for normal estimation\n");
  print_info ("                 --cb_min x, y, z                = minimum x, y and z values to use\n");
  print_info ("                 --cb_max x, y, z                = maximum x, y and z values to use\n");
  print_info ("                 --vg x, y, z | x                = leaf size for voxel grid for x, y and z cordinates, if just x is passed, it is used for all cordinates\n");
  print_info ("                 --sor mean, std                 = mean and std for a statistical outlier removal filter\n");
  print_info ("                 --ne radius                     = radius of the sphere used to estimate the nomal of each point\n");
  print_info ("                 --reescale_factor factor        = factor that will reescale all the points (change measurament unity)\n");
  print_info ("                 --centralize                    = use it to put the origin of the pointcloud at the geometric center of the points\n");
  print_info ("                 --align                         = use it to aling the x axis of the coordinate system with the axis of minor variation on point cloud\n");
  print_info ("                 --noise_limit limit             = limit of a random uniform noise applied at the normal direction for each point\n");
  print_info ("                 --cube_reescale_factor factor   = make all the point cloud lies in a cube of edge size equal factor\n");
}