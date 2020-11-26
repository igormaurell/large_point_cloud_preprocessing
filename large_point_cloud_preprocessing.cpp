#include <iostream>
#include <string>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_co (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_vg (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_sor (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);

  //params
  bool is_co = false;
  std::vector<float> min(3, 0.0);
  std::vector<float> max(3, 0.0);

  bool is_vg = false;
  std::vector<float> vg_params(3, 0.0);

  bool is_sor = false;
  std::vector<float> sor_params(2, 0.0);

  bool is_normal = false;
  float normal_param;
  for(int i = 0; i < argc; i++) {
    int j;
    if (strcmp(argv[i], "-c") == 0){
      if(argc <= (i+3)) {
        std::cerr << "Missing cut-off parameters, it will not be executed." << std::endl;
        continue;
      }
      std::cerr << "Cut-off Parameters:" << std::endl;
      std::string delimiter = ","; 
      for(j = 0; j < 3; j++){
        std::string lim = argv[i + j + 1];
        std::size_t found = lim.find(delimiter);
        if (found==std::string::npos){
          std::cerr << "Cut-off parameters in wrong format, it will not be executed. Right format is: -c min_x,max_x min_y,max_y min_z,max_z" \
          << std::endl;
          break;
        }
        std::string mins = lim.substr(0, found);
        std::string maxs = lim.substr(found + 1, lim.size());
        min[j] = std::stof(mins);
        max[j] = std::stof(maxs);
        std::cerr << min[j] << " " << max[j] << std::endl;
      }
      if(j == 3) is_co = true;
    }
    else if (strcmp(argv[i], "-v") == 0){
      if(argc <= (i+3)) {
        std::cerr << "Missing voxel grid parameters, it will not be executed." << std::endl;
        continue;
      }
      std::cerr << "Voxel grid Parameters:" << std::endl;
      for(j = 0; j < 3; j++){
        std::string param = argv[i + j + 1];
        vg_params[j] = std::stof(param);
        std::cerr << vg_params[j] << std::endl;
      }
      is_vg = true;
    }
    else if (strcmp(argv[i], "-s") == 0){
      if(argc <= (i+2)) {
        std::cerr << "Missing statistical outlier removal parameters, it will not be executed." << std::endl;
        continue;
      }
      std::cerr << "Statistical Outlier Removal Parameters:" << std::endl;
      for(j = 0; j < 2; j++){
        std::string param = argv[i + j + 1];
        sor_params[j] = std::stof(param);
        std::cerr << sor_params[j] << std::endl;
      }
      is_sor = true;
    }
    else if(strcmp(argv[i], "-n") == 0){
      if(argc <= (i+1)) {
        std::cerr << "Missing normal estimation parameters, it will not be executed." << std::endl;
        continue;
      }
      std::cerr << "Normal Estimation Parameters:" << std::endl;
      std::string param = argv[i + 1];
      normal_param = std::stof(param);
      std::cerr << normal_param << std::endl;
      is_normal = true;
    }
  }

  std::cerr<<std::endl;

  auto start = std::chrono::steady_clock::now();

  auto start_local = std::chrono::steady_clock::now();
  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  std::string filename = argv[1];
  std::cerr << "Reading Point Cloud..." << std::endl;
  reader.read<pcl::PointXYZ> (filename.c_str(), *cloud);
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;
  auto end_local = std::chrono::steady_clock::now();
  std::cerr << "The reading process took: " 
  << std::chrono::duration_cast<std::chrono::seconds>(end_local - start_local).count() 
  << " sec"<< std::endl << std::endl;


  if(is_co) {
    start_local = std::chrono::steady_clock::now();
    std::cerr << "Cut-off Filtering..." << std::endl;
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
      pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, min[0])));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, max[0])));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, min[1])));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, max[1])));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, min[2])));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, max[2])));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
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
  

  if(is_vg) {
    start_local = std::chrono::steady_clock::now();
    std::cerr << "Voxel Grid Filtering..." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
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


  // Create the filtering object
  if(is_sor) {
    start_local = std::chrono::steady_clock::now();
    std::cerr << "Statistical Outlier Removal Filtering..." << std::endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
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

  if(is_normal) {
    start_local = std::chrono::steady_clock::now();
    std::cerr << "Normal Estimation..." << std::endl;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_filtered_sor);
    ne.setSearchSurface(cloud_filtered_co);
    ne.setSearchMethod(tree);
    ne.setViewPoint(0, 0, 0);
    ne.setRadiusSearch (normal_param);
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
  writer.write<pcl::PointXYZ> (out_filename.c_str(), *cloud_filtered_sor, false);
  if(is_normal) {
    out_filename = out_filename.substr(0, out_filename.size() - 4) + "-normal.pcd";
    writer.write<pcl::PointNormal> (out_filename.c_str(), *cloud_normals, false);
  }
  end_local = std::chrono::steady_clock::now();
  std::cerr << "The writing process took: " 
  << std::chrono::duration_cast<std::chrono::seconds>(end_local - start_local).count() 
  << " sec"<< std::endl << std::endl;

  auto end = std::chrono::steady_clock::now();

  std::cerr << "The overall process took: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() 
  << " sec"<< std::endl;

  return (0);
}