#ifndef FILTERS_H
#define FILTERS_H
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

namespace as
{
    template <typename PointT>
    class Filters{
    public:
        Filters();

        Filters(std::vector<double> _cb_min, std::vector<double> _cb_max, std::vector<double> _vg_params,
                std::vector<double> _sor_params) : cb_min(_cb_min), cb_max(_cb_max),
                                                vg_params(_vg_params), sor_params(_sor_params) {}

        bool cropBoxFilter(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {
            if(cb_min.size() == 3 || cb_max.size() == 3) {
                std::vector<double> lcb_min, lcb_max;
                if(cb_min.size() != 3) lcb_min = std::vector<double>(3, -std::numeric_limits<double>::infinity());
                else lcb_min = cb_min;

                if(cb_max.size() != 3) lcb_max = std::vector<double>(3, std::numeric_limits<double>::infinity());
                else lcb_max = cb_max;

                auto start = std::chrono::steady_clock::now();
                print_info("\nCrop Box Filtering...\n");
                Eigen::Vector4f min_p, max_p;
                min_p << lcb_min[0], lcb_min[1], lcb_min[2], 1;
                max_p << lcb_max[0], lcb_max[1], lcb_max[2], 1;
                typename pcl::CropBox<PointT> cp;
                cp.setInputCloud(cloud);
                cp.setMin(min_p);
                cp.setMax(max_p);
                cp.filter(*cloud);
                print_info("PointCloud after Crop Box filtering: "); print_value("%d data points\n", cloud->width * cloud->height);
                auto end = std::chrono::steady_clock::now();
                print_info("The Crop Box filtering process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else {
                print_info("\nCrop Box was not parameterized\n");
                return false;
            }  
        }

        bool voxelGridFilter(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {   
            if(vg_params.size() == 3) {
                auto start = std::chrono::steady_clock::now();
                print_info("\nVoxel Grid Filtering...\n");
                typename pcl::VoxelGrid<PointT> vg;
                vg.setInputCloud (cloud);
                vg.setLeafSize (vg_params[0], vg_params[1], vg_params[2]);
                vg.filter (*cloud);
                typename pcl::PointCloud<PointT>::Ptr aux(nullptr);
                print_info("PointCloud after voxel grid filtering: "); print_value("%d data points\n", cloud->width * cloud->height);
                auto end = std::chrono::steady_clock::now();
                print_info("The voxel grid filtering process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else {
                print_info("\nVoxel Grid was not parameterized\n");
                return false;
            }
            
        }

        bool statisticalOutlierRemovalFilter(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {
            if(sor_params.size() == 2) {
                auto start = std::chrono::steady_clock::now();
                print_info("\nStatistical Outlier Removal Filtering...\n");
                typename pcl::StatisticalOutlierRemoval<PointT> sor;
                sor.setInputCloud (cloud);
                sor.setMeanK (sor_params[0]);
                sor.setStddevMulThresh (sor_params[1]);
                sor.filter (*cloud);
                print_info("PointCloud after statistical filtering: "); print_value("%d data points\n", cloud->width * cloud->height);
                auto end = std::chrono::steady_clock::now();
                print_info("The statistical outlier removal process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else {
                print_info("\nStatistical Outlier Removal was not parameterized\n");
                return false;
            }

        }

        void filter(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {   
            print_info("\n\nFiltering Point Cloud...");
            cropBoxFilter(cloud);

            voxelGridFilter(cloud);

            statisticalOutlierRemovalFilter(cloud);

            print_info("Filtered.\n");
        }
        
        void setCutOffParams(std::vector<double>& _cb_min, std::vector<double>& _cb_max){
            cb_min = std::vector<double>(_cb_min);
            cb_max = std::vector<double>(_cb_max);
        }
        void setVoxelGridParams(std::vector<double>& _vg_params){
            vg_params = std::vector<double>(_vg_params);
        }

        void setStatisticalOutlierRemovalParams(std::vector<double>& _sor_params){
            sor_params = std::vector<double>(_sor_params);
        }

    private:

        std::vector<double> cb_min;
        std::vector<double> cb_max;

        std::vector<double> vg_params;

        std::vector<double> sor_params;
    };

    template <>
    class Filters<pcl::PCLPointCloud2>{
    public:
        Filters();

        Filters(std::vector<double> _cb_min, std::vector<double> _cb_max, std::vector<double> _vg_params,
                std::vector<double> _sor_params) : cb_min(_cb_min), cb_max(_cb_max),
                                                vg_params(_vg_params), sor_params(_sor_params) {}

        bool cropBoxFilter(pcl::PCLPointCloud2::Ptr& cloud)
        {
            if(cb_min.size() == 3 || cb_max.size() == 3) {
                std::vector<double> lcb_min, lcb_max;
                if(cb_min.size() != 3) lcb_min = std::vector<double>(3, -std::numeric_limits<double>::infinity());
                else lcb_min = cb_min;

                if(cb_max.size() != 3) lcb_max = std::vector<double>(3, std::numeric_limits<double>::infinity());
                else lcb_max = cb_max;

                auto start = std::chrono::steady_clock::now();
                print_info("\nCrop Box Filtering...\n");
                Eigen::Vector4f min_p, max_p;
                min_p << lcb_min[0], lcb_min[1], lcb_min[2], 1;
                std::cout<<min_p(0)<<" "<<min_p(1)<<" "<<min_p(2)<<" "<<min_p(3)<<std::endl;
                max_p << lcb_max[0], lcb_max[1], lcb_max[2], 1;
                std::cout<<max_p(0)<<" "<<max_p(1)<<" "<<max_p(2)<<" "<<max_p(3)<<std::endl;
                pcl::CropBox<pcl::PCLPointCloud2> cp;
                cp.setInputCloud(cloud);
                cp.setMin(min_p);
                cp.setMax(max_p);
                cp.filter(*cloud);
                print_info("PointCloud after Crop Box filtering: "); print_value("%d data points\n", cloud->width * cloud->height);
                auto end = std::chrono::steady_clock::now();
                print_info("The Crop Box filtering process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else {
                print_info("\nCrop Box was not parameterized\n");
                return false;
            }  
        }

        bool voxelGridFilter(pcl::PCLPointCloud2::Ptr& cloud)
        {   
            if(vg_params.size() == 3) {
                auto start = std::chrono::steady_clock::now();
                print_info("\nVoxel Grid Filtering...\n");
                pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
                vg.setInputCloud (cloud);
                vg.setLeafSize (vg_params[0], vg_params[1], vg_params[2]);
                vg.filter (*cloud);
                pcl::PCLPointCloud2::Ptr aux(nullptr);
                print_info("PointCloud after voxel grid filtering: "); print_value("%d data points\n", cloud->width * cloud->height);
                auto end = std::chrono::steady_clock::now();
                print_info("The voxel grid filtering process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else {
                print_info("\nVoxel Grid was not parameterized\n");
                return false;
            }
            
        }

        bool statisticalOutlierRemovalFilter(pcl::PCLPointCloud2::Ptr& cloud)
        {
            if(sor_params.size() == 2) {
                auto start = std::chrono::steady_clock::now();
                print_info("\nStatistical Outlier Removal Filtering...\n");
                pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
                sor.setInputCloud (cloud);
                sor.setMeanK (sor_params[0]);
                sor.setStddevMulThresh (sor_params[1]);
                sor.filter (*cloud);
                print_info("PointCloud after statistical filtering: "); print_value("%d data points\n", cloud->width * cloud->height);
                auto end = std::chrono::steady_clock::now();
                print_info("The statistical outlier removal process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else {
                print_info("\nStatistical Outlier Removal was not parameterized\n");
                return false;
            }

        }

        void filter(pcl::PCLPointCloud2::Ptr& cloud)
        {   
            print_info("\n\nFiltering Point Cloud...");
            cropBoxFilter(cloud);

            voxelGridFilter(cloud);

            statisticalOutlierRemovalFilter(cloud);

            print_info("\n");
        }
        
        void setCutOffParams(std::vector<double>& _cb_min, std::vector<double>& _cb_max){
            cb_min = std::vector<double>(_cb_min);
            cb_max = std::vector<double>(_cb_max);
        }

        void setVoxelGridParams(std::vector<double>& _vg_params){
            vg_params = std::vector<double>(_vg_params);
        }

        void setStatisticalOutlierRemovalParams(std::vector<double>& _sor_params){
            sor_params = std::vector<double>(_sor_params);
        }

    private:

        std::vector<double> cb_min;
        std::vector<double> cb_max;

        std::vector<double> vg_params;

        std::vector<double> sor_params;
    };
}

#endif //FILTERS_H