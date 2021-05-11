#ifndef NORMAL_ESTIMATION_H
#define NORMAL_ESTIMATION_H
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include "point_types/point_types.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

namespace as
{
    template <typename PointT>
    class NormalEstimation{
    public:
        NormalEstimation() : cloud(new pcl::PointCloud<PointT>), search(new pcl::PointCloud<PointT>), ne_param(0.f) {}

        NormalEstimation(double _ne_param) : cloud(new pcl::PointCloud<PointT>), ne_param(_ne_param) {}

        void setInputCloud(pcl::PCLPointCloud2::Ptr& input)
        {
            pcl::fromPCLPointCloud2(*input, *cloud);      
        }

        void setInputCloud(typename pcl::PointCloud<PointT>::Ptr& input)
        {
            cloud = input;
        }

        void setSearchSurface(typename pcl::PointCloud<PointT>::Ptr& _search)
        {
            search = _search;
        }


        void normalEstimationOMP(typename pcl::PointCloud<PointT>::Ptr& input, pcl::PointCloud<Normal>::Ptr& output)
        {   
            auto start = std::chrono::steady_clock::now();
            print_info("\nNormal Estimation...\n");
            typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
            typename pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
            ne.setInputCloud(input);
            ne.setSearchSurface(search);
            ne.setSearchMethod(tree);
            ne.setViewPoint(0, 0, 0);
            ne.setRadiusSearch (ne_param);
            ne.compute (*output);
            // pcl::concatenateFields(*input, *normals, *output);
            auto end = std::chrono::steady_clock::now();
            print_info("The normal estimation process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
        }

        bool compute(typename pcl::PointCloud<Normal>::Ptr& output)
        {
            pcl::PointCloud<Normal>::Ptr out(new pcl::PointCloud<Normal>);
            if(ne_param != 0) {
                normalEstimationOMP(cloud, out);
                output = out;
                return true;
            }
            return false;
        }

        bool compute(pcl::PCLPointCloud2::Ptr& output)
        {
            pcl::PointCloud<Normal>::Ptr out(new pcl::PointCloud<Normal>);
            if (compute(out)) {
                pcl::toPCLPointCloud2(*out, *output);
                return true;
            }
            return false;
        }

    private:
        typename pcl::PointCloud<PointT>::Ptr cloud;
        typename pcl::PointCloud<PointT>::Ptr search;

        double ne_param;
    };
}

#endif //NormalEstimation_H