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
        NormalEstimation() : neomp_param(0.f) {}

        NormalEstimation(double _neomp_param) : neomp_param(_neomp_param) { }

        bool normalEstimationOMP(typename pcl::PointCloud<PointT>::Ptr& input, pcl::PointCloud<Normal>::Ptr& output,
                                 typename pcl::PointCloud<PointT>::Ptr& search)
        {   
            if(neomp_param != 0) {
                auto start = std::chrono::steady_clock::now();
                print_info("\nNormal Estimation...\n");
                typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
                typename pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
                ne.setInputCloud(input);
                ne.setSearchSurface(search);
                ne.setSearchMethod(tree);
                ne.setViewPoint(0, 0, 0);
                ne.setRadiusSearch (neomp_param);
                ne.compute (*output);
                auto end = std::chrono::steady_clock::now();
                print_info("The normal estimation process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else {
                print_info("\nNormal Estimation was not parameterized\n");
                return false;
            }
        }

        bool compute(typename pcl::PointCloud<PointT>::Ptr& input, pcl::PointCloud<Normal>::Ptr& output,
                     typename pcl::PointCloud<PointT>::Ptr& search)
        {
            print_info("\n\nEstimating Normal of Point Cloud...");
            bool result = normalEstimationOMP(input, output, search);
            print_info("Normal Estimated...\n");
            return result;
        }

        bool compute(pcl::PCLPointCloud2::Ptr& input, pcl::PCLPointCloud2::Ptr& output,
                     typename pcl::PointCloud<PointT>::Ptr& search)
        {
            typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            pcl::PointCloud<Normal>::Ptr out(new pcl::PointCloud<Normal>);
            pcl::fromPCLPointCloud2(*input, *cloud);      
            if (compute(cloud, out, search)) {
                pcl::toPCLPointCloud2(*out, *output);
                return true;
            }
            return false;
        }

        void setNormalEstimationOMPParam(double _neomp_param) {
            neomp_param = _neomp_param;
        }

    private:
        double neomp_param;
    };
}

#endif //NormalEstimation_H