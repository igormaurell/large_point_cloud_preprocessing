#ifndef NORMALIZATION_H
#define NORMALIZATION_H
#define PCL_NO_PRECOMPILE

#include <cmath>
#include <ctime>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/random.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

namespace as
{
    template <typename PointT>
    class Normalization{
    public:
        Normalization(double _reescale_param = 1.f, bool _centralize_param = false, bool _align_param = false,
                      double _noise_add_param = 0.f, double _cube_reescale_param = 0.f) : 
                      reescale_param(_reescale_param), noise_add_param(_noise_add_param), centralize_param(_centralize_param),
                      align_param(_align_param), cube_reescale_param(_cube_reescale_param) { }

      
        bool reescale(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {
            if(reescale_param != 1.f && reescale_param != 0.f) {
                auto start = std::chrono::steady_clock::now();
                print_info("\nReescale...\n");
                Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                addReescaleTransform(transform);
                transformPC(cloud, transform);
                auto end = std::chrono::steady_clock::now();
                print_info("The reescale process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else if(reescale_param == 1.f) {
                print_info("\n1 is not a valid parameter for Reescale\n");
                return false;
            }
            else {
                print_info("\nReescale was not parameterized\n");
                return false;
            }
        }

        bool centralize(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {
            if(centralize_param) {
                auto start = std::chrono::steady_clock::now();
                print_info("\nCentralize...\n");
                Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                addCentralizeTransform(cloud, transform);
                transformPC(cloud, transform);
                auto end = std::chrono::steady_clock::now();
                print_info("The Centralize process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else {
                print_info("\nCentralize was not parameterized\n");
                return false;
            }
        }

        bool align(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {
            if(align_param) {
                auto start = std::chrono::steady_clock::now();
                print_info("\nAlign...\n");
                Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                addAlignTransform(cloud, transform);
                transformPC(cloud, transform, true);
                auto end = std::chrono::steady_clock::now();
                print_info("The Align process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else {
                print_info("\nAlign was not parameterized\n");
                return false;
            }
        }
        
        template<
            typename T = PointT,
            pcl::traits::HasNormal<T>* = nullptr
        >
        bool noiseAdd(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {
            if(noise_add_param != 0.f) {
                auto start = std::chrono::steady_clock::now();
                print_info("\nNoise Add...\n");

                unsigned int seed = static_cast<unsigned int>( time(NULL) );
                pcl::common::UniformGenerator<double> rand(-noise_add_param, noise_add_param, seed);

                for(int i = 0; i < cloud->points.size(); i++) {
                    double n = rand.run();
                    Eigen::Vector3f xyz; xyz << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
                    Eigen::Vector3f normal; normal << cloud->points[i].normal_x, cloud->points[i].normal_y, cloud->points[i].normal_z;
                    Eigen::Vector3f noise = normal * n;
                    Eigen::Vector3f new_xyz = xyz + noise;

                    if(!std::isnan(new_xyz(0))) {
                        cloud->points[i].x = new_xyz(0);
                        cloud->points[i].y = new_xyz(1);
                        cloud->points[i].z = new_xyz(2);
                    }
                }

                auto end = std::chrono::steady_clock::now();
                print_info("The Noise Add process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else {
                print_info("\nNoise Add was not parameterized\n");
                return false;
            }
        }

        template<
            typename T = PointT,
            pcl::traits::HasNoNormal<T>* = nullptr
        >
        bool noiseAdd(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {
            print_info("\nNoise Add Error. Point type has no normal.\n");
            return false;
        }
        
        bool cubeReescale(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {
            if(cube_reescale_param > 0) {
                auto start = std::chrono::steady_clock::now();
                print_info("\nCube Reescale...\n");
                Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                addCubeReescaleTransform(cloud, transform);
                transformPC(cloud, transform);
                auto end = std::chrono::steady_clock::now();
                print_info("The Cube Reescale process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else {
                print_info("\nCube Reescale was not parameterized\n");
                return false;
            }
        }

        void normalize(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {      
            print_info("\n\nNormalizing Point Cloud...");
            auto start = std::chrono::steady_clock::now();

            reescale(cloud);
                
            centralize(cloud);

            align(cloud);

            noiseAdd(cloud);

            cubeReescale(cloud);

            print_info("\n");
        }
        
        void setReescaleParam(double _reescale_param) {
            reescale_param = _reescale_param;
        }

        void setNoiseParam(double _noise_add_param) {
            noise_add_param = _noise_add_param;
        }

        void setCentralizeParam(bool _centralize_param) {
            centralize_param = _centralize_param;
        }

        void setAlignParam(bool _align_param) {
            align_param = _align_param;
        }

        void setCubeReescaleParam(double _cube_reescale_param) {
            cube_reescale_param = _cube_reescale_param;
        }

    private:
        double reescale_param;
        bool centralize_param;
        bool align_param;
        double noise_add_param;
        double cube_reescale_param;
        
        template<
            typename T = PointT,
            pcl::traits::HasNormal<T>* = nullptr
        >
        void transformPC(typename pcl::PointCloud<PointT>::Ptr& cloud, Eigen::Matrix4f& transform, bool transform_normals = false) {
            if(transform_normals) transformPointCloudWithNormals(*cloud, *cloud, transform);
            else transformPointCloud(*cloud, *cloud, transform);
        }

        template<
            typename T = PointT,
            pcl::traits::HasNoNormal<T>* = nullptr
        >
        void transformPC(typename pcl::PointCloud<PointT>::Ptr& cloud, Eigen::Matrix4f& transform, bool transform_normals = false) {
            transformPointCloud(*cloud, *cloud, transform);
        }


        void addReescaleTransform(Eigen::Matrix4f& transform) {
            transform(0,0) *= reescale_param;
            transform(1,1) *= reescale_param;
            transform(2,2) *= reescale_param;
        }

        void addCentralizeTransform(typename pcl::PointCloud<PointT>::Ptr& cloud, Eigen::Matrix4f& transform) {
            Eigen::Matrix< float, 4, 1 > centroid;
            compute3DCentroid(*cloud, centroid);
            transform.block<3,1>(0,3) = -centroid.block<3,1>(0,0);
        }

        void addAlignTransform(typename pcl::PointCloud<PointT>::Ptr& cloud, Eigen::Matrix4f& transform) {
            PCA<PointT> pca;
            pca.setInputCloud(cloud);
            Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
            Eigen::Vector3f smallest_ev = eigen_vectors.block<3,1>(0,2);
            Eigen::Vector3f x_axis; x_axis << 1, 0, 0;
            Eigen::Quaternionf q;
            Eigen::Matrix3f R;
            R = q.setFromTwoVectors(smallest_ev, x_axis).toRotationMatrix();
            transform.block<3,3>(0,0) = R;
        }

        void addCubeReescaleTransform(typename pcl::PointCloud<PointT>::Ptr& cloud, Eigen::Matrix4f& transform)
        {
            PointT min, max;
            getMinMax3D(*cloud, min, max);
            std::vector<double> dim = {max.x - min.x, max.y - min.y, max.z - min.z};
            std::sort(dim.begin(), dim.end());
            transform(0,0) *= 1.f/dim[2]*cube_reescale_param;
            transform(1,1) *= 1.f/dim[2]*cube_reescale_param;
            transform(2,2) *= 1.f/dim[2]*cube_reescale_param;
        }
        
    };
}

#endif //NORMALIZATION_H