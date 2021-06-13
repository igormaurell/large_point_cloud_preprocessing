#ifndef NORMALIZATION_H
#define NORMALIZATION_H
#define PCL_NO_PRECOMPILE

#include <random>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

#include "point_types/point_types.h"

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
                transformPointCloudWithNormals(*cloud, *cloud, transform);
                auto end = std::chrono::steady_clock::now();
                print_info("The reescale process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else if(reescale_param == 0.f) {
                print_info("\n0 is not a valid parameter for Reescale\n");
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
                transformPointCloudWithNormals(*cloud, *cloud, transform);
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
                transformPointCloudWithNormals(*cloud, *cloud, transform);
                auto end = std::chrono::steady_clock::now();
                print_info("The Align process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
                return true;
            }
            else {
                print_info("\nAlign was not parameterized\n");
                return false;
            }
        }

        bool noiseAdd(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {
            if(noise_add_param != 0.f) {
                auto start = std::chrono::steady_clock::now();
                print_info("\nNoise Add...\n");
                
                //it has to work for organized point cloud too
                std::vector<double> uniform_distribution(cloud->width * cloud->height);
                std::default_random_engine generator{std::random_device{}()};
                std::uniform_real_distribution<double>  distr(-noise_add_param, noise_add_param);
                std::generate(std::begin(uniform_distribution), std::end(uniform_distribution), [&]{ return distr(generator); });

                std::vector<PointT, Eigen::aligned_allocator<PointT> > points = cloud->points;
                for(int i = 0; i < uniform_distribution.size(); i++) {
                    Eigen::Vector3f xyz; xyz << points[i].x, points[i].y, points[i].z;
                    //float* n = points[i].normal;
                    Eigen::Vector3f normal; normal << points[i].normal_x, points[i].normal_y, points[i].normal_z;
                    Eigen::Vector3f noise = normal * uniform_distribution[i];

                    Eigen::Vector3f new_xyz = xyz + noise;

                    points[i].x = new_xyz(0);
                    points[i].y = new_xyz(1);
                    points[i].z = new_xyz(2);
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

        bool cubeReescale(typename pcl::PointCloud<PointT>::Ptr& cloud)
        {
            if(cube_reescale_param) {
                auto start = std::chrono::steady_clock::now();
                print_info("\nCube Reescale...\n");
                Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                addCubeReescaleTransform(cloud, transform);
                transformPointCloudWithNormals(*cloud, *cloud, transform);
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
            std::string methods = "";
            
            if(reescale_param != 1.f && reescale_param != 0.f) methods += "Reescale";
            else print_info("\nReescale was not parameterized\n");

            if(centralize_param) {
                if(methods.size() > 0) methods += "-";
                methods += "Centralize";
            }
            if(align_param) {
                if(methods.size() > 0) methods += "-";
                methods += "Align";
            }
            if(noise_add_param == 0.f && cube_reescale_param != 0.f) {
                if(methods.size() > 0) methods += "-";
                methods += "Cube Reescale";
            }
            
            if(methods.size() > 0) {
                print_info("\n%s...\n", methods.c_str());
            }

            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
           
            if(reescale_param != 1.f && reescale_param != 0.f) addReescaleTransform(transform);
            
            std::string print_after = "";
            if(centralize_param && align_param) {
                addCentralizeAlignTransform(cloud, transform);
            }
            else if(centralize_param) {
                addCentralizeTransform(cloud, transform);
                print_after += "\nAlign was not parameterized\n";
            }
            else if(align_param) {
                addAlignTransform(cloud, transform); 
                print_after += "\nCentralized was not parameterized\n";
            }
            else {
                print_after += "\nCentralized was not parameterized\n";
                print_after += "\nAlign was not parameterized\n";
            }

            bool cube_done = false;
            if(noise_add_param == 0.f && cube_reescale_param != 0.f) {
                addCubeReescaleTransform(cloud, transform);
                cube_done = true;
            }

            transformPointCloudWithNormals(*cloud, *cloud, transform);

            std::cout<<print_after;

            auto end = std::chrono::steady_clock::now();
            if(methods.size() > 0) {
                print_info("The %s process took: ", methods.c_str()); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());
            }
            
            noiseAdd(cloud);

            if(!cube_done) {
                cubeReescale(cloud);
            }

            print_info("Normalized.\n");
        }

        void normalize(pcl::PCLPointCloud2::Ptr& cloud)
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_pc(new pcl::PointCloud<PointT>);
            pcl::fromPCLPointCloud2(*cloud, *cloud_pc);
            normalize(cloud);
            pcl::toPCLPointCloud2(*cloud, *cloud_pc);
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

        void addReescaleTransform(Eigen::Matrix4f& transform) {
            transform(3,3) = reescale_param;
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

        //using just PCA for centroid and rotation (optimize)
        void addCentralizeAlignTransform(typename pcl::PointCloud<PointT>::Ptr& cloud, Eigen::Matrix4f& transform) {
            PCA<PointT> pca;
            pca.setInputCloud(cloud);

            Eigen::Vector4f centroid = pca.getMean();
            transform.block<3,1>(0,3) = -centroid.block<3,1>(0,0);

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
            transform(3,3) = dim[2]*cube_reescale_param;
        }
        
    };
}

#endif //NORMALIZATION_H