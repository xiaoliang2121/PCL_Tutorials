#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());

    pcl::io::loadPCDFile<pcl::PointXYZ>("../data/table_scene_mug_stereo_textured.pcd",*cloud);

    std::cout<<"Cloud Point size: "<<cloud->points.size()<<std::endl;

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (normals);
    // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr
            tree(new pcl::search::KdTree<pcl::PointXYZ>);

    vfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::VFHSignature308>::Ptr
            vfhs(new pcl::PointCloud<pcl::VFHSignature308>());

    // Compute the features
    vfh.compute (*vfhs);

    // fpfhs->points.size () should have the same size as the input cloud->points.size ()*
    std::cout<<"fpfhs Point size: "<<vfhs->points.size()<<std::endl;

    return 0;
}
