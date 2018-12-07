#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr
            cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("../data/table_scene_lms400.pcd",*cloud);

    pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr
            tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr
            cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_normals);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0,0.0,0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,cloud_normals);

    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}
