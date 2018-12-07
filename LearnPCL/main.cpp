#include <iostream>
#include <pcl/point_types.h>
//#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr
            cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("../data/table_scene_mug_stereo_textured.pcd",*cloud);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);

    pcl::PointCloud<pcl::Normal>::Ptr
            cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0,0.0,0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,cloud_normals);

    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}
