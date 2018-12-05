#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/point_types.h>
#include <pcl/octree/octree.h>

int main (int argc, char** argv)
{
    srand((unsigned int)time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr
            cloudA(new pcl::PointCloud<pcl::PointXYZ>),
            cloudB(new pcl::PointCloud<pcl::PointXYZ>);

    // 创建点云数据
    cloudA->width =128;
    cloudA->height =1;
    cloudA->points.resize (cloudA->width * cloudA->height);
    for (size_t i=0; i< cloudA->points.size (); ++i)
    {
        cloudA->points[i].x =64.0f* rand () / (RAND_MAX +1.0f);
        cloudA->points[i].y =64.0f* rand () / (RAND_MAX +1.0f);
        cloudA->points[i].z =64.0f* rand () / (RAND_MAX +1.0f);
    }

    //为cloudB创建点云
    cloudB->width =128;
    cloudB->height =1;
    cloudB->points.resize (cloudB->width * cloudB->height);
    for (size_t i=0; i< cloudB->points.size (); ++i)
    {
        cloudB->points[i].x =64.0f* rand () / (RAND_MAX +1.0f);
        cloudB->points[i].y =64.0f* rand () / (RAND_MAX +1.0f);
        cloudB->points[i].z =64.0f* rand () / (RAND_MAX +1.0f);
    }

    // 八叉树分辨率 即体素的大小
    float resolution = 32.0f;
    // 初始化空间变化检测对象
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);
    //添加点云到八叉树，建立八叉树
    octree.setInputCloud(cloudA);
    octree.addPointsFromInputCloud();
    // 交换八叉树缓存，但是cloudA对应的八叉树仍在内存中
    octree.switchBuffers();

    //添加 cloudB到八叉树
    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();

    //获取前一cloudA对应的八叉树在cloudB对应八叉树中没有的体素
    std::vector<int> newPointIdxVec;
    octree.getPointIndicesFromNewVoxels(newPointIdxVec);

    //打印输出点
    std::cout<<"Output from getPointIndicesFromNewVoxels:"<<std::endl;
    for (size_t i=0; i<newPointIdxVec.size(); ++i)
        std::cout<<i<<"# Index:"<<newPointIdxVec[i]
                   <<"  Point:"<<cloudB->points[newPointIdxVec[i]].x <<" "
                  <<cloudB->points[newPointIdxVec[i]].y <<" "
                 <<cloudB->points[newPointIdxVec[i]].z
                <<std::endl;

    return 0;
}
