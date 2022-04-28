#include <iostream>
#include <ctime>
#include <vector>

#include <pcl/point_cloud.h> //引入创建点云类对象所需头文件
#include <pcl/octree/octree_search.h>
#include <pcl/visualization/pcl_visualizer.h> //引入创建可视化类对象头文件

using namespace std;

int main(int argc, char **argv)
{
    //创建随机数种子
    srand(time(NULL));

    //创建点云指针对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    //Generate PointCloud data
    cloudPtr->width = 1000;
    cloudPtr->height = 1; //无序点云
    cloudPtr->resize(cloudPtr->width * cloudPtr->height);
    //点云坐标范围[0,1024),随机分布在一个立方体的范围内
    for (size_t i = 0; i < cloudPtr->points.size(); i++)
    {
        cloudPtr->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloudPtr->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloudPtr->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }
    //设置查找点
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    //设置分辨率为128,分辨率描述了leaf节点的最小尺寸
    float resolution = 128.0f;
    //创建octree
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    //输入点云
    octree.setInputCloud(cloudPtr);
    //通过输入的点云来构建octree
    octree.addPointsFromInputCloud();

    //调用octree的方法进行查找
    //voxel近邻搜索，返回查找点所在的体素voxel的其他点的索引
    //创建保存结果索引的向量
    vector<int> pointIdx;

    if (octree.voxelSearch(searchPoint, pointIdx) > 0)
    {
        //输出查找点
        cout << "Neighbor Search with Voxel at: " 
             << " " << searchPoint.x
             << " " << searchPoint.y
             << " " << searchPoint.z << endl;
        for (size_t i = 0; i < pointIdx.size(); i++)
        {
            cout << " " << cloudPtr->points[pointIdx[i]].x
                 << " " << cloudPtr->points[pointIdx[i]].y
                 << " " << cloudPtr->points[pointIdx[i]].z << endl;
        }
    }
    //KNN查找点
    //设置Ｋ，保存结果索引和距离的向量,保存索引的为int向量，保存距离的为float向量
    int K = 10;
    vector<int> pointKNNIdx(10);
    vector<float> pointKNNSquaredDist(10);

    //输出query point信息
    cout << "KNN search at " 
         << " " << searchPoint.x
         << " " << searchPoint.y
         << " " << searchPoint.z << " " << "With K =" << " " << K << endl;

    if (octree.nearestKSearch(searchPoint, K, pointKNNIdx, pointKNNSquaredDist)>0)
    {
        for(size_t i =0; i < pointKNNIdx.size(); i++)
        {
            cout << " " << cloudPtr->points[pointKNNIdx[i]].x
                 << " " << cloudPtr->points[pointKNNIdx[i]].y
                 << " " << cloudPtr->points[pointKNNIdx[i]].z 
                 << " " << "distance: " << pointKNNSquaredDist[i] << endl;
        }
    }
    //半径范围搜索
    //设置半径,范围在[0, 256)
    float Radius = 256.0f * rand()/(RAND_MAX + 1.0f);
    //结果索引和距离保存的vector
    vector<int> pointRadiusIdx;
    vector<float> pointRadiusSquaredDist;
    //输出query point
    cout << "Neighbors within radius search at" 
         << " " << searchPoint.x
         << " " << searchPoint.y
         << " " << searchPoint.z 
         << " " << "With radius =" << " " << Radius << endl;
    //调用半径查找方法
    if (octree.radiusSearch(searchPoint, Radius, pointRadiusIdx, pointRadiusSquaredDist))
    {
        for(size_t i = 0; i < pointRadiusIdx.size(); i++)
        {
            cout << " " << cloudPtr->points[pointRadiusIdx[i]].x
                 << " " << cloudPtr->points[pointRadiusIdx[i]].y
                 << " " << cloudPtr->points[pointRadiusIdx[i]].z
                 << " " << "square distance:" << " " << pointRadiusSquaredDist[i] << endl;
        }
    }
//可视化
//创建Viewer对象
pcl::visualization::PCLVisualizer viewer("PCL Viewr");
//设置原点
pcl::PointXYZ oPoint(0.0, 0.0, 0.0);
viewer.setBackgroundColor(0.0, 0.0, 0.4);
viewer.addPointCloud(cloudPtr, "cloud");
viewer.addLine(oPoint, searchPoint);
viewer.addSphere(searchPoint, Radius, "sphere", 0);
viewer.addCoordinateSystem(200);

while ((!viewer.wasStopped()))
{
    viewer.spinOnce();
}

return 0;
}