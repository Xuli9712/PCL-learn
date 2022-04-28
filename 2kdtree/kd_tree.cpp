#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/kdtree/kdtree_flann.h>   //创建kdtree类需要的头文件
#include <pcl/point_cloud.h>           //创建点云类所需要的头文件
#include <pcl/visualization/cloud_viewer.h> //点云可视化头文件
using namespace std;


int main(int argc, char **argv)
{
    //随机初始化种子,使得每次生成随机数不一样
    srand(time(NULL));

    //生成点云数据：1000个无序点云
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //以指针形式生成点云（后面将点云输入kdtree的搜索空间输入的是指针）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    //cloudPtr = cloud.makeShared();
    //cloud = *cloudPtr;


    //点云尺寸
    cloudPtr->width = 1000;
    cloudPtr->height = 1;
    cloudPtr->points.resize(cloudPtr->width*cloudPtr->height);

    //随机填充点云数据0-1023
    //注意for循环条件用;间隔
    for(size_t i =0; i < cloudPtr->width*cloudPtr->height; i++)
    {
        cloudPtr->points[i].x = 1024.0f*rand()/(RAND_MAX + 1.0f);
        cloudPtr->points[i].y = 1024.0f*rand()/(RAND_MAX + 1.0f);
        cloudPtr->points[i].z = 1024.0f*rand()/(RAND_MAX + 1.0f);
    }

    //创建查询点searchpoint
    //注意：只是创建一个点，设置点的数据类型即可，不用创建点云PointCloud类
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f*rand()/(RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f*rand()/(RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f*rand()/(RAND_MAX + 1.0f);
    
    //创建kdtree类的对象， 利用kdtree类的两个方法kdtree.nearestKSearch和kdtree.radiusSearch来寻找“邻居”
    //方法输入的参数是（待查询点， K或Radius， 用来存放邻居点索引的vector, 用来存放邻居点距离平方的vector)
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    //设置搜索空间并输入点云指针
    kdtree.setInputCloud(cloudPtr);
    
    //方法1：kdtree找knn最近邻10个点
    //创建kdtree的实现类kdtreeFLANN(Fast Approximate Nearest Neighbor) 
    //设置k近邻搜索,搜索10个临近点
    int K = 10;
    //列表存放1.搜索到的前k=10个最近点索引 2.对应临近点和查找点距离的平方
    vector<int> pointKNNIdx(K);
    vector<float> pointKNNSquaredDist(K);

    //输出查询点情况
    cout << "KNN search at" << " " << searchPoint.x << " " << searchPoint.y << " " 
         << searchPoint.z << " " << "with K = " << K << endl; 


    if (kdtree.nearestKSearch(searchPoint, K, pointKNNIdx, pointKNNSquaredDist) > 0)
    {
        for(size_t i=0; i < pointKNNIdx.size(); i++)
        {
            cout << " " << cloudPtr->points[pointKNNIdx[i]].x
                 << " " << cloudPtr->points[pointKNNIdx[i]].y
                 << " " << cloudPtr->points[pointKNNIdx[i]].z
                 << " " << "SquaredDistance:" << pointKNNSquaredDist[i] << endl;
        }
    }

    //方法2：kdtree按radius半径范围搜索
    //设置范围在)[0，,256）的随机半径
    float Radius = 256.0f * rand()/(RAND_MAX + 1.0f);
    //存放查询结果（邻居点索引以及距离）的vector
    vector<int> pointRadiusIdx;
    vector<float> pointRadiusSquaredDist;
    //输出查询点情况
    cout << "Radius = " << Radius << "search at "
         << " " << searchPoint.x
         << " " << searchPoint.y
         << " " << searchPoint.z << endl;

    if (kdtree.radiusSearch(searchPoint, Radius, pointRadiusIdx, pointRadiusSquaredDist) > 0)
    {
        for(size_t i = 0; i < pointRadiusIdx.size(); i++)
        {
            cout << cloudPtr->points[pointRadiusIdx[i]].x
                 << cloudPtr->points[pointRadiusIdx[i]].y
                 << cloudPtr->points[pointRadiusIdx[i]].z
                 << " " << "Squared Distance:"
                 << pointRadiusSquaredDist[i] << endl;
        }
    }
    //可视化
    //设置原点
    pcl::PointXYZ oPoint(0.0, 0.0, 0.0);
    //创建viewer对象
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    //设置viewer
    viewer.setBackgroundColor(0.0, 0.0, 0.3);
    viewer.addCoordinateSystem(200);
    viewer.addLine(oPoint, searchPoint);
    viewer.addPointCloud(cloudPtr, "cloud");
    viewer.addSphere(searchPoint, Radius, "sphere", 0);
    
    while(!viewer.wasStopped())
    {
        //每次循环调用重绘函数
        viewer.spinOnce();
    }


return 0;

}

