#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h> // 滤波文件头
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h> //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>  //模型定义头文件
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char **argv)
{
    // Create the normal estimation class, and pass the input dataset to it
    // 定义输入和输出点云
    if (argc != 2)
    {
        printf("\nicp  example \n--------------------------\n");
        printf("/home/agv/1/build/bin/demo pcd1 \n");
        printf("/home/agv/1/build/bin/demo r2000_1.pcd\n");
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *target_cloud) == -1) //*打开点云文件
        printf("load failed\n");
    cerr << "pointcloud representing :" << target_cloud->points.size() << "data points" << std::endl;

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(target_cloud));
    std::vector<int> inliers;
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(0.001);
    ransac.computeModel();
    ransac.getInliers(inliers);
    
    pcl::copyPointCloud<pcl::PointXYZ>(*target_cloud,inliers,*final);

   

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(target_cloud);
    sor.setMeanK(5);
    sor.setStddevMulThresh(0.1);
    sor.filter(*target_cloud);
        pcl::io::savePCDFileASCII("/home/agv/1/build/bin/filter.pcd", *target_cloud);
        
        std::cerr << "Cloud after filtering: " << std::endl;
        std::cerr << *target_cloud << std::endl;

        pcl::ModelCoefficients::Ptr coefficients_line(new pcl::ModelCoefficients);

        pcl::PointIndices::Ptr inliers_line(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZ> seg; //ransac进行直线识别

        seg.setOptimizeCoefficients(true);

        seg.setModelType(pcl::SACMODEL_LINE);

        seg.setMethodType(pcl::SAC_RANSAC);

        seg.setMaxIterations(100);

        seg.setDistanceThreshold(1);

        seg.setInputCloud(target_cloud);

        seg.segment(*inliers_line, *coefficients_line);
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        extract.setInputCloud(target_cloud);
        extract.setIndices(inliers_line);
        extract.setNegative(false);
        extract.filter(*out_cloud);
    std:cerr<<"pointcloud representing :"<<out_cloud->points.size()<<"data points"<<std::endl;
        pcl::io::savePCDFileASCII("/home/agv/1/build/bin/line.pcd", *out_cloud);
        cout<<"pcl_viewer  -fc 255,255,255 r2000_1.pcd -fc 255,0,0 filter.pcd "<<endl;

        return 0;
}
/*
//3）计算边界

    //要先计算分割出平面的法向量，然后提取轮廓边界。

    //calculate boundary;

    pcl::PointCloud<pcl::Boundary>
        boundary;

pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;

est.setInputCloud(cloud_plane);

est.setInputNormals(normals_plane);

est.setSearchMethod(tree_plane);

est.setKSearch(50); //一般这里的数值越高，最终边界识别的精度越好

pcl::search::KdTree<pcl::PointXYZ>;

est.compute(boundary); */