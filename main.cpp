#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h> // 滤波文件头
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
void mavlink_dcm_to_euler(const double dcm[3][3], float *roll, float *pitch, float *yaw)
{
   float phi, theta, psi;
   theta = asin(-dcm[2][0]);

   if (fabsf(theta - (float)M_PI_2) < 1.0e-3f)
   {
      phi = 0.0f;
      psi = (atan2f(dcm[1][2] - dcm[0][1],
                    dcm[0][2] + dcm[1][1]) +
             phi);
   }
   else if (fabsf(theta + (float)M_PI_2) < 1.0e-3f)
   {
      phi = 0.0f;
      psi = atan2f(dcm[1][2] - dcm[0][1],
                   dcm[0][2] + dcm[1][1] - phi);
   }
   else
   {
      phi = atan2f(dcm[2][1], dcm[2][2]);
      psi = atan2f(dcm[1][0], dcm[0][0]);
   }

   *roll = phi;
   *pitch = theta;
   *yaw = psi;
}

double MatchCloud(int time,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget,
                  pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &matrix)
{
   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

   icp.setMaximumIterations(time);
   // Setting maximum step size for More-Thuente line search.

   icp.setInputSource(cloudSource);
   icp.setInputTarget(cloudTarget);
   pcl::PointCloud<pcl::PointXYZ> Final;
   icp.align(Final);
   matrix = icp.getFinalTransformation();
   std::cout << "time=" << time << endl;
   std::cout << matrix << endl;

   double dcm[3][3] = {0};
   float roll, pitch, yaw = 0.0;
   for (int i = 0; i < 3; i++)
   {
      dcm[i][0] = matrix(i, 0);
      dcm[i][1] = matrix(i, 1);
      dcm[i][2] = matrix(i, 2);
   }

   mavlink_dcm_to_euler(dcm, &roll, &pitch, &yaw);
   printf("rotatex=%lf\n", roll);
   printf("rotatey=%lf\n", pitch);
   printf("rotatez=%lf\n", yaw);
   double score =  icp.getFitnessScore();

   return score;
}

int main(int argc, char **argv)
{
   if (argc != 3)
   {
      printf("\nicp  example \n--------------------------\n");
      printf("./main pcd1 pcd2\n");
      printf("./main r2000_1.pcd r2000_2.pcd\n");
      return -1;
   }

   // 定义输入和输出点云
   pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

   if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *target_cloud) == -1) //*打开点云文件
      printf("load failed\n");
   if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *input_cloud) == -1) //*打开点云文件
      printf("load failed\n");



   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *target_cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (target_cloud);
  sor.setMeanK (20);
  sor.setStddevMulThresh (0.01);
  sor.setNegative (true);
  sor.filter (*cloud_filtered);

  std::cerr << "target_cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

   pcl::io::savePCDFileASCII("cloud_filtered.pcd", *cloud_filtered);


   pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
  sor.setInputCloud (input_cloud);
  sor.setMeanK (20);
  sor.setStddevMulThresh (0.01);
  sor.setNegative (true);
  sor.filter (*cloud_filtered1);

  std::cerr << "input_cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered1 << std::endl;

   pcl::io::savePCDFileASCII("cloud_filtered1.pcd", *cloud_filtered1);

   // 创建IterativeClosestPoint的实例
   // setInputSource将target_cloud作为输入点云
   // setInputTarget将平移后的input_cloud作为目标点云
   pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 matrix1;
   
   // cout<<MatchCloud(5, input_cloud, target_cloud,matrix1)<<endl;
   // cout << MatchCloud(20, input_cloud, target_cloud, matrix1) << endl;
   // cout << MatchCloud(50, input_cloud, target_cloud, matrix1) << endl;
   // cout << MatchCloud(100, input_cloud, target_cloud, matrix1) << endl;
   
   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

   icp.setMaximumIterations(50);
   // Setting maximum step size for More-Thuente line search.

   icp.setInputSource(cloud_filtered);
   icp.setInputTarget(cloud_filtered1);

   // 创建一个 pcl::PointCloud<pcl::PointXYZ>实例 Final 对象,存储配准变换后的源点云,
   // 应用 ICP 算法后, IterativeClosestPoint 能够保存结果点云集,如果这两个点云匹配正确的话
   // （即仅对其中一个应用某种刚体变换，就可以得到两个在同一坐标系下相同的点云）,那么 icp. hasConverged()= 1 (true),
   // 然后会输出最终变换矩阵的匹配分数和变换矩阵等信息。
   pcl::PointCloud<pcl::PointXYZ>
       Final;
   icp.align(Final); // double dcm[3][3] = {0};
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 matrix;
   matrix = icp.getFinalTransformation();
   std::cout << matrix << std::endl;

   

   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
   // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud1 (new pcl::PointCloud<pcl::PointXYZ> ());

   pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, matrix);
   // pcl::transformPointCloud (*input_cloud,*transformed_cloud1,matrix.inverse());
   pcl::io::savePCDFileASCII("transformed_cloud.pcd", *transformed_cloud);

   double dcm[3][3] = {0};
   float roll, pitch, yaw = 0.0;
   for (int i = 0; i < 3; i++)
   {
      dcm[i][0] = matrix(i, 0);
      dcm[i][1] = matrix(i, 1);
      dcm[i][2] = matrix(i, 2);
   }

   mavlink_dcm_to_euler(dcm, &roll, &pitch, &yaw);
   printf("rotatex=%lf\n", roll);
   printf("rotatey=%lf\n", pitch);
   printf("rotatez=%lf\n", yaw);
 
   pcl::visualization::PCLVisualizer::Ptr
       viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
   viewer_final->setBackgroundColor(0, 0, 0);

   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
       target_color(cloud_filtered, 255, 0, 0);
   viewer_final->addPointCloud<pcl::PointXYZ>(cloud_filtered, target_color, "target cloud");
   viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

   // Coloring and visualizing transformed input cloud (green).
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
       input_color(cloud_filtered1, 0, 255, 200);
   viewer_final->addPointCloud<pcl::PointXYZ>(cloud_filtered1, input_color, "input cloud");
   viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "input cloud");

   // Coloring and visualizing transformed input cloud (blue).
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
       transformed_color(transformed_cloud, 0, 0, 255);
   viewer_final->addPointCloud<pcl::PointXYZ>(transformed_cloud, transformed_color, "transformed_cloud cloud");
   viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "transformed_cloud cloud");
   
   // Starting visualizer
   viewer_final->addCoordinateSystem(1.0, "global");
   viewer_final->initCameraParameters();

   // Wait until visualizer window is closed.
   while (!viewer_final->wasStopped())
   {
      viewer_final->spinOnce(100);
   }

   return (0);
}
