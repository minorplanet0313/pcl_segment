#include <ros/ros.h>
#include <sstream>
#include <string.h>
// PCL specific includes
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

//点云分割
#include <pcl/ModelCoefficients.h>   
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h> //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h> //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件

//欧式聚类提取
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

// 法向量
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

//自定义消息
#include <pcl_segment/Object.h>
#include <pcl_segment/Objects.h>

ros::Publisher pass_pub;
ros::Publisher voxel_pub;
ros::Publisher statistical_pub;
// ros::Publisher radius_pub;
ros::Publisher planar_pub;
ros::Publisher objects_pub;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  //直通滤波
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered0(new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>); 

  pcl::PassThrough<pcl::PointXYZRGB> pass; //设置滤波器对象 
  pass.setInputCloud(cloud);            //设置输入点云

  //x方向
  pass.setFilterFieldName("x");         //设置过滤时所需要点云类型的z字段
  pass.setFilterLimits(-0.25, 0.25);       //设置在过滤字段上的范围
  pass.filter(*cloud_filtered0); //执行滤波，滤波结果保存在cloud_filtered
  //y方向
  pass.setFilterFieldName("y");         //设置过滤时所需要点云类型的z字段
  pass.setFilterLimits(-0.55, 0.2);       //设置在过滤字段上的范围
  pass.filter(*cloud_filtered1); //执行滤波，滤波结果保存在cloud_filtered
  //z方向
  pass.setFilterFieldName("z");         //设置过滤时所需要点云类型的z字段
  pass.setFilterLimits(0.0, 1.0);       //设置在过滤字段上的范围
  //pass.setFilterLimitsNegative (true);//设置保留范围内的还是过滤范围内的
  pass.filter(*cloud_filtered2); //执行滤波，滤波结果保存在cloud_filtered

  std::cout << "直通滤波成功，剩余点数： " << cloud_filtered2->width * cloud_filtered2->height << std::endl;
  return cloud_filtered2;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_grid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  //体素滤波
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); 

  pcl::VoxelGrid<pcl::PointXYZRGB> sor;//创建滤波器对象 
  sor.setInputCloud(cloud);//设置待滤波的点云 
  sor.setLeafSize(0.003, 0.003, 0.003);
  sor.filter(*cloud_filtered);

  std::cout << "体素滤波成功，剩余点数： " << cloud_filtered->width * cloud_filtered->height << std::endl;
  return cloud_filtered;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr statistical_removal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  //统计学移除外点
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); 
  
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;//创建滤波器对象
  sor.setInputCloud(cloud);//设置待滤波的对象
  sor.setMeanK(20);//设置在进行统计时考虑查询点邻近点数
  sor.setStddevMulThresh(0.5);//设置判断是否为离群点的阈值，标准差倍数，如果一个点的距离超出平均距离一个标准差以上，该点被标记为离群点
  sor.filter(*cloud_filtered);//滤波并保存

  std::cout << "StatisticalOutlierRemoval滤波成功,剩余点数： " << cloud_filtered->width * cloud_filtered->height << std::endl;
  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr radius_removal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  //使用RadiusOutlierRemoval移除离群点
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); 

  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;//创建滤波器
  outrem.setInputCloud(cloud);//设置输入点云
  outrem.setRadiusSearch(0.008);//设置在0.8半径的范围内找邻近点
  outrem.setMinNeighborsInRadius(5);//设置邻近点集数小于5的删除
  outrem.filter(*cloud_filtered);//执行条件滤波,存储结果到cloud_filtered

  std::cout << "RadiusOutlierRemoval滤波成功,剩余点数： " << cloud_filtered->width * cloud_filtered->height << std::endl;
  return cloud_filtered;
}

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr planar_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
// {
//   //平面点云分割
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); 
//   *cloud_filtered = *cloud;

//   pcl::SACSegmentation<pcl::PointXYZRGB> seg; //点云对像
//   pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //点云索引对象
//   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //模型系数
  
//   seg.setOptimizeCoefficients(true);  //设置优化
//   seg.setModelType(pcl::SACMODEL_PLANE); //设置分割模型为平面
//   seg.setMethodType(pcl::SAC_RANSAC); //设置分割方法
//   seg.setMaxIterations(100); //最大迭代次数
//   seg.setDistanceThreshold(0.01); //设置距离阈值

//   int i = 0, nr_points = (int)cloud_filtered->points.size(); //滤波后点的个数
//   while (cloud_filtered->points.size() > 0.7 * nr_points)
//   {
//     // 从点云中分割最大的平面
//     seg.setInputCloud(cloud_filtered);
//     seg.segment(*inliers, *coefficients); //分割，保存内点和模型系数
//     if (inliers->indices.size() == 0)
//     {
//       std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//       break;
//     }

//     // 提取剩余点
//     pcl::ExtractIndices<pcl::PointXYZRGB> extract; //创建提取对象
//     extract.setInputCloud(cloud_filtered); //输入待提取的点云
//     extract.setIndices(inliers);  //输入平面模型内点
//     extract.setNegative(true); //提取剩余的点
//     extract.filter(*cloud_filtered);
//   }

//   std::cout << "分割平面成功，剩余点数： " << cloud_filtered->width * cloud_filtered->height << std::endl;
//   return cloud_filtered;
// }

void euclidean_cluster_extraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  //欧式聚类提取
  // 为提取点云时使用的搜索对象利用输入点云cloud_filtered创建kd树对象
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  pcl::PCDWriter writer;//创建写对象

  std::vector<pcl::PointIndices> cluster_indices;    //创建点云索引向量，用于存储实际的点云信息，每个检测到的点云聚类被保存在这里
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec; //
  ec.setClusterTolerance(0.02);                      // 设置近邻搜索的搜索半径为2cm
  ec.setMinClusterSize(200);                         //设置一个聚类搜索需要的最少点数目为100
  ec.setMaxClusterSize(25000);                       //设置一个聚类需要的最大点数目为25000
  ec.setSearchMethod(tree);                          //设置点云的搜索机制为kdtree
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices); //从点云中提取聚类,并将点云索引保存在cluster_indices

  //迭代访问点云索引cluster_indices,直到分割出所有聚类
  int j = 0;
  pcl_segment::Object object;
  pcl_segment::Objects objects;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    object.num = j;
    //创建新的点云数据集cloud_cluster,将所有当前聚类写入到点云数据集中
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
      cloud_cluster->points.push_back(cloud->points[*pit]); 

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "正在提取第 "<< j << " 个聚类， 共 "<< cloud_cluster->points.size() << " 个点" << std::endl;

    // 计算聚类质心
    Eigen::Vector4f centroids;
    pcl::compute3DCentroid(*cloud_cluster, centroids);              //进行质心计算
    std::cout<<"第 "<< j <<" 个聚类质心为：("<<centroids[0]<<","<<centroids[1]<<","<<centroids[2]<<")."<<std::endl;
    
    object.x=centroids[0];
    object.y=centroids[1];
    object.z=centroids[2];

    // 计算距离质心最近的点
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;//FLANN 快速近似近邻算法库实现的KdTree
    kdtree.setInputCloud (cloud_cluster);
    pcl::PointXYZRGB searchPoint;//设置查询对象点

    searchPoint.x = centroids[0];
    searchPoint.y = centroids[1];
    searchPoint.z = centroids[2];

    int K = 1; // K邻近搜索个数
    std::vector<int> pointIdxNKNSearch(K);//保存邻近点索引
    std::vector<float> pointNKNSquaredDistance(K);//保存对象点与邻近点的距离平方值

    std::cout << "K nearest neighbor search at (" << searchPoint.x 
              << " " << searchPoint.y 
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;
  
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )//最邻近查找，返回邻近点数
    {
      for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        std::cout << "    "  <<   (*cloud_cluster)[ pointIdxNKNSearch[i] ].x 
                  << " " << (*cloud_cluster)[ pointIdxNKNSearch[i] ].y 
                  << " " << (*cloud_cluster)[ pointIdxNKNSearch[i] ].z 
                  << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

     // 计算聚类法向量
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne; //估计法线
    ne.setInputCloud (cloud_cluster);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());//创建一个空的kdtree对象,并把它传递给法线估计对象
    ne.setSearchMethod (tree);//基于给出的输入数据集,kdtree将被建立
    
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);//输出数据集
    ne.setRadiusSearch (0.03);//使用半径在查询点周围3厘米范围内的所有邻元素
    ne.compute (*cloud_normals);//计算特征值

    std::cout<<"["<<cloud_normals->points[pointIdxNKNSearch[0]].normal_x<<" "
                                <<cloud_normals->points[pointIdxNKNSearch[0]].normal_y<<" "
                                <<cloud_normals->points[pointIdxNKNSearch[0]].normal_z<<" "
                                <<cloud_normals->points[pointIdxNKNSearch[0]].curvature<<"]"<< std::endl;
    object.theta_x=cloud_normals->points[pointIdxNKNSearch[0]].normal_x;
    object.theta_y=cloud_normals->points[pointIdxNKNSearch[0]].normal_y;
    object.theta_z=cloud_normals->points[pointIdxNKNSearch[0]].normal_z;

    // 计算聚类最值
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    int M = cloud_cluster->points.size();
    for (int i = 0;i <M;i++)
    {
            pcl::PointXYZ p;
            p.x = cloud_cluster->points[i].x;
            p.y = cloud_cluster->points[i].y;
            p.z = cloud_cluster->points[i].z; 
            cloud_xyz->points.push_back(p);
    }
    cloud_xyz->width = M;
    cloud_xyz->height = 1;
 
    pcl::PointXYZ min;//用于存放三个轴的最小值
    pcl::PointXYZ max;//用于存放三个轴的最大值
    pcl::getMinMax3D(*cloud_xyz,min,max);
 
    std::cout<<"min.z = "<<min.z<<std::endl;
    std::cout<<"max.z = "<<max.z<<"\n"<<std::endl;

    object.mode = 1;

    objects.object.push_back(object);

    // 保存聚类
    // std::stringstream ss;
    // ss << "/home/wt/图片/pcl/cloud_cluster_" << j << ".pcd";
    // writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_cluster, false); 
    j++;
  }

  objects_pub.publish(objects);
  std::cout << "欧式聚类提取成功，提取物体个数： " << j << std::endl;
}

void save_to_pcd(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string name)
{
  //保存成pcd文件
  std::string path;
  std::stringstream ss;
  std::string s1;

  path = "/home/wt/图片/pcl/";
  ss << ros::Time::now();
  s1 = ss.str();
  std::string file_name = path + "cloud_" + name + "_" + s1 + ".pcd";

  pcl::io::savePCDFileASCII(file_name, *cloud);
  std::cout << "保存到文件： " << file_name << std::endl;

}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

  // 格式转换：sensor_msgs::PointCloud2转换为pcl::PointCloud<pcl::PointXYZRGB>
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // // 坐标变换 
  // float theta = M_PI / 4; // 弧度角
  // Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
  // // 在 X 轴上定义一个 1.5 米的平移.
  // transform_1.translation() << 1.5, 0.0, 0.0;
  // // 和前面一样的旋转; Z 轴上旋转 theta 弧度
  // transform_1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  // // 打印变换矩阵
  // std::cout << "Method #2: using a Affine3f: " << transform_1.matrix() << std::endl;
  // // 执行变换，并将结果保存在新创建的transformed_cloud 中
  // pcl::transformPointCloud(*cloud, *cloud, transform_1);

  //直通滤波
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_pass(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_filtered_pass = passthrough(cloud);
   // 发布点云
  sensor_msgs::PointCloud2 pass_output;
  pcl::toROSMsg(*cloud_filtered_pass, pass_output); //点云格式转换：pcl::PointCloud转换为sensor_msgs::PointCloud2
  pass_pub.publish(pass_output);

  // 体素滤波
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_filtered_voxel = voxel_grid(cloud_filtered_pass);
  // 发布点云
  sensor_msgs::PointCloud2 voxel_output;
  pcl::toROSMsg(*cloud_filtered_voxel, voxel_output); //点云格式转换：pcl::PointCloud转换为sensor_msgs::PointCloud2
  voxel_pub.publish(voxel_output);

  //统计学移除外点滤波
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_statistical(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_filtered_statistical = statistical_removal(cloud_filtered_voxel);
  // 发布点云
  sensor_msgs::PointCloud2 statistical_output;
  pcl::toROSMsg(*cloud_filtered_statistical, statistical_output); //点云格式转换：pcl::PointCloud转换为sensor_msgs::PointCloud2
  statistical_pub.publish(statistical_output);

  // //使用RadiusOutlierRemoval移除离群点
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_radius(new pcl::PointCloud<pcl::PointXYZRGB>);
  // cloud_filtered_radius = radius_removal(cloud_filtered_statistical);
  //  // 发布点云
  // sensor_msgs::PointCloud2 radius_output;
  // pcl::toROSMsg(*cloud_filtered_radius, radius_output); //点云格式转换：pcl::PointCloud转换为sensor_msgs::PointCloud2
  // radius_pub.publish(radius_output);

  // //平面点云分割
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_seg_planar(new pcl::PointCloud<pcl::PointXYZRGB>);
  // cloud_seg_planar = planar_segmentation(cloud_filtered_statistical);\
  // // 发布点云
  // sensor_msgs::PointCloud2 planar_output;
  // pcl::toROSMsg(*cloud_seg_planar, planar_output); //点云格式转换：pcl::PointCloud转换为sensor_msgs::PointCloud2
  // planar_pub.publish(planar_output);

  //欧式聚类提取
  euclidean_cluster_extraction(cloud_filtered_statistical);

  

  //保存点云
  //save_to_pcd(cloud_seg_planar, "cloud_seg_planar");
  
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "my_pcl_tutorial"); //初始化ROS节点
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>( "/camera/depth/color/points", 1, cloud_cb); //创建订阅者

  pass_pub = nh.advertise<sensor_msgs::PointCloud2>("pass", 1); //创建发布者
  voxel_pub = nh.advertise<sensor_msgs::PointCloud2>("voxel", 1); //创建发布者
  statistical_pub = nh.advertise<sensor_msgs::PointCloud2>("statistical", 1); //创建发布者
  // radius_pub = nh.advertise<sensor_msgs::PointCloud2>("radius", 1); //创建发布者
  // planar_pub = nh.advertise<sensor_msgs::PointCloud2>("planar", 1); //创建发布者

  objects_pub = nh.advertise<pcl_segment::Objects>("objects", 1); 
  
  ros::spin(); //循环等待回调函数
}
