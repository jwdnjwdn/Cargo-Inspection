#pragma once
#include <fstream>
#include <iostream>
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <boost/thread/thread.hpp>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <time.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <vtkAutoInit.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/surface/concave_hull.h>
#include <vector>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>
class Compute_point {
	public:
		//the rotate formla
		Eigen::Matrix4f rotate_matrix(Eigen::Vector3f angle_before, Eigen::Vector3f angle_after) {

			angle_before.normalize();
			angle_after.normalize();
			float angle = acos(angle_before.dot(angle_after));//点积，得到两向量的夹角
			Eigen::Vector3f p_rotate = angle_before.cross(angle_after);//叉积，得到的还是向量
			p_rotate.normalize();//该向量的单位向量，即旋转轴的单位向量
			Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
			rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
			rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle));//这里跟公式比多了一个括号，但是看实验结果它是对的。
			rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));

			rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
			rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
			rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));

			rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
			rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
			rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));
			return rotationMatrix;
		}

		//read point cloud
		void readPLYFileBinary(std::string& in_file_path, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_input) {

			//读取文件
			ifstream in;
			in.open(in_file_path);
			if (!in) {
				cout << "can't open the ply file" << endl;
			}
			std::string line;
			//读取头部关键字 判断类型
			std::string point_type_bin = "binary";
			std::string point_type_asc = "ascii";

			while (!in.eof()) {
				getline(in, line);
				if (line.find(point_type_bin) != std::string::npos)
				{
					cout << "binary format point cloud loaded! " << endl;
					break;
				}
				else if (line.find(point_type_asc) != std::string::npos)
				{
					cout << "ascii format point cloud loaded! " << endl;
					break;

				}
				else
					continue;
			}

			std::string num_flag = "element vertex";
			int pos = num_flag.size();
			int numPts = 0;
			while (!in.eof()) {
				getline(in, line);
				if (line.find(num_flag) != std::string::npos) {
					std::string num = line.substr(pos);
					numPts = atoi(num.c_str());
					cout << numPts << " points" << endl;
				}
				if (line == "end_header") {
					break;
				}

			}

			//提取点放置到容器Pts中
			float x, y, z;
			/*std::vector<float> Pts;*/
			while (!in.eof()) {
				in >> x >> y >> z;
				std::getline(in, line);
				if (in.fail())
					break;
				cloud_input->push_back(pcl::PointXYZ(x, y, z));
				/*Pts.push_back(x);
				Pts.push_back(y);
				Pts.push_back(z);*/
			}
			in.close();
		}

		//detect the platform
		double detectpintai(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double& maxpintai_x,
			double& minpintai_x, double& maxpintai_y, double& minpintai_y, double& pintai_height,
			Eigen::VectorXf& coefficient1) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pingtai(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pingtai2(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg2(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_save(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud_pingtai2(new pcl::PointCloud<pcl::PointXYZ>);
			cloud_ori = cloud;
			double startthresd = 500, endthresd = 1500;
			clock_t start = clock();
			std::cout << "cloud" << cloud_ori->size() << std::endl;
			for (int i = 0; i < cloud_ori->size(); i++) {
				/*cloud_save->push_back(cloud_ori->points[i]);*/
				if (cloud_ori->points[i].z > startthresd && cloud_ori->points[i].z < endthresd) {
					cloud_pingtai->push_back(cloud_ori->points[i]);
					/*cout << "sss" << endl;*/
				}
			}
			/*pcl::io::savePLYFile("E:\\文档资料\\外包\\OBB\\cloud_pingtai.ply", *cloud_pingtai);*/

			int cout[20] = { 0 }, max = -INT_MAX, flag = 0;
			for (int i = 0; i < cloud_pingtai->size(); i++) {
				cout[int(cloud_pingtai->points[i].z) / 100]++;
			}

			for (int i = 0; i < 20; i++) {
				if (cout[i] > max) {
					max = cout[i];
					flag = i;
				}
			}
			for (int i = 0; i < cloud_ori->size(); i++) {
				if (/*cloud->points[i].x > -400 && cloud->points[i].x < 210 &&
					cloud->points[i].y > -130 && cloud->points[i].y < 340&&*/
					cloud_ori->points[i].z > flag * 100.0 && cloud_ori->points[i].z < (flag + 1) * 100) {
					cloud_seg2->push_back(cloud_ori->points[i]);

				}
			}
			std::cout << "cloud_seg2" << cloud_seg2->size() << std::endl;
			pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane
			(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_seg2));	//指定拟合点云与几何模型
			pcl::RandomSampleConsensus<pcl::PointXYZ> ransac1(model_plane);	//创建随机采样一致性对象
			ransac1.setDistanceThreshold(10);	//内点到模型的最大距离
			ransac1.setMaxIterations(1000);		//最大迭代次数
			ransac1.computeModel();				//执行RANSAC空间直线拟合

			std::vector<int> inliers3;				//存储内点索引的向量
			ransac1.getInliers(inliers3);			//提取内点对应的索引

			Eigen::VectorXf coefficient;
			ransac1.getModelCoefficients(coefficient);
			coefficient1 = coefficient;
			/// 根据索引提取内点
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line1(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud<pcl::PointXYZ>(*cloud_seg2, inliers3, *cloud_plane);


			double maxx = -DBL_MAX, minx = DBL_MAX;
			/*pcl::io::savePLYFile("E:\\文档资料\\外包\\OBB\\cloud_pintai.ply", *cloud_plane);*/

			for (int i = 0; i < cloud_plane->size(); i++) {
				if (cloud_plane->points[i].x > maxx) {
					maxx = cloud_plane->points[i].x;
				}
				if (cloud_plane->points[i].x < minx) {
					minx = cloud_plane->points[i].x;
				}
			}
			int widthgridx = (int)(maxx - minx) / 40 + 1;
			int sum[200] = { 0 };
			int record[200] = { 0 };
			int x;

			//计算每个滑块中的点云数量。
			for (int m = 0; m < cloud_plane->size(); m++) {
				x = (cloud_plane->points[m].x - minx) / 40;
				sum[x]++;
			}
			for (int j = 0; j <= 200; j++) {
				if (sum[j] >= 2000) {
					record[j] = 1;
				}
			}
			//判断点云数量
			for (int m = 0; m < cloud_plane->size(); m++) {
				int x = (cloud_plane->points[m].x - minx) / 40;
				if (record[x] == 1) {
					cloud_pingtai2->push_back(cloud_plane->points[m]);
				}
			}
			/*cloud->clear();*/
			double maxtai_x = -DBL_MAX, mintai_x = DBL_MAX, maxtai_y = -DBL_MAX, mintai_y = DBL_MAX, averagez = 0;
			/*pcl::io::savePLYFile("E:\\文档资料\\外包\\OBB\\cloud_pintai.ply", *cloud_plane);*/

			for (int i = 0; i < cloud_pingtai2->size(); i++) {
				if (cloud_pingtai2->points[i].x > maxtai_x) {
					maxtai_x = cloud_pingtai2->points[i].x;
				}
				if (cloud_pingtai2->points[i].x < mintai_x) {
					mintai_x = cloud_pingtai2->points[i].x;
				}
				if (cloud_pingtai2->points[i].y > maxtai_y) {
					maxtai_y = cloud_pingtai2->points[i].y;
				}
				if (cloud_pingtai2->points[i].y < mintai_y) {
					mintai_y = cloud_pingtai2->points[i].y;

				}
			}

			averagez = (double)averagez / cloud_pingtai2->size();
			std::cout << coefficient1[0] << coefficient1[1] << coefficient1[2] << std::endl;
			pcl::transformPointCloud(*cloud_pingtai2, *trans_cloud_pingtai2, rotate_matrix(Eigen::Vector3f(coefficient1[0], coefficient1[1], coefficient1[2]),
				Eigen::Vector3f(0, 0, -1)));

			for (int i = 0; i < trans_cloud_pingtai2->size(); i++) {
				averagez += trans_cloud_pingtai2->points[i].z;
			}
			averagez = (double)averagez / trans_cloud_pingtai2->size();
			maxpintai_x = maxtai_x, minpintai_x = mintai_x,
				maxpintai_y = maxtai_y, minpintai_y = mintai_y, pintai_height = averagez;


		}

		//compute the size
		void calculatedimensions(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double maxx, double minx,
			double maxy, double miny, double Zvalue, Eigen::VectorXf coefficient1, double& length, double& width, double& height, double& volume) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg2(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud_seg2(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_removepoint(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud_seg_removepoint(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud_seg2_remove(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points(new pcl::PointCloud<pcl::PointXYZ>);

			double max_x = maxx, min_x = minx, max_y = maxy, min_y = miny, Z = Zvalue;
			auto start = clock();
			/*cout << "max_x=" << maxx << ", min_x=" << minx << ", max_y=" << maxy << ", min_y" << miny << ", Z" << Z << endl;*/

			/*cout << max_x << min_x << endl;*/
			for (int i = 0; i < cloud->size(); i++) {
				/*cout <<cloud->points[i].x<<" "<< cloud->points[i].y<<" "<<cloud->points[i].z << endl;*/
				if (cloud->points[i].x > min_x && cloud->points[i].x<max_x
					&& cloud->points[i].y>(min_y + 100) && cloud->points[i].y < max_y) {
					cloud_seg->push_back(cloud->points[i]);
				}
			}

			pcl::transformPointCloud(*cloud_seg, *trans_cloud_seg2, rotate_matrix(Eigen::Vector3f(coefficient1[0], coefficient1[1], coefficient1[2]),
				Eigen::Vector3f(0, 0, -1)));

			for (int i = 0; i < trans_cloud_seg2->size(); i++) {
				if (trans_cloud_seg2->points[i].z > 600 && trans_cloud_seg2->points[i].z < Z - 20) {
					cloud_seg2->push_back(trans_cloud_seg2->points[i]);
				}
			}

			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud(cloud_seg2);
			sor.setMeanK(200);
			sor.setStddevMulThresh(10);
			sor.filter(*cloud_seg_removepoint);
			pcl::transformPointCloud(*cloud_seg_removepoint, *trans_cloud_seg_removepoint, rotate_matrix(Eigen::Vector3f(coefficient1[0], coefficient1[1], coefficient1[2]),
				Eigen::Vector3f(0, 0, -1)));
			double maxtarget_x = -DBL_MAX, mintarget_x = DBL_MAX,
				maxtarget_y = -DBL_MAX, mintarget_y = DBL_MAX,
				maxtarget_z = -DBL_MAX, mintarget_z = DBL_MAX;
			double avgz = 0;
			int avgzcout = 0;
			for (int j = 0; j < trans_cloud_seg_removepoint->size(); j++) {
				if (trans_cloud_seg_removepoint->points[j].z < mintarget_z)
				{
					mintarget_z = trans_cloud_seg_removepoint->points[j].z;
				}
				if (trans_cloud_seg_removepoint->points[j].z > maxtarget_z)
				{
					maxtarget_z = trans_cloud_seg_removepoint->points[j].z;
				}
			}
			for (int j = 0; j < trans_cloud_seg_removepoint->size(); j++) {
				if (trans_cloud_seg_removepoint->points[j].z > mintarget_z &&
					trans_cloud_seg_removepoint->points[j].z < mintarget_z + 1)
				{
					avgz += trans_cloud_seg_removepoint->points[j].z;
					avgzcout += 1;
				}
			}

			//定义储存极值的两个点
			pcl::PointXYZ minPoint, maxPoint;
			pcl::getMinMax3D(*trans_cloud_seg_removepoint, minPoint, maxPoint);
			//输出结果

			length = (double)maxPoint.x - minPoint.x;
			width = (double)maxPoint.y - minPoint.y;
			/*cout << "------------相关参数----------" << endl;
			std::cout << "            lenght=" << length << std::endl;
			std::cout << "            width=" << width << std::endl;*/
			/*pcl::io::savePLYFile("E:\\文档资料\\外包\\OBB\\pc\\trans_cloud_seg_removepoint.ply", *trans_cloud_seg_removepoint);*/

			avgz = (double)avgz / avgzcout;
			height = abs(avgz - Z);


			volume = length * width * height;
			auto end = clock();

			/*cout << "            height=" << height << "mm" << endl;
			cout << "            Dimensions=" << volume << "mm" << endl;
			cout << "            runtime=" << (double)(end - start) / CLOCKS_PER_SEC * 1000 << "ms" << endl;*/
		}

		//Calibrate the platform
		void Calibrate(const pcl::PointCloud<pcl::PointXYZ>::Ptr pintaicloud, double& maxpintai_x, double& minpintai_x,
			double& maxpintai_y, double& minpintai_y, double& pintai_height,
			Eigen::VectorXf& coefficient1) {
			/*std::string pintaifilename = "G:\\waibao\\computeOBB\\platform2.ply";*/
			pcl::PointCloud<pcl::PointXYZ>::Ptr pintaicloud(new pcl::PointCloud<pcl::PointXYZ>);
			/*readPLYFileBinary(pintaifilename, pintaicloud);*/

			detectpintai(pintaicloud, maxpintai_x, minpintai_x, maxpintai_y,
				minpintai_y, pintai_height, coefficient1);
		}





};