#pragma once
#include "libobsensor/ObSensor.hpp"
#include <pcl/point_types.h>
//#include "opencv2/opencv.hpp"
#include <fstream>
#include <iostream>
#if defined(__linux__)
#include "../conio.h"
#else
#include <conio.h>
#endif

#define KEY_ESC 27
#define KEY_R 82
#define KEY_r 114
class Obzh_pointcloud {

    pcl::PointCloud<pcl::PointXYZ>::Ptr pingtai_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    public:
        //保存点云数据到ply
        void savePointsToPly(std::shared_ptr<ob::Frame> frame, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output, std::string fileName) {
            int   pointsSize = frame->dataSize() / sizeof(OBPoint);
            FILE* fp = fopen(fileName.c_str(), "wb+");
            fprintf(fp, "ply\n");
            fprintf(fp, "format ascii 1.0\n");
            fprintf(fp, "element vertex %d\n", pointsSize);
            fprintf(fp, "property float x\n");
            fprintf(fp, "property float y\n");
            fprintf(fp, "property float z\n");
            fprintf(fp, "end_header\n");

            OBPoint* point = (OBPoint*)frame->data();
            cloud_output->width = pointsSize;
            cloud_output->height = 1;
            cloud_output->resize(cloud_output->width * cloud_output->height);
            for (int i = 0; i < pointsSize; i++) {
                fprintf(fp, "%.3f %.3f %.3f\n", point->x, point->y, point->z);
                cloud_output->push_back(pcl::PointXYZ(point->x, point->y, point->z));
                point++;
            }

            fflush(fp);
            fclose(fp);
        }

        //保存彩色点云数据到ply
        void saveRGBPointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
            int   pointsSize = frame->dataSize() / sizeof(OBColorPoint);
            FILE* fp = fopen(fileName.c_str(), "wb+");
            fprintf(fp, "ply\n");
            fprintf(fp, "format ascii 1.0\n");
            fprintf(fp, "element vertex %d\n", pointsSize);
            fprintf(fp, "property float x\n");
            fprintf(fp, "property float y\n");
            fprintf(fp, "property float z\n");
            fprintf(fp, "property uchar red\n");
            fprintf(fp, "property uchar green\n");
            fprintf(fp, "property uchar blue\n");
            fprintf(fp, "end_header\n");

            OBColorPoint* point = (OBColorPoint*)frame->data();
            for (int i = 0; i < pointsSize; i++) {
                fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", point->x, point->y, point->z, (int)point->r, (int)point->g, (int)point->b);
                point++;
            }

            fflush(fp);
            fclose(fp);
        }
        void SelectPointType(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int code,int key,int retry) try{
            ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);

            //创建pipeline
            ob::Pipeline pipeline;

            //获取彩色相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
            auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);

            //通过接口设置感兴趣项，返回对应Profile列表的首个Profile
            auto colorProfile = colorProfiles->getVideoStreamProfile(640, 480, OB_FORMAT_YUYV);
            if (!colorProfile) {
                colorProfile = colorProfiles->getVideoStreamProfile(640, 480, OB_FORMAT_I420);
                if (!colorProfile)
                    colorProfile = colorProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
            }

            //获取深度相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
            auto depthProfiles = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
            //通过接口设置感兴趣项，返回对应Profile列表的首个Profile
            auto depthProfile = depthProfiles->getVideoStreamProfile(640, 480, OB_FORMAT_Y16);
            if (!depthProfile) {
                depthProfile = depthProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
            }

            //通过创建Config来配置Pipeline要启用或者禁用哪些流，这里将启用彩色流和深度流
            std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
            config->enableStream(colorProfile);
            config->enableStream(depthProfile);

            // 开启D2C对齐, 生成RGBD点云时需要开启
            if (pipeline.getDevice()->isPropertySupported(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL, OB_PERMISSION_WRITE)) {
                pipeline.getDevice()->setBoolProperty(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL, true);
            }

            // 启动
            pipeline.start(config);

            // 创建点云Filter对象（点云Filter创建时会在Pipeline内部获取设备参数, 所以尽量在Filter创建前配置好设备）
            ob::PointCloudFilter pointCloud;

            auto cameraParam = pipeline.getCameraParam();
            pointCloud.setCameraParam(cameraParam);

            // 操作提示
            std::cout << "Press R or r to create RGBD PointCloud and save to ply file! " << std::endl;
            std::cout << "Press D or d to create Depth PointCloud and save to ply file! " << std::endl;
            std::cout << "Press ESC to exit! " << std::endl;

            int count = 0;
            while (true) {
                auto frameset = pipeline.waitForFrames(100);
                if (kbhit()) {
                    //按ESC键退出
                    if (key == KEY_ESC) {
                        break;
                    }
                    if (key == 'R' || key == 'r') {
                        count = 0;
                        //限制最多重复10次
                        while (count++ < retry) {
                            //等待一帧数据，超时时间为100ms
                            auto frameset = pipeline.waitForFrames(100);
                            if (frameset != nullptr && frameset->depthFrame() != nullptr && frameset->colorFrame() != nullptr) {
                                try {
                                    //生成彩色点云并保存
                                    std::cout << "Save RGBD PointCloud ply file..." << std::endl;
                                    pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
                                    std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
                                    saveRGBPointsToPly(frame, "RGBPoints.ply");
                                    std::cout << "RGBPoints.ply Saved" << std::endl;
                                }
                                catch (std::exception& e) {
                                    std::cout << "Get point cloud failed" << std::endl;
                                }
                                break;
                            }
                        }
                    }
                    else if (key == 'D' &&code==1) {
                        count = 0;
                        //限制最多重复10次
                        while (count++ < retry) {
                            //等待一帧数据，超时时间为100ms
                            auto frameset = pipeline.waitForFrames(100);
                            if (frameset != nullptr && frameset->depthFrame() != nullptr) {
                                try {
                                    //生成点云并保存
                                    std::cout << "Save Depth PointCloud to ply file..." << std::endl;
                                    pointCloud.setCreatePointFormat(OB_FORMAT_POINT);
                                    std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
                                    savePointsToPly(frame, cloud, "DepthPoints.ply");
                                    std::cout << "DepthPoints.ply Saved" << std::endl;
                                }
                                catch (std::exception& e) {
                                    std::cout << "Get point cloud failed" << std::endl;
                                }
                                break;
                            }
                        }
                    }
                    else if (key == 'D' &&code==2) {
                        count = 0;
                        //限制最多重复10次
                        while (count++ < retry) {
                            //等待一帧数据，超时时间为100ms
                            auto frameset = pipeline.waitForFrames(100);
                            if (frameset != nullptr && frameset->depthFrame() != nullptr) {
                                try {
                                    //生成点云并保存
                                    std::cout << "Save Depth PointCloud to ply file..." << std::endl;
                                    pointCloud.setCreatePointFormat(OB_FORMAT_POINT);
                                    std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
                                    savePointsToPly(frame, cloud, "DepthPoints.ply");
                                    std::cout << "DepthPoints.ply Saved" << std::endl;
                                }
                                catch (std::exception& e) {
                                    std::cout << "Get point cloud failed" << std::endl;
                                }
                                break;
                            }
                        }
                    }
                }
            }
            //停止pipeline
            pipeline.stop();

        }
        catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            std::cout<<"ERROR!"<<endl;
        }
        
     
   
};
