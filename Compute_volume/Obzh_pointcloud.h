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
        //����������ݵ�ply
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

        //�����ɫ�������ݵ�ply
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

            //����pipeline
            ob::Pipeline pipeline;

            //��ȡ��ɫ��������������ã��������ķֱ��ʣ�֡�ʣ��Լ�֡�ĸ�ʽ
            auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);

            //ͨ���ӿ����ø���Ȥ����ض�ӦProfile�б���׸�Profile
            auto colorProfile = colorProfiles->getVideoStreamProfile(640, 480, OB_FORMAT_YUYV);
            if (!colorProfile) {
                colorProfile = colorProfiles->getVideoStreamProfile(640, 480, OB_FORMAT_I420);
                if (!colorProfile)
                    colorProfile = colorProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
            }

            //��ȡ�����������������ã��������ķֱ��ʣ�֡�ʣ��Լ�֡�ĸ�ʽ
            auto depthProfiles = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
            //ͨ���ӿ����ø���Ȥ����ض�ӦProfile�б���׸�Profile
            auto depthProfile = depthProfiles->getVideoStreamProfile(640, 480, OB_FORMAT_Y16);
            if (!depthProfile) {
                depthProfile = depthProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
            }

            //ͨ������Config������PipelineҪ���û��߽�����Щ�������ｫ���ò�ɫ���������
            std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
            config->enableStream(colorProfile);
            config->enableStream(depthProfile);

            // ����D2C����, ����RGBD����ʱ��Ҫ����
            if (pipeline.getDevice()->isPropertySupported(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL, OB_PERMISSION_WRITE)) {
                pipeline.getDevice()->setBoolProperty(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL, true);
            }

            // ����
            pipeline.start(config);

            // ��������Filter���󣨵���Filter����ʱ����Pipeline�ڲ���ȡ�豸����, ���Ծ�����Filter����ǰ���ú��豸��
            ob::PointCloudFilter pointCloud;

            auto cameraParam = pipeline.getCameraParam();
            pointCloud.setCameraParam(cameraParam);

            // ������ʾ
            std::cout << "Press R or r to create RGBD PointCloud and save to ply file! " << std::endl;
            std::cout << "Press D or d to create Depth PointCloud and save to ply file! " << std::endl;
            std::cout << "Press ESC to exit! " << std::endl;

            int count = 0;
            while (true) {
                auto frameset = pipeline.waitForFrames(100);
                if (kbhit()) {
                    //��ESC���˳�
                    if (key == KEY_ESC) {
                        break;
                    }
                    if (key == 'R' || key == 'r') {
                        count = 0;
                        //��������ظ�10��
                        while (count++ < retry) {
                            //�ȴ�һ֡���ݣ���ʱʱ��Ϊ100ms
                            auto frameset = pipeline.waitForFrames(100);
                            if (frameset != nullptr && frameset->depthFrame() != nullptr && frameset->colorFrame() != nullptr) {
                                try {
                                    //���ɲ�ɫ���Ʋ�����
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
                        //��������ظ�10��
                        while (count++ < retry) {
                            //�ȴ�һ֡���ݣ���ʱʱ��Ϊ100ms
                            auto frameset = pipeline.waitForFrames(100);
                            if (frameset != nullptr && frameset->depthFrame() != nullptr) {
                                try {
                                    //���ɵ��Ʋ�����
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
                        //��������ظ�10��
                        while (count++ < retry) {
                            //�ȴ�һ֡���ݣ���ʱʱ��Ϊ100ms
                            auto frameset = pipeline.waitForFrames(100);
                            if (frameset != nullptr && frameset->depthFrame() != nullptr) {
                                try {
                                    //���ɵ��Ʋ�����
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
            //ֹͣpipeline
            pipeline.stop();

        }
        catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            std::cout<<"ERROR!"<<endl;
        }
        
     
   
};
