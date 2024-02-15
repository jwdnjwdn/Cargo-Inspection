#include <iostream>
#include "Compute_point.h"
#include "Obzh_pointcloud.h"
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "json/value.h"
#include "json/reader.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <json/json.h>
#include <ws2ipdef.h>

using namespace pcl;

int main()
{
	std::cout << "This is server" << std::endl;
	// socket
	int listenfd = socket(AF_INET, SOCK_STREAM, 0);
	if (listenfd == -1) {
		std::cout << "Error: socket" << std::endl;
		return 0;
	}
	// bind
	struct sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(8000);
	addr.sin_addr.s_addr = INADDR_ANY;
	if (bind(listenfd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
		std::cout << "Error: bind" << std::endl;
		return 0;
	}
	// listen
	if (listen(listenfd, 5) == -1) {
		std::cout << "Error: listen" << std::endl;
		return 0;
	}


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	double maxpintai_x[3] = { 0 }, minpintai_x[3] = { 0 }, maxpintai_y[3] = { 0 }, minpintai_y[3] = { 0 }, pintai_height[3] = { 0 };
	double height[3] = { 0 }, width[3] = { 0 }, length[3] = { 0 }, volume[3] = { 0 };
	double avgmaxx = 0, avgminx = 0, avgmaxy = 0, avgminy = 0, pintai_H = 0,
		avglength = 0, avgwidth = 0, avgheight = 0, avgvolume = 0;
	int retry = 0, key = 0, stable_read = 0, tolerance = 0, code;
	Eigen::VectorXf coefficient1;
	double tolerance = 0;

	Eigen::VectorXf coefficient1;
	// accept
	int conn;
	char clientIP[INET_ADDRSTRLEN] = "";
	struct sockaddr_in clientAddr;
	socklen_t clientAddrLen = sizeof(clientAddr);
	const char* sendData="hello, client!";
	while (true) {
		std::cout << "...listening" << std::endl;
		conn = accept(listenfd, (struct sockaddr*)&clientAddr, &clientAddrLen);
		if (conn < 0) {
			std::cout << "Error: accept" << std::endl;
			continue;
		}
		inet_ntop(AF_INET, &clientAddr.sin_addr, clientIP, INET_ADDRSTRLEN);
		std::cout << "...connect " << clientIP << ":" << ntohs(clientAddr.sin_port) << std::endl;

		char buf[255];
		std::string strretry;
		std::string strstable_read;
		std::string strtolerance;

		Json::Value val;
		Json::Reader reader;

		Json::Value root;
		std::string information;


		Json::FastWriter fast;
		while (true) {
			memset(buf, 0, sizeof(buf));
			int len = recv(conn, buf, sizeof(buf), 0);
			
			if (reader.parse(buf, val))
			{
				strretry = val["RETRY"].asInt();
				strstable_read = val["STABLE_READ"].asString();
				strtolerance = val["TOLERANCE"].asInt();
			}
			retry = atoi(strretry.c_str());
			stable_read = atoi(strstable_read.c_str());
			tolerance = atoi(strtolerance.c_str());

			std::cout << retry << " " << stable_read << " " << tolerance << std::endl;
			buf[len] = '\0';
			if (strcmp(buf, "exit") == 0) {
				std::cout << "...disconnect " << clientIP << ":" << ntohs(clientAddr.sin_port) << std::endl;
				break;
			}
			Compute_point compute;
			Obzh_pointcloud ob;


			for (int i = 0; i < retry; i++) {
				ob.SelectPointType(cloud, code, key, retry);
				compute.detectpintai(cloud, maxpintai_x[i], minpintai_x[i], maxpintai_y[i], minpintai_y[3], pintai_height[3], coefficient1);
				cloud->clear();
			}
			if (abs(maxpintai_x[0] - maxpintai_x[1]) < tolerance && abs(maxpintai_x[0] - maxpintai_x[2]) < tolerance && abs(maxpintai_x[1] - maxpintai_x[2]) < tolerance
				&& abs(minpintai_x[0] - minpintai_x[1]) < tolerance && abs(minpintai_x[0] - minpintai_x[2]) < tolerance && abs(minpintai_x[1] - minpintai_x[2]) < tolerance
				&& abs(maxpintai_y[0] - maxpintai_y[1]) < tolerance && abs(maxpintai_y[0] - maxpintai_y[2]) < tolerance && abs(maxpintai_y[1] - maxpintai_y[2]) < tolerance
				&& abs(minpintai_y[0] - minpintai_y[1]) < tolerance && abs(minpintai_y[0] - minpintai_y[1]) < tolerance && abs(minpintai_y[0] - minpintai_y[1]) < tolerance
				&& abs(pintai_height[0] - pintai_height[1]) < tolerance && abs(pintai_height[0] - pintai_height[1]) < tolerance && abs(pintai_height[0] - pintai_height[1]) < tolerance) {
				avgmaxx = (maxpintai_x[0] + maxpintai_x[1] + maxpintai_x[2]) / 3;
				avgminx = (minpintai_y[0] + minpintai_y[1] + minpintai_y[2]) / 3;
				avgmaxy = (maxpintai_y[0] + maxpintai_y[1] + maxpintai_y[2]) / 3;
				avgminy = (minpintai_y[0] + minpintai_y[1] + minpintai_y[2]) / 3;
				pintai_H = (pintai_height[0] + pintai_height[1] + pintai_height[2]);
			}
			else {
				return ERROR("detection of platform ERROR");
				sendData = "detection of platform ERROR!";
			}

			for (int i = 0; i < retry; i++) {
				ob.SelectPointType(cloud, code, key, retry);
				compute.calculatedimensions(cloud, avgmaxx, avgminx, avgmaxy, avgminy, pintai_H, coefficient1, length[i], width[i], height[i], volume[i]);
				cloud->clear();
			}
			if (abs(length[0] - length[1]) < tolerance && abs(length[0] - length[2]) < tolerance && abs(length[1] - length[2]) < tolerance
				&& abs(width[0] - width[1]) < tolerance && abs(width[0] - width[2]) < tolerance && abs(width[1] - width[2]) < tolerance
				&& abs(height[0] - height[1]) < tolerance && abs(height[0] - height[2]) < tolerance && abs(height[1] - height[2]) < tolerance
				) {
				avglength = (length[0] + length[1] + length[2]) / 3;
				avgwidth = (width[0] + width[1] + width[2]) / 3;
				avgheight = (height[0] + height[1] + height[2]) / 3;
				avgvolume = (volume[0] + volume[1] + volume[2])/3;
				
				root["length"] = Json::Value(avglength);
				root["width"] = Json::Value(avgwidth);
				root["height"] = Json::Value(avgheight);
				root["volume"] = Json::Value(avgvolume);
				information=fast.write(root);
				sendData = information.c_str();
			}
			else {
				return ERROR("Detection ERROR!");
				sendData = "Detection ERROR!";
			}
			
			
			send(conn, sendData, len, 0);
		}
		close(conn);
	}

	close(listenfd);

	return 0;


}



