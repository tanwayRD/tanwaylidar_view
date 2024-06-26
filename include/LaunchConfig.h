
/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 15-03-2022
 *  Author: LN

 *  config setting for Tanway LIDARs
**************************************************/

#ifndef LAUNCHCONFIG_H_
#define LAUNCHCONFIG_H_

#include <strings.h>
#include <ros/ros.h> //generic C++ stuff

class LaunchConfig
{
public:
	LaunchConfig();
	~LaunchConfig();

	void ReadLaunchParams(ros::NodeHandle& nh_private);

public:
	std::string m_connectType = "";
	std::string m_filePath = "";
	std::string m_localHost = "" ;
	std::string m_lidarHost = "" ;
	int m_localPointCloudPort = -1;
	int m_localDIFPort = -1;
	//int m_lidarPort = -1;
	std::string m_frameID = "TanwayTP" ;
	std::string m_topic = "/tanwaylidar_pointcloud" ;
	std::string m_imuTopic = "/tanwaylidar_imu" ;

	int m_lidarType = -1;

	double m_correctedAngle1 = 0;
	double m_correctedAngle2 = 0;
	double m_correctedAngle3 = 0;

	//transform
	double m_transformRotateX = 0;
	double m_transformRotateY = 0;
	double m_transformRotateZ = 0;
	double m_transformMoveX = 0;
	double m_transformMoveY = 0;
	double m_transformMoveZ = 0;

	//jointabc
	bool m_bJointabc = false;
	double m_jointabc_node1 = 0;
	double m_jointabc_node2 = 0;
	int m_jointabc_one_face = 0;
	int m_jointabc_two_face = 0;
};

#endif
