/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 30-01-2021
 *  Author: LN

 *  config setting for Tanway LIDARs
**************************************************/
#include <LaunchConfig.h>
#include "../sdk/TanwayLidarSDK.h"

bool g_enable = false;
std::vector<int> g_AngleRanges;
std::vector<int> g_ChannelRanges;
double g_config1 = 0.3;
double g_config2 = 0.3;
std::vector<int> g_DiscardPoint;

LaunchConfig::LaunchConfig()
{

}

LaunchConfig::~LaunchConfig()
{

}
void LaunchConfig::ReadLaunchParams(ros::NodeHandle& nh_private)
{
	nh_private.param<std::string>("ConnectType", m_connectType, "on-line");
	nh_private.param<std::string>("PcapFilePath", m_filePath, "");
	nh_private.param<std::string>("LocalHost", m_localHost, "192.168.111.204");
	nh_private.param<int>("LocalPointloudPort", m_localPointCloudPort, 5600);
	nh_private.param<int>("LocalDIFPort", m_localDIFPort, 5700);

	nh_private.param<std::string>("LidarHost", m_lidarHost, "192.168.111.51");
	//nh_private.param<int>("LidarPort", m_lidarPort, 5050);

	nh_private.param<std::string>("frame_id", m_frameID, "TanwayTP");
	nh_private.param<std::string>("topic", m_topic, "/tanwaylidar_pointcloud");
	nh_private.param<std::string>("imu_topic", m_imuTopic, "/tanwaylidar_imu");

	nh_private.param<int>("LidarType", m_lidarType, -1);

	//transform
	nh_private.param<double>("TransformRotateX", m_transformRotateX, 0);
	nh_private.param<double>("TransformRotateY", m_transformRotateY, 0);
	nh_private.param<double>("TransformRotateZ", m_transformRotateZ, 0);
	nh_private.param<double>("TransformMoveX", m_transformMoveX, 0);
	nh_private.param<double>("TransformMoveY", m_transformMoveY, 0);
	nh_private.param<double>("TransformMoveZ", m_transformMoveZ, 0);

	//TSP03-32
	if (LT_TSP0332 == m_lidarType)
	{
		nh_private.param<double>("CorrectedAngle1", m_correctedAngle1, 0);
		nh_private.param<double>("CorrectedAngle2", m_correctedAngle2, -6.0);
	}
	//Scope-192
	else if (LT_Scope192 == m_lidarType)
	{
		nh_private.param<double>("CorrectedAngle1", m_correctedAngle1, 0);
		nh_private.param<double>("CorrectedAngle2", m_correctedAngle2, -0.12);
		nh_private.param<double>("CorrectedAngle3", m_correctedAngle3, -0.24);
	}
	//ScopeMini-A2-192
	else if (LT_ScopeMiniA2_192 == m_lidarType)
	{
		nh_private.param<double>("CorrectedAngle1", m_correctedAngle1, 0);
		nh_private.param<double>("CorrectedAngle2", m_correctedAngle2, 0.1);
		nh_private.param<double>("CorrectedAngle3", m_correctedAngle3, 0.2);
	}
	//Scope256-polar
	else if(LT_Scope256_Polar == m_lidarType)
	{
		nh_private.getParam("Enable",g_enable);
		if(g_enable){
			nh_private.getParam("AngleRanges",g_AngleRanges);
			nh_private.getParam("ChannelRanges",g_ChannelRanges);
   			nh_private.getParam("Config_1", g_config1);
			nh_private.getParam("Config_2", g_config2);
			nh_private.getParam("DiscardPoint",g_DiscardPoint);

			if(g_AngleRanges.size() != g_ChannelRanges.size() || g_ChannelRanges.size() % 2 != 0 || g_DiscardPoint.size() % 3 != 0){
				ROS_WARN("yaml param is invalid!");
				g_enable = false;
			}
		}
	}
}

