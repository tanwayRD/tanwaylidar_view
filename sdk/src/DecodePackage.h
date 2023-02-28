/*
* Software License Agreement (BSD License)
*
*  Copyright (c) Tanway science and technology co., LTD.
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without modification,
*  are permitted provided  that the following conditions are met:
*
*   1.Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*
*   2.Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*
*   3.Neither the name of the copyright holder(s) nor the names of its  contributors
*     may be used to endorse or promote products derived from this software without
*     specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
*  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
#pragma once
#include "PackageCache.h"
#include "TWException.h"


#define MemberCheck(member) \
template<typename T>\
struct has_member_##member{\
    template <typename _T>static auto check(_T)->typename std::decay<decltype(_T::member)>::type;\
    static void check(...);\
    using type=decltype(check(std::declval<T>()));\
    enum{value=!std::is_void<type>::value};\
};

MemberCheck(x)
MemberCheck(y)
MemberCheck(z)
MemberCheck(intensity)
MemberCheck(distance)
MemberCheck(channel)
MemberCheck(angle)
MemberCheck(echo)
MemberCheck(sepIndex)
MemberCheck(faceIndex)
MemberCheck(color)
MemberCheck(t_sec)
MemberCheck(t_usec)

#define PointT_HsaMember(C, member) has_member_##member<C>::value


template <typename PointT>
class DecodePackage
{
public:
	DecodePackage(std::shared_ptr<PackageCache> packageCachePtr, TWLidarType lidarType, std::mutex* mutex);
	DecodePackage(){};
	virtual ~DecodePackage();

	void Start();
	void RegPointCloudCallback(const std::function<void(typename TWPointCloud<PointT>::Ptr)>& callback);
	void RegGPSCallback(const std::function<void(const std::string&)>& callback);
	void RegExceptionCallback(const std::function<void(const TWException&)>& callback);

	void SetPackageCache(std::shared_ptr<PackageCache> packageCachePtr){ m_packageCachePtr = packageCachePtr; }
	void SetLidarType(TWLidarType lidarType){m_lidarType = lidarType;}
	void SetCorrectionAngleToTSP0332(float angle1, float angle2);
	void SetCorrectionAngleToScope192(float angle1, float angle2, float angle3);
	void SetCorrectionAngleToScopeMiniA2_192(float angle1, float angle2, float angle3);
	void SetSeparateDistance(float sepValue);

	void SetMutex(std::mutex* mutex){ m_mutex = mutex; }

private:
	void BeginDecodePackageData();
	

	void DecodeTensorLite(char* udpData, unsigned int t_sec, unsigned int t_usec);
	void DecodeTensorPro(char* udpDataOri, unsigned int t_sec, unsigned int t_usec);
	void DecodeTensorPro_echo2(char* udpData, unsigned int t_sec, unsigned int t_usec);
	void DecodeScope(char* udpDataOri);
	void DecodeTensorPro0332(char* udpDataOri, unsigned int t_sec, unsigned int t_usec);
	void DecodeScope192(char* udpDataOri);
	void DecodeScopeMiniA2_192(char* udpDataOri);

	void DecodeGPSData(char* udpData);	//decode gps date


protected:
	virtual void UseDecodePointPro(int echo, double horAngle, int channel, float hexL, float hexPulseWidth, int offset, char* data, unsigned int t_sec, unsigned int t_usec);
	virtual void UseDecodePointTSP03_32(int echo, double horAngle, int channel, float hexL, float hexPulseWidth, int offset, char* data, unsigned int t_sec, unsigned int t_usec);
	virtual void UseDecodePointScope(int echo, int sepIndex, int faceIndex, double horAngle, int channel, float hexL, float hexPulseWidth);
	virtual void UseDecodePointScope_192(int echo, int sepIndex, int faceIndex, double horAngle, int channel, float hexL, float hexPulseWidth);
	virtual void UseDecodePointScopeMiniA2_192(int echo, int sepIndex, int faceIndex, double horAngle, int channel, float hexL, float hexPulseWidth);
	virtual void ProcessPointCloud(){};

protected:
	int FourHexToInt(unsigned char high, unsigned char highmiddle, unsigned char middle, unsigned char low);
	int TwoHextoInt(unsigned char high, unsigned char low);
	void FilterDisturbPoint(char* blockPrev, char* blockNext, char* blockMid, int protocol, double intervalAngle);

public:
	double m_startAngle = 30.0;
	double m_endAngle = 150.0;
	double m_scopeABaddCSeparateDist = 4.0;
protected:
	double m_firstSeparateAngle = -1;
	double m_calRA = (float)(3.14159265f / 180.0f);
	double m_calPulse = 0.032;
	//double m_calSimple = 500 * 2.997924 / 10.f / 16384.f / 2;  普通TDC
	double m_calSimple = 0.032 * 2.997924 / 10.f / 2; //FPGA-TDC

	//192 valid correction
	double m_corHeight = -1.15;
	double m_corHeightUp = 0.08;
	double m_corHeightDown = 0.08;
	double m_corHeightRange = 0.1;
	int m_corChannel = 32;
	double m_channelValidDistance = 4.0;
	bool m_channelValid[64] = {true};

	//TSP03-32、Scope162 temporary variable
	double x_cal_1 = 0;
	double x_cal_2 = 0;
	double y_cal_1 = 0;
	double y_cal_2 = 0;
	double z_cal_1 = 0;
	double z_cal_2 = 0;

	//Tensor
	double m_verticalChannelAngle16[16] =
	{
		-5.274283f, -4.574258f,	-3.872861f, -3.1703f, -2.466783f, -1.762521f, -1.057726f, -0.352611f,
		0.352611f, 1.057726f, 1.762521f, 2.466783f, 3.1703f, 3.872861f, 4.574258f, 5.274283f
	};
	double m_verticalChannelAngle16_cos_vA_RA[16] = { 0.0 };
	double m_verticalChannelAngle16_sin_vA_RA[16] = { 0.0 };
	double m_skewing_sin_tsp[2] = { 0.0 };
	double m_skewing_cos_tsp[2] = { 0.0 };

	//Scope
	double m_verticalChannelAngle64[64] =
	{
		-14.64f, -14.17f, -13.69f, -13.22f, -12.75f, -12.28f, -11.81f, -11.34f, -10.87f, -10.40f, -9.93f, -9.47f, -9.00f, -8.54f, -8.07f, -7.61f, -7.14f, -6.68f, -6.22f, -5.76f, -5.29f, -4.83f, -4.37f, -3.91f, -3.45f, -2.99f, -2.53f, -2.07f, -1.61f, -1.15f, -0.69f, -0.23f,
		0.23f, 0.69f, 1.15f, 1.61f, 2.07f, 2.53f, 2.99f, 3.45f, 3.91f, 4.37f, 4.83f, 5.29f, 5.76f, 6.22f, 6.68f, 7.14f, 7.61f, 8.07f, 8.54f, 9.00f, 9.47f, 9.93f, 10.40f, 10.87f, 11.34f, 11.81f, 12.28f, 12.75f, 13.22f, 13.69f, 14.17f, 14.64f
	};
	double m_verticalChannelAngle64_cos_vA_RA[64] = { 0.0 };
	double m_verticalChannelAngle64_sin_vA_RA[64] = { 0.0 };
	double m_skewing_sin_scope[3] = { 0.0 };
	double m_skewing_cos_scope[3] = { 0.0 };
	double m_rotate_scope_sin = sin(-10.0 * m_calRA);
	double m_rotate_scope_cos = cos(-10.0 * m_calRA);
	double ScopeA_Elevation_A = 0.0;
	double ScopeB_Elevation_A = 0.12;
	double ScopeC_Elevation_A = 0.24;
	double ScopeA_Elevation_A_Correct = 0.0;
	double ScopeB_Elevation_A_Correct = 0.12;
	double ScopeC_Elevation_A_Correct = 0.24;

	//ScopeMiniA2
	float m_verticalChannelAngle_Scope64_A2[64] =
	{
		-12.368f, -11.986f, -11.603f, -11.219f, -10.834f, -10.448f, -10.061f, -9.674f, -9.285f, -8.896f, -8.505f, -8.115f, -7.723f, -7.331f, -6.938f, -6.545f, -6.151f, -5.756f, -5.361f, -4.966f, -4.570f, -4.174f, -3.777f, -3.381f, -2.983f, -2.586f, -2.189f, -1.791f, -1.393f, -0.995f, -0.597f, -0.199f,
		0.199f, 0.597f, 0.995f, 1.393f, 1.791f, 2.189f, 2.586f, 2.983f, 3.381f, 3.777f, 4.174f, 4.570f, 4.966f, 5.361f, 5.756f, 6.151f, 6.545f, 6.938f, 7.331f, 7.723f, 8.115f, 8.505f, 8.896f, 9.285f, 9.674f, 10.061f, 10.448f, 10.834f, 11.219f, 11.603f, 11.986f, 12.368f
	};
	double m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[64] = { 0.f };
	double m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[64] = { 0.f };
	double m_skewing_scopeMiniA2_Angle[3] = {0.0, 0.1, 0.2};
	double m_skewing_scopeMiniA2_Angle_Correct[3] = {0.0, 0.1, 0.2};
	double m_skewing_sin_scopeMiniA2_192[3] = { 0.0 };
	double m_skewing_cos_scopeMiniA2_192[3] = { 0.0 };


private:
	std::shared_ptr<PackageCache> m_packageCachePtr;
	TWLidarType m_lidarType;
	std::atomic<bool>  run_decode;
	std::atomic<bool>  run_exit;
	std::mutex* m_mutex;

	char m_cashUDPData[UDP_MAX_LENGTH] = { 0 };
	int m_cashUDPLength = 0;

	std::function<void(typename TWPointCloud<PointT>::Ptr)> m_funcPointCloud = NULL;
	std::function<void(const std::string&)> m_funcGPS = NULL;
	std::function<void(const TWException&)> m_funcException = NULL;

public:
	typename TWPointCloud<PointT>::Ptr m_pointCloudPtr;
};

template <typename PointT>
void DecodePackage<PointT>::RegPointCloudCallback(const std::function<void(typename TWPointCloud<PointT>::Ptr)>& callback)
{
	m_funcPointCloud = callback;
}

template <typename PointT>
void DecodePackage<PointT>::RegGPSCallback(const std::function<void(const std::string&)>& callback)
{
	m_funcGPS = callback;
}

template <typename PointT>
void DecodePackage<PointT>::RegExceptionCallback(const std::function<void(const TWException&)>& callback)
{
	m_funcException = callback;
}

template <typename PointT>
void DecodePackage<PointT>::SetCorrectionAngleToScope192(float angle1, float angle2, float angle3)
{
	ScopeA_Elevation_A_Correct = angle1 + (ScopeA_Elevation_A_Correct - ScopeA_Elevation_A);
	ScopeB_Elevation_A_Correct = angle2 + (ScopeB_Elevation_A_Correct - ScopeB_Elevation_A);
	ScopeC_Elevation_A_Correct = angle3 + (ScopeC_Elevation_A_Correct - ScopeC_Elevation_A);

	m_skewing_sin_scope[0] = sin(ScopeA_Elevation_A_Correct * m_calRA);
	m_skewing_sin_scope[1] = sin(ScopeB_Elevation_A_Correct * m_calRA);
	m_skewing_sin_scope[2] = sin(ScopeC_Elevation_A_Correct * m_calRA);

	m_skewing_cos_scope[0] = cos(ScopeA_Elevation_A_Correct * m_calRA);
	m_skewing_cos_scope[1] = cos(ScopeB_Elevation_A_Correct * m_calRA);
	m_skewing_cos_scope[2] = cos(ScopeC_Elevation_A_Correct * m_calRA);
}

template <typename PointT>
void DecodePackage<PointT>::SetCorrectionAngleToScopeMiniA2_192(float angle1, float angle2, float angle3)
{
	m_skewing_scopeMiniA2_Angle[0] = angle1;
	m_skewing_scopeMiniA2_Angle[1] = angle2;
	m_skewing_scopeMiniA2_Angle[2] = angle3;
	m_skewing_scopeMiniA2_Angle_Correct[0] = angle1;
	m_skewing_scopeMiniA2_Angle_Correct[1] = angle2;
	m_skewing_scopeMiniA2_Angle_Correct[2] = angle3;

	m_skewing_sin_scopeMiniA2_192[0] = sin(angle1 * m_calRA);
	m_skewing_sin_scopeMiniA2_192[1] = sin(angle2 * m_calRA);
	m_skewing_sin_scopeMiniA2_192[2] = sin(angle3 * m_calRA);

	m_skewing_cos_scopeMiniA2_192[0] = cos(angle1 * m_calRA);
	m_skewing_cos_scopeMiniA2_192[1] = cos(angle2 * m_calRA);
	m_skewing_cos_scopeMiniA2_192[2] = cos(angle3 * m_calRA);
}

template <typename PointT>
void DecodePackage<PointT>::SetSeparateDistance(float sepValue)
{
	m_scopeABaddCSeparateDist = sepValue;
}

template <typename PointT>
void DecodePackage<PointT>::SetCorrectionAngleToTSP0332(float angle1, float angle2)
{
	m_skewing_sin_tsp[0] = sin(angle1 * m_calRA);
	m_skewing_sin_tsp[1] = sin(angle2 * m_calRA);  //-6.0

	m_skewing_cos_tsp[0] = cos(angle1 * m_calRA);
	m_skewing_cos_tsp[1] = cos(angle2 * m_calRA);
}

template <typename PointT>
int DecodePackage<PointT>::TwoHextoInt(unsigned char high, unsigned char low)
{
	int addr = low & 0xFF;
	addr |= ((high << 8) & 0XFF00);
	return addr;
}

template <typename PointT>
int DecodePackage<PointT>::FourHexToInt(unsigned char high, unsigned char highmiddle, unsigned char middle, unsigned char low)
{
	int addr = low & 0xFF;
	addr |= ((middle << 8) & 0xFF00);
	addr |= ((highmiddle << 16) & 0xFF0000);
	addr |= ((high << 24) & 0xFF000000);
	return addr;
}

template <typename PointT>
void DecodePackage<PointT>::FilterDisturbPoint(char* blockPrev, char* blockNext, char* blockMid, int protocol, double intervalAngle)
{
	//计算block_prev距离值
	double prevDistance[16] = { 0 };
	double midDistance[16] = { 0 };
	double nextDistance[16] = { 0 };

	if (102 == protocol) //Scope
	{
		//提取block_prev 水平角度值
		//角度值 （索引：128-131）  GPS值 （索引：132-135）  预留值（索引：136-139）
		unsigned int prevHextoAngle = FourHexToInt(blockPrev[136 - 8], blockPrev[136 - 7], blockPrev[136 - 6], blockPrev[136 - 5]);
		double prevHorAngle = prevHextoAngle * 0.00001;//getHorizontalAngle(prevHextoAngle);
		//提取block_mid 水平角度值
		//unsigned int midHextoAngle = FourHexToInt(blockMid[136 - 8], blockMid[136 - 7], blockMid[136 - 6], blockMid[136 - 5]);
		//float midHorAngle = getHorizontalAngle(midHextoAngle);
		//提取block_next 水平角度值
		unsigned int nextHextoAngle = FourHexToInt(blockNext[136 - 8], blockNext[136 - 7], blockNext[136 - 6], blockNext[136 - 5]);
		double nextHorAngle = nextHextoAngle * 0.00001;//getHorizontalAngle(nextHextoAngle);

		if (fabs(prevHorAngle - nextHorAngle) > 30) return;

		//计算block_prev距离值
		for (int seq = 0; seq < 16; seq++)
		{
			//计算距离 单回波
			double hexToInt1 = TwoHextoInt(blockPrev[seq * 8 + 0], blockPrev[seq * 8 + 1]);
			prevDistance[seq] = hexToInt1 * m_calSimple;
		}

		//计算block_mid距离值
		for (int seq = 0; seq < 16; seq++)
		{
			//计算距离 单回波
			double hexToInt1 = TwoHextoInt(blockMid[seq * 8 + 0], blockMid[seq * 8 + 1]);
			midDistance[seq] = hexToInt1 * m_calSimple;
		}

		//计算block_next距离值
		for (int seq = 0; seq < 16; seq++)
		{
			//计算距离 单回波
			double hexToInt1 = TwoHextoInt(blockNext[seq * 8 + 0], blockNext[seq * 8 + 1]);
			nextDistance[seq] = hexToInt1 * m_calSimple;
		}
	}
	else if (110 == protocol)
	{
		//提取block_prev 水平角度值
		unsigned int prevHextoAngle = FourHexToInt(blockPrev[64], blockPrev[65], blockPrev[66], blockPrev[67]);
		double prevHorAngle = prevHextoAngle * 0.00001;// getHorizontalAngle(prevHextoAngle);
		//提取block_mid 水平角度值
		//unsigned int midHextoAngle = FourHexToInt(blockMid[64], blockMid[65], blockMid[66], blockMid[67]);
		//float midHorAngle = getHorizontalAngle(midHextoAngle);
		//提取block_next 水平角度值
		unsigned int nextHextoAngle = FourHexToInt(blockNext[64], blockNext[65], blockNext[66], blockNext[67]);
		double nextHorAngle = nextHextoAngle * 0.00001; //getHorizontalAngle(nextHextoAngle);

		if (fabs(prevHorAngle - nextHorAngle) > 30) return;

		//计算block_prev距离值
		for (int seq = 0; seq < 16; seq++)
		{
			//计算距离
			double hexToInt1 = TwoHextoInt(blockPrev[seq * 4 + 0], blockPrev[seq * 4 + 1]);
			prevDistance[seq] = hexToInt1 * m_calSimple;
		}
		//计算block_mid距离值
		for (int seq = 0; seq < 16; seq++)
		{
			//计算距离
			double hexToInt1 = TwoHextoInt(blockMid[seq * 4 + 0], blockMid[seq * 4 + 1]);
			midDistance[seq] = hexToInt1 * m_calSimple;
		}
		//计算block_next距离值
		for (int seq = 0; seq < 16; seq++)
		{
			//计算距离
			double hexToInt1 = TwoHextoInt(blockNext[seq * 4 + 0], blockNext[seq * 4 + 1]);
			nextDistance[seq] = hexToInt1 * m_calSimple;
		}
	}

	//误差计算
	double errorDistanceRatio = sin(intervalAngle * m_calRA) * 1;

	//比对mid中的点 和 prev、next中的点的距离，是否存在误差范围内相同距离的点
	for (int seq = 0; seq < 16; seq++)
	{
		double distance = midDistance[seq];
		bool bValidValue = false;
		//查找blockPrev
		for (int j = 0; j < 16; j++)
		{
			if (fabs(distance - prevDistance[j]) < distance * errorDistanceRatio)
			{
				bValidValue = true;
				break;
			}
		}
		//检查，有效点则直接继续
		if (bValidValue) continue;

		//查找blockNext
		for (int j = 0; j < 16; j++)
		{
			if (fabs(distance - nextDistance[j]) < distance * errorDistanceRatio)
			{
				bValidValue = true;
				break;
			}
		}
		//检查，无效点则清空距离值
		if (!bValidValue)
		{
			if (102 == protocol)
			{
				blockMid[seq * 8 + 0] = 0;
				blockMid[seq * 8 + 1] = 0;
			}
			else if (110 == protocol)
			{
				blockMid[seq * 4 + 0] = 0;
				blockMid[seq * 4 + 1] = 0;
			}
		}
	}
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, x)>::type setX(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, x)>::type setX(PointT& point, const float& value)
{
	point.x = value;
}


template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, y)>::type setY(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, y)>::type setY(PointT& point, const float& value)
{
	point.y = value;
}


template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, z)>::type setZ(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, z)>::type setZ(PointT& point, const float& value)
{
	point.z = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, intensity)>::type setIntensity(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, intensity)>::type setIntensity(PointT& point, const float& value)
{
	point.intensity = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, channel)>::type setChannel(PointT& point, const int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, channel)>::type setChannel(PointT& point, const int& value)
{
	point.channel = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, angle)>::type setAngle(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, angle)>::type setAngle(PointT& point, const float& value)
{
	point.angle = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, echo)>::type setEcho(PointT& point, const int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, echo)>::type setEcho(PointT& point, const int& value)
{
	point.echo = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, color)>::type setColor(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, color)>::type setColor(PointT& point, const float& value)
{
	point.color = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, t_sec)>::type setT_sec(PointT& point, const unsigned int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, t_sec)>::type setT_sec(PointT& point, const unsigned int& value)
{
	point.t_sec = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, t_usec)>::type setT_usec(PointT& point, const unsigned int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, t_usec)>::type setT_usec(PointT& point, const unsigned int& value)
{
	point.t_usec = value;
}

template <typename PointT>
void DecodePackage<PointT>::Start()
{
	run_decode.store(true);
	run_exit.store(false);
	m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
	std::thread(std::bind(&DecodePackage::BeginDecodePackageData, this)).detach();
}

template <typename PointT>
DecodePackage<PointT>::DecodePackage(std::shared_ptr<PackageCache> packageCachePtr, TWLidarType lidarType, std::mutex* mutex): 
	m_packageCachePtr(packageCachePtr), m_lidarType(lidarType), m_mutex(mutex)
{
	run_decode.store(false);
	run_exit.store(false);

	//Scope-192
	m_skewing_sin_scope[0] = sin(ScopeA_Elevation_A_Correct * m_calRA);
	m_skewing_sin_scope[1] = sin(ScopeB_Elevation_A_Correct * m_calRA);
	m_skewing_sin_scope[2] = sin(ScopeC_Elevation_A_Correct * m_calRA);

	m_skewing_cos_scope[0] = cos(ScopeA_Elevation_A_Correct* m_calRA);
	m_skewing_cos_scope[1] = cos(ScopeB_Elevation_A_Correct * m_calRA);
	m_skewing_cos_scope[2] = cos(ScopeC_Elevation_A_Correct * m_calRA);

	for (int i = 0; i < 64; i++)
	{
		double vA = m_verticalChannelAngle64[i];
		m_verticalChannelAngle64_cos_vA_RA[i] = cos(vA * m_calRA);
		m_verticalChannelAngle64_sin_vA_RA[i] = sin(vA * m_calRA);
	}

	//Scope-Mini-A2-192
	m_skewing_sin_scopeMiniA2_192[0] = sin(m_skewing_scopeMiniA2_Angle_Correct[0] * m_calRA);
	m_skewing_sin_scopeMiniA2_192[1] = sin(m_skewing_scopeMiniA2_Angle_Correct[1] * m_calRA);
	m_skewing_sin_scopeMiniA2_192[2] = sin(m_skewing_scopeMiniA2_Angle_Correct[2] * m_calRA);

	m_skewing_cos_scopeMiniA2_192[0] = cos(m_skewing_scopeMiniA2_Angle_Correct[0] * m_calRA);
	m_skewing_cos_scopeMiniA2_192[1] = cos(m_skewing_scopeMiniA2_Angle_Correct[1] * m_calRA);
	m_skewing_cos_scopeMiniA2_192[2] = cos(m_skewing_scopeMiniA2_Angle_Correct[2] * m_calRA);

	for (int i = 0; i < 64; i++)
	{
		double vA = m_verticalChannelAngle_Scope64_A2[i];
		m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[i] = cos(vA * m_calRA);
		m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[i] = sin(vA * m_calRA);
		m_channelValid[i] = true;
	}
	//
	//m_channelValidDistance = 4.0;
	//m_channelValid[32-1] = false;

	//TSP03-32
	m_skewing_sin_tsp[0] = sin(0.0 * m_calRA);
	m_skewing_sin_tsp[1] = sin(-6.0 * m_calRA);  //-6.0

	m_skewing_cos_tsp[0] = cos(0.0 * m_calRA);
	m_skewing_cos_tsp[1] = cos(-6.0 * m_calRA);

	for (int i = 0; i < 16; i++)
	{
		//计算
		double vA = m_verticalChannelAngle16[i];
		m_verticalChannelAngle16_cos_vA_RA[i] = cos(vA * m_calRA);
		m_verticalChannelAngle16_sin_vA_RA[i] = sin(vA * m_calRA);
	}

}

template <typename PointT>
DecodePackage<PointT>::~DecodePackage()
{
	//std::this_thread::sleep_for(std::chrono::seconds(1));
	run_decode.store(false);

	while (!run_exit)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

template <typename PointT>
void DecodePackage<PointT>::BeginDecodePackageData()
{
	run_exit.store(false);
	while (run_decode)
	{
		if (m_packageCachePtr->Size() <= 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		TWUDPPackage::Ptr packagePtr = m_packageCachePtr->PopFrontPackage();

		if (packagePtr->m_length == 120)
		{
			DecodeGPSData(packagePtr->m_szData);
			continue;
		}

		switch (m_lidarType)
		{
		case LT_TensorLite:
			if (packagePtr->m_length == 1440)
				DecodeTensorLite(packagePtr->m_szData, packagePtr->t_sec, packagePtr->t_usec);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		case LT_TensorPro:
			if (packagePtr->m_length == 1440)
				DecodeTensorPro(packagePtr->m_szData, packagePtr->t_sec, packagePtr->t_usec);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		case LT_TensorPro_echo2:
			if (packagePtr->m_length == 1440)
				DecodeTensorPro_echo2(packagePtr->m_szData, packagePtr->t_sec, packagePtr->t_usec);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		case LT_TSP0332:
			if (packagePtr->m_length == 1440)
				DecodeTensorPro0332(packagePtr->m_szData, packagePtr->t_sec, packagePtr->t_usec);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		case LT_Scope:
			if (packagePtr->m_length == 1120)
				DecodeScope(packagePtr->m_szData);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		case LT_Scope192:
			if (packagePtr->m_length == 1120)
				DecodeScope192(packagePtr->m_szData);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		case LT_ScopeMiniA2_192:
			if (packagePtr->m_length == 1120)
				DecodeScopeMiniA2_192(packagePtr->m_szData);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		default:
		{
			USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_INVALID_DEVICE, "Invalid device type!");
		}
			break;
		}

	}
	run_exit.store(true);
	USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_EXIT_DECODE, std::string("The decode package thread has exited!"));
}

template <typename PointT>
void DecodePackage<PointT>::UseDecodePointPro(int echo, double horAngle, int channel, float hexL, float hexPulseWidth, int offset, char* data, unsigned int t_sec, unsigned int t_usec)
{
	//distance
	double L = hexL * m_calSimple;
	if (L <= 0 || L > 300) return;

	double intensity = hexPulseWidth * m_calPulse;

	double cos_hA = cos(horAngle * m_calRA);
	double sin_hA = sin(horAngle * m_calRA);

	double vA = m_verticalChannelAngle16[channel-1];
	double cos_vA_RA = cos(vA * m_calRA);
	double x = L * cos_vA_RA * cos_hA;
	double y = L * cos_vA_RA * sin_hA;
	double z = L * sin(vA * m_calRA);

	PointT basic_point;
	setX(basic_point, static_cast<float>(x));
	setY(basic_point, static_cast<float>(y));
	setZ(basic_point, static_cast<float>(z));
	setIntensity(basic_point, static_cast<float>(intensity));
	setChannel(basic_point, channel);
	setAngle(basic_point, static_cast<float>(horAngle));
	setEcho(basic_point, echo);
	setT_sec(basic_point, t_sec);
	setT_usec(basic_point, t_usec);

	m_pointCloudPtr->PushBack(basic_point);

}

template <typename PointT>
void DecodePackage<PointT>::UseDecodePointTSP03_32(int echo, double horAngle, int channel, float hexL, float hexPulseWidth, int offset, char* data, unsigned int t_sec, unsigned int t_usec)
{
	//distance
	double L = hexL * m_calSimple;
	if (L <= 0 ) return;

	double intensity = hexPulseWidth * m_calPulse;

	double x = L * (m_verticalChannelAngle16_cos_vA_RA[channel - 1] * x_cal_1 + m_verticalChannelAngle16_sin_vA_RA[channel - 1] * x_cal_2);
	double y = L * (m_verticalChannelAngle16_cos_vA_RA[channel - 1] * y_cal_1 + m_verticalChannelAngle16_sin_vA_RA[channel - 1] * y_cal_2);
	double z = -L * (m_verticalChannelAngle16_cos_vA_RA[channel - 1] * z_cal_1 + m_verticalChannelAngle16_sin_vA_RA[channel - 1] * z_cal_2);

	PointT basic_point;
	setX(basic_point, static_cast<float>(x));
	setY(basic_point, static_cast<float>(y));
	setZ(basic_point, static_cast<float>(z));
	setIntensity(basic_point, static_cast<float>(intensity));
	setChannel(basic_point, channel);
	setAngle(basic_point, static_cast<float>(horAngle));
	setEcho(basic_point, echo);
	setT_sec(basic_point, t_sec);
	setT_usec(basic_point, t_usec);

	m_pointCloudPtr->PushBack(basic_point);
}

template <typename PointT>
void DecodePackage<PointT>::UseDecodePointScope(int echo, int sepIndex, int faceIndex, double horAngle, int channel, float hexL, float hexPulseWidth)
{
	if (sepIndex == 0)
		m_firstSeparateAngle = horAngle;
	else
	{
		if (m_firstSeparateAngle >= 0)
			horAngle = m_firstSeparateAngle;
	}

	//distance
	double L = hexL * m_calSimple;
	if (L <= 0 || L > 300) return;

	double intensity = hexPulseWidth * m_calPulse;

	double cos_hA = cos(horAngle * m_calRA);
	double sin_hA = sin(horAngle * m_calRA);

	double vA = m_verticalChannelAngle64[channel - 1];
	double cos_vA_RA = cos(vA * m_calRA);
	double x = L * cos_vA_RA * cos_hA;
	double y = L * cos_vA_RA * sin_hA;
	double z = L * sin(vA * m_calRA);


	PointT basic_point;
	setX(basic_point, static_cast<float>(x));
	setY(basic_point, static_cast<float>(y));
	setZ(basic_point, static_cast<float>(z));
	setIntensity(basic_point, static_cast<float>(intensity));
	setChannel(basic_point, channel);
	setAngle(basic_point, static_cast<float>(horAngle));
	setEcho(basic_point, echo);

	m_pointCloudPtr->PushBack(basic_point);
}

template <typename PointT>
void DecodePackage<PointT>::UseDecodePointScope_192(int echo, int sepIndex, int faceIndex, double horAngle, int channel, float hexL, float hexPulseWidth)
{
	if (sepIndex == 0)
		m_firstSeparateAngle = horAngle;
	else
	{
		if (m_firstSeparateAngle >= 0)
			horAngle = m_firstSeparateAngle;
	}

	//distance
	double L = hexL * m_calSimple;
	//分界距离后的叠加
	if (0 == faceIndex && L < (m_scopeABaddCSeparateDist - 0.05)) L = 0;
	if (1 == faceIndex && L < (m_scopeABaddCSeparateDist - 0.05)) L = 0;
	if (2 == faceIndex && L > (m_scopeABaddCSeparateDist + 0.05)) L = 0;
	if (L <= 0 || L > 300) return;

	double intensity = hexPulseWidth * m_calPulse;

	double x_tmp = L * (m_verticalChannelAngle64_cos_vA_RA[channel - 1] * x_cal_1 + m_verticalChannelAngle64_sin_vA_RA[channel - 1] * x_cal_2);
	double y_tmp = L * (m_verticalChannelAngle64_cos_vA_RA[channel - 1] * y_cal_1 + m_verticalChannelAngle64_sin_vA_RA[channel - 1] * y_cal_2);
	double z_tmp = -L * (m_verticalChannelAngle64_cos_vA_RA[channel - 1] * z_cal_1 + m_verticalChannelAngle64_sin_vA_RA[channel - 1] * z_cal_2);

	double x = x_tmp * m_rotate_scope_cos - y_tmp * m_rotate_scope_sin;
	double y = x_tmp * m_rotate_scope_sin + y_tmp * m_rotate_scope_cos;
	double z = z_tmp;

	if (channel <= m_corChannel)
	{
		if (z < (m_corHeight - m_corHeightDown))
		{
			z = m_corHeight - m_corHeightDown;
		}
		else if (z > (m_corHeight + m_corHeightUp) && z <= (m_corHeight + m_corHeightUp + m_corHeightRange))
		{
			z = m_corHeight + m_corHeightUp;
		}
	}

	PointT basic_point;
	setX(basic_point, static_cast<float>(x));
	setY(basic_point, static_cast<float>(y));
	setZ(basic_point, static_cast<float>(z));
	setIntensity(basic_point, static_cast<float>(intensity));
	setChannel(basic_point, channel);
	setAngle(basic_point, static_cast<float>(horAngle));
	setEcho(basic_point, echo);
	//setT_sec(basic_point, t_sec);
	//setT_usec(basic_point, t_usec);

	m_pointCloudPtr->PushBack(basic_point);
}


template <typename PointT>
void DecodePackage<PointT>::UseDecodePointScopeMiniA2_192(int echo, int sepIndex, int faceIndex, double horAngle, int channel, float hexL, float hexPulseWidth)
{
	if (sepIndex == 0)
		m_firstSeparateAngle = horAngle;
	else
	{
		if (m_firstSeparateAngle >= 0)
			horAngle = m_firstSeparateAngle;
	}

	//distance
	double L = hexL * m_calSimple;
	//分界距离后的叠加
	if (0 == faceIndex && L < (m_scopeABaddCSeparateDist - 0.05)) L = 0;
	if (1 == faceIndex && L < (m_scopeABaddCSeparateDist - 0.05)) L = 0;
	if (2 == faceIndex && L > (m_scopeABaddCSeparateDist + 0.05)) L = 0;
	if (L <= 0 || L > 300) return;

	double intensity = hexPulseWidth * m_calPulse;

	double x = L * (m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[channel - 1] * x_cal_1 + m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[channel - 1] * x_cal_2);
	double y = L * (m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[channel - 1] * y_cal_1 + m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[channel - 1] * y_cal_2);
	double z = -L * (m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[channel - 1] * z_cal_1 + m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[channel - 1] * z_cal_2);

	if (channel <= m_corChannel)
	{
		if (z < (m_corHeight - m_corHeightDown))
		{
			z = m_corHeight - m_corHeightDown;
		}
		else if (z > (m_corHeight + m_corHeightUp) && z <= (m_corHeight + m_corHeightUp + m_corHeightRange))
		{
			z = m_corHeight + m_corHeightUp;
		}
	}

	if (!(m_channelValid[channel-1]) && L <= m_channelValidDistance) return;

	PointT basic_point;
	setX(basic_point, static_cast<float>(x));
	setY(basic_point, static_cast<float>(y));
	setZ(basic_point, static_cast<float>(z));
	setIntensity(basic_point, static_cast<float>(intensity));
	setChannel(basic_point, channel);
	setAngle(basic_point, static_cast<float>(horAngle));
	setEcho(basic_point, echo);
	//setT_sec(basic_point, t_sec);
	//setT_usec(basic_point, t_usec);

	m_pointCloudPtr->PushBack(basic_point);
}

template <typename PointT>
void DecodePackage<PointT>::DecodeGPSData(char* udpData)
{
	//gps string
	std::string gps_value = "";
	//status valid(0x41);  invalid(0x56)
	std::string gps_status = "GPS STATUS: (Unknown)";
	bool bUnknown = false;
	if (udpData[3] == 0x41)
		gps_status = "GPS STATUS: (Valid)";
	else if (udpData[3] == 0x56)
		gps_status = "GPS STATUS: (Invalid)";
	else
	{
		gps_status = "GPS STATUS: (Unknown)";
		bUnknown = true;
	}

	//GPS time info
	//                   0    1    2    3    4    5    6   7    8    9    10   11   12   13   14   15   16   17   18    19
	char sz_gps[20] = { '2', '0', ' ', ' ', '-', ' ', ' ', '-', ' ', ' ', ' ', ' ', ' ', ':', ' ', ' ', ':', ' ', ' ', '\0' };

	sz_gps[2] = bUnknown ? '#' : udpData[4]; //year
	sz_gps[3] = bUnknown ? '#' : udpData[5];
	sz_gps[5] = bUnknown ? '#' : udpData[6]; //month
	sz_gps[6] = bUnknown ? '#' : udpData[7];
	sz_gps[8] = bUnknown ? '#' : udpData[8]; //day
	sz_gps[9] = bUnknown ? '#' : udpData[9];

	sz_gps[11] = bUnknown ? '#' : udpData[10]; //hour
	sz_gps[12] = bUnknown ? '#' : udpData[11];
	sz_gps[14] = bUnknown ? '#' : udpData[12]; //minute
	sz_gps[15] = bUnknown ? '#' : udpData[13];
	sz_gps[17] = bUnknown ? '#' : udpData[14]; //second
	sz_gps[18] = bUnknown ? '#' : udpData[15];

	//GPS time
	//unsigned int time_us = FourHexToInt(udpData[16], udpData[17], udpData[18], udpData[19]);

	if (bUnknown)
		gps_value = gps_status;
	else
		gps_value = gps_status + std::string("  ") + std::string(sz_gps);

	std::lock_guard<std::mutex> lock(*m_mutex);
	if (m_funcGPS) m_funcGPS(gps_value);
}

template <typename PointT>
void DecodePackage<PointT>::DecodeTensorLite(char* udpData, unsigned int t_sec, unsigned int t_usec)
{
	DecodeTensorPro(udpData, t_sec, t_usec);
}

template <typename PointT>
void DecodePackage<PointT>::DecodeTensorPro(char* udpDataOri, unsigned int t_sec, unsigned int t_usec)
{
	char udpData[UDP_MAX_LENGTH] = { 0 };
	int length = UDP_MAX_LENGTH;
	memcpy(udpData, udpDataOri, length);
	if (true)
	{
		if (m_cashUDPLength <= 0 || m_cashUDPLength != length)
		{
			memcpy(m_cashUDPData, udpDataOri, length);
			m_cashUDPLength = length;
			return;
		}
		else
		{
			//提取缓存UDP
			char prevUDPData[UDP_MAX_LENGTH] = { 0 };
			memcpy(prevUDPData, m_cashUDPData, length);
			//写入新缓存UDP
			memcpy(m_cashUDPData, udpDataOri, length);

			//计算上一个缓存UDP的 第20列
			FilterDisturbPoint(&(prevUDPData[18 * 72]), &(m_cashUDPData[0 * 72]), &(prevUDPData[19 * 72]), 110, 5.5);
			//计算当前UDP的 第1-19列
			FilterDisturbPoint(&(prevUDPData[19 * 72]), &(m_cashUDPData[1 * 72]), &(m_cashUDPData[0 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[0 * 72]), &(m_cashUDPData[2 * 72]), &(m_cashUDPData[1 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[1 * 72]), &(m_cashUDPData[3 * 72]), &(m_cashUDPData[2 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[2 * 72]), &(m_cashUDPData[4 * 72]), &(m_cashUDPData[3 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[3 * 72]), &(m_cashUDPData[5 * 72]), &(m_cashUDPData[4 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[4 * 72]), &(m_cashUDPData[6 * 72]), &(m_cashUDPData[5 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[5 * 72]), &(m_cashUDPData[7 * 72]), &(m_cashUDPData[6 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[6 * 72]), &(m_cashUDPData[8 * 72]), &(m_cashUDPData[7 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[7 * 72]), &(m_cashUDPData[9 * 72]), &(m_cashUDPData[8 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[8 * 72]), &(m_cashUDPData[10 * 72]), &(m_cashUDPData[9 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[9 * 72]), &(m_cashUDPData[11 * 72]), &(m_cashUDPData[10 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[10 * 72]), &(m_cashUDPData[12 * 72]), &(m_cashUDPData[11 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[11 * 72]), &(m_cashUDPData[13 * 72]), &(m_cashUDPData[12 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[12 * 72]), &(m_cashUDPData[14 * 72]), &(m_cashUDPData[13 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[13 * 72]), &(m_cashUDPData[15 * 72]), &(m_cashUDPData[14 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[14 * 72]), &(m_cashUDPData[16 * 72]), &(m_cashUDPData[15 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[15 * 72]), &(m_cashUDPData[17 * 72]), &(m_cashUDPData[16 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[16 * 72]), &(m_cashUDPData[18 * 72]), &(m_cashUDPData[17 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[17 * 72]), &(m_cashUDPData[19 * 72]), &(m_cashUDPData[18 * 72]), 110, 5.5);


			//提取UDP数据用于计算
			memcpy(udpData, prevUDPData, length);
		}
	}


	for (int blocks_num = 0; blocks_num < 20; blocks_num++)
	{
		int offset = blocks_num * 72;

		//horizontal angle
		int hextoAngle = FourHexToInt(udpData[offset + 64], udpData[offset + 65], udpData[offset + 66], udpData[offset + 67]);
		double horizontalAngle = hextoAngle * 0.00001;

		if (horizontalAngle < m_startAngle && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();
			m_pointCloudPtr->stamp = (uint64_t)t_sec * 1000 * 1000 + t_usec;

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(360);
			continue;
		}
		if (horizontalAngle <m_startAngle || horizontalAngle > m_endAngle) continue;

		int seq = 0;
		while (seq < 16)
		{
			//distance
			float hexL = TwoHextoInt(udpData[offset + seq * 4], udpData[offset + seq * 4 + 1])*1.0f;
			//
			float hexPlusWidth = TwoHextoInt(udpData[offset + seq * 4 + 2], udpData[offset + seq * 4 + 3])*1.0f;

			//channel 1-16
			int channel = seq + 1;

			//time of point
			unsigned int cur_sec = t_sec;
			unsigned int cur_usec = t_usec;
			if (t_usec < blocks_num * 29.4) //The time interval between the two columns is 29.4 subtle
			{
				cur_sec = cur_usec - 1;
				cur_usec = (unsigned int)(t_usec + 1000000 - blocks_num * 29.4);
			}
			else
			{
				cur_usec = (unsigned int)(t_usec - blocks_num * 29.4);
			}

			//using
			UseDecodePointPro(1, horizontalAngle, channel, hexL, hexPlusWidth, offset, udpData, cur_sec, cur_usec);

			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeTensorPro_echo2(char* udpData, unsigned int t_sec, unsigned int t_usec)
{
	for (int blocks_num = 0; blocks_num < 20; blocks_num++)
	{
		int offset = blocks_num * 72;

		//horizontal angle
		int hextoAngle = FourHexToInt(udpData[offset + 64], udpData[offset + 65], udpData[offset + 66], udpData[offset + 67]);
		double horizontalAngle = hextoAngle * 0.00001;

		if (horizontalAngle < m_startAngle && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(360);
			continue;
		}
		if (horizontalAngle <m_startAngle || horizontalAngle > m_endAngle) continue;

		int seq = 0;
		while (seq < 16)
		{
			//distance
			float hexL = TwoHextoInt(udpData[offset + seq * 4], udpData[offset + seq * 4 + 1])*1.0f;
			//
			float hexPlusWidth = TwoHextoInt(udpData[offset + seq * 4 + 2], udpData[offset + seq * 4 + 3])*1.0f;

			//channel: 1-16
			int channel = seq + 1;

			//using
			UseDecodePointPro(blocks_num%2 + 1, horizontalAngle, channel, hexL, hexPlusWidth, offset, udpData, t_sec, t_usec);

			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeTensorPro0332(char* udpDataOri, unsigned int t_sec, unsigned int t_usec)
{
	char udpData[UDP_MAX_LENGTH] = { 0 };
	int length = UDP_MAX_LENGTH;
	memcpy(udpData, udpDataOri, length);
	if (true)
	{
		if (m_cashUDPLength <= 0 || m_cashUDPLength != length)
		{
			memcpy(m_cashUDPData, udpDataOri, length);
			m_cashUDPLength = length;
			return;
		}
		else
		{
			//提取缓存UDP
			char prevUDPData[UDP_MAX_LENGTH] = { 0 };
			memcpy(prevUDPData, m_cashUDPData, length);
			//写入新缓存UDP
			memcpy(m_cashUDPData, udpDataOri, length);

			//计算上一个缓存UDP的 第20列
			FilterDisturbPoint(&(prevUDPData[18 * 72]), &(m_cashUDPData[0 * 72]), &(prevUDPData[19 * 72]), 110, 5.5);
			//计算当前UDP的 第1-19列
			FilterDisturbPoint(&(prevUDPData[19 * 72]), &(m_cashUDPData[1 * 72]), &(m_cashUDPData[0 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[0 * 72]), &(m_cashUDPData[2 * 72]), &(m_cashUDPData[1 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[1 * 72]), &(m_cashUDPData[3 * 72]), &(m_cashUDPData[2 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[2 * 72]), &(m_cashUDPData[4 * 72]), &(m_cashUDPData[3 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[3 * 72]), &(m_cashUDPData[5 * 72]), &(m_cashUDPData[4 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[4 * 72]), &(m_cashUDPData[6 * 72]), &(m_cashUDPData[5 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[5 * 72]), &(m_cashUDPData[7 * 72]), &(m_cashUDPData[6 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[6 * 72]), &(m_cashUDPData[8 * 72]), &(m_cashUDPData[7 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[7 * 72]), &(m_cashUDPData[9 * 72]), &(m_cashUDPData[8 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[8 * 72]), &(m_cashUDPData[10 * 72]), &(m_cashUDPData[9 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[9 * 72]), &(m_cashUDPData[11 * 72]), &(m_cashUDPData[10 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[10 * 72]), &(m_cashUDPData[12 * 72]), &(m_cashUDPData[11 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[11 * 72]), &(m_cashUDPData[13 * 72]), &(m_cashUDPData[12 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[12 * 72]), &(m_cashUDPData[14 * 72]), &(m_cashUDPData[13 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[13 * 72]), &(m_cashUDPData[15 * 72]), &(m_cashUDPData[14 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[14 * 72]), &(m_cashUDPData[16 * 72]), &(m_cashUDPData[15 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[15 * 72]), &(m_cashUDPData[17 * 72]), &(m_cashUDPData[16 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[16 * 72]), &(m_cashUDPData[18 * 72]), &(m_cashUDPData[17 * 72]), 110, 5.5);
			FilterDisturbPoint(&(m_cashUDPData[17 * 72]), &(m_cashUDPData[19 * 72]), &(m_cashUDPData[18 * 72]), 110, 5.5);


			//提取UDP数据用于计算
			memcpy(udpData, prevUDPData, length);
		}
	}

	for (int blocks_num = 0; blocks_num < 20; blocks_num++)
	{
		int offset = blocks_num * 72;

		//horizontal angle
		int hextoAngle = FourHexToInt(udpData[offset + 64], udpData[offset + 65], udpData[offset + 66], udpData[offset + 67]);
		double horizontalAngle = hextoAngle * 0.00001;
		//face index
		unsigned char  hexMirror = udpData[offset + 68];
		hexMirror = hexMirror >> 7;
		unsigned short faceIndex = hexMirror;

		if (horizontalAngle < m_startAngle && 0 == faceIndex && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();
			m_pointCloudPtr->stamp = (uint64_t)t_sec * 1000 * 1000 + t_usec;

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(360);
			continue;
		}
		if (horizontalAngle < m_startAngle || horizontalAngle > m_endAngle) continue;

		//
		double hA = 0.5 * horizontalAngle * m_calRA;
		double hA_sin = sin(hA);
		double hA_cos = cos(hA);

		x_cal_1 = 2.0 * m_skewing_cos_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_cos*hA_cos - 1;
		x_cal_2 = 2.0 * m_skewing_sin_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_cos;

		y_cal_1 = 2.0 * m_skewing_cos_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_sin * hA_cos;
		y_cal_2 = 2.0 * m_skewing_sin_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_sin;

		z_cal_1 = 2.0 * m_skewing_sin_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_cos;
		z_cal_2 = 2.0 * m_skewing_sin_tsp[faceIndex] * m_skewing_sin_tsp[faceIndex] - 1;


		int seq = 0;
		while (seq < 16)
		{
			//distance
			float hexL = TwoHextoInt(udpData[offset + seq * 4], udpData[offset + seq * 4 + 1])*1.0f;
			//
			float hexPlusWidth = TwoHextoInt(udpData[offset + seq * 4 + 2], udpData[offset + seq * 4 + 3])*1.0f;

			//channel 1-16
			int channel = seq + 1;

			//time of point
			unsigned int cur_sec = t_sec;
			unsigned int cur_usec = t_usec;
			if (t_usec < blocks_num * 29.4) //The time interval between the two columns is 29.4 subtle
			{
				cur_sec = cur_usec - 1;
				cur_usec = (unsigned int)(t_usec + 1000000 - blocks_num * 29.4);
			}
			else
			{
				cur_usec = (unsigned int)(t_usec - blocks_num * 29.4);
			}

			//using
			UseDecodePointTSP03_32(1, horizontalAngle, channel, hexL, hexPlusWidth, offset, udpData, cur_sec, cur_usec);

			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeScope(char* udpDataOri)
{
	char udpData[UDP_MAX_LENGTH] = { 0 };
	int length = UDP_MAX_LENGTH;
	memcpy(udpData, udpDataOri, length);
	if (true)
	{
		if (m_cashUDPLength <= 0 || m_cashUDPLength != length)
		{
			memcpy(m_cashUDPData, udpDataOri, length);
			m_cashUDPLength = length;
			return;
		}
		else
		{
			//提取缓存UDP
			char prevUDPData[UDP_MAX_LENGTH] = { 0 };
			memcpy(prevUDPData, m_cashUDPData, length);
			//写入新缓存UDP
			memcpy(m_cashUDPData, udpDataOri, length);


			//计算上一个缓存UDP后一列
			FilterDisturbPoint(&(prevUDPData[0 * 140]), &(m_cashUDPData[0 * 140]), &(prevUDPData[4 * 140]), 102, 7.5);
			FilterDisturbPoint(&(prevUDPData[1 * 140]), &(m_cashUDPData[1 * 140]), &(prevUDPData[5 * 140]), 102, 7.5);
			FilterDisturbPoint(&(prevUDPData[2 * 140]), &(m_cashUDPData[2 * 140]), &(prevUDPData[6 * 140]), 102, 7.5);
			FilterDisturbPoint(&(prevUDPData[3 * 140]), &(m_cashUDPData[3 * 140]), &(prevUDPData[7 * 140]), 102, 7.5);
			//计算当前UDP的前一列
			FilterDisturbPoint(&(prevUDPData[4 * 140]), &(m_cashUDPData[4 * 140]), &(m_cashUDPData[0 * 140]), 102, 7.5);
			FilterDisturbPoint(&(prevUDPData[5 * 140]), &(m_cashUDPData[5 * 140]), &(m_cashUDPData[1 * 140]), 102, 7.5);
			FilterDisturbPoint(&(prevUDPData[6 * 140]), &(m_cashUDPData[6 * 140]), &(m_cashUDPData[2 * 140]), 102, 7.5);
			FilterDisturbPoint(&(prevUDPData[7 * 140]), &(m_cashUDPData[7 * 140]), &(m_cashUDPData[3 * 140]), 102, 7.5);

			//提取UDP数据用于计算
			memcpy(udpData, prevUDPData, length);
		}
	}

	for (int blocks_num = 0; blocks_num < 8; blocks_num++)
	{
		int offset = blocks_num * 140;

		//horizontal angle index: 128-131
		int hextoAngle = FourHexToInt(udpData[offset + 128], udpData[offset + 129], udpData[offset + 130], udpData[offset + 131]);
		double horizontalAngle = hextoAngle * 0.00001;

		if (horizontalAngle < m_startAngle && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(360);
			continue;
		}
		if (horizontalAngle <m_startAngle || horizontalAngle > m_endAngle) continue;


		//separate index
		unsigned char  hexSepIndex = udpData[offset + 136];
		unsigned short sepIndex = hexSepIndex >> 6;
		//face id
		unsigned char  hexFaceIndex = udpData[offset + 136];
		hexFaceIndex = hexFaceIndex << 2;
		unsigned short faceIndex = hexFaceIndex >> 6;

		int seq = 0;
		while (seq < 16)
		{
			//echo 1
			float hexL1 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 0], udpData[offset + seq * 8 + 1]));
			float hexPulseWidth1 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 2], udpData[offset + seq * 8 + 3]));

			//echo 2
			float hexL2 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 4], udpData[offset + seq * 8 + 5]));
			float hexPulseWidth2 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 6], udpData[offset + seq * 8 + 7]));

			//channel
			int channel = 65 - (16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1);

			//using
			UseDecodePointScope(1, sepIndex, faceIndex, horizontalAngle, channel, hexL1, hexPulseWidth1);
			//UseDecodePointScope(2, sepIndex, faceIndex, horizontalAngle, channel, hexL2, hexPulseWidth2);

			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeScope192(char* udpDataOri)
{
	char udpData[UDP_MAX_LENGTH] = { 0 };
	int length = UDP_MAX_LENGTH;
	memcpy(udpData, udpDataOri, length);
	if (true)
	{
		if (m_cashUDPLength <= 0 || m_cashUDPLength != length)
		{
			memcpy(m_cashUDPData, udpDataOri, length);
			m_cashUDPLength = length;
			return;
		}
		else
		{
			//提取缓存UDP
			char prevUDPData[UDP_MAX_LENGTH] = { 0 };
			memcpy(prevUDPData, m_cashUDPData, length);
			//写入新缓存UDP
			memcpy(m_cashUDPData, udpDataOri, length);


			//计算上一个缓存UDP后一列
			FilterDisturbPoint(&(prevUDPData[0 * 140]), &(m_cashUDPData[0 * 140]), &(prevUDPData[4 * 140]), 102, 7.5);
			FilterDisturbPoint(&(prevUDPData[1 * 140]), &(m_cashUDPData[1 * 140]), &(prevUDPData[5 * 140]), 102, 7.5);
			FilterDisturbPoint(&(prevUDPData[2 * 140]), &(m_cashUDPData[2 * 140]), &(prevUDPData[6 * 140]), 102, 7.5);
			FilterDisturbPoint(&(prevUDPData[3 * 140]), &(m_cashUDPData[3 * 140]), &(prevUDPData[7 * 140]), 102, 7.5);
			//计算当前UDP的前一列
			FilterDisturbPoint(&(prevUDPData[4 * 140]), &(m_cashUDPData[4 * 140]), &(m_cashUDPData[0 * 140]), 102, 7.5);
			FilterDisturbPoint(&(prevUDPData[5 * 140]), &(m_cashUDPData[5 * 140]), &(m_cashUDPData[1 * 140]), 102, 7.5);
			FilterDisturbPoint(&(prevUDPData[6 * 140]), &(m_cashUDPData[6 * 140]), &(m_cashUDPData[2 * 140]), 102, 7.5);
			FilterDisturbPoint(&(prevUDPData[7 * 140]), &(m_cashUDPData[7 * 140]), &(m_cashUDPData[3 * 140]), 102, 7.5);

			//提取UDP数据用于计算
			memcpy(udpData, prevUDPData, length);
		}
	}

	double horizontalAngle = 0;
	//face id
	unsigned short faceIndex = 0;

	for (int blocks_num = 0; blocks_num < 8; blocks_num++)
	{
		int offset = blocks_num * 140;
		if (0 == blocks_num || 4 == blocks_num)
		{
			//horizontal angle index: 128-131
			int HextoAngle = FourHexToInt(udpData[offset + 128], udpData[offset + 129], udpData[offset + 130], udpData[offset + 131]);
			horizontalAngle = HextoAngle  * 0.00001;

			unsigned char  hexMirror = udpData[offset + 136];
			hexMirror = hexMirror << 2;
			faceIndex = hexMirror >> 6;

			//提前UDP中的三个面的偏移角度
			float offsetAngle = 0;
			unsigned char  hexACount = udpData[offset + 136];
			hexACount = hexACount << 4;
			unsigned short uACount = hexACount >> 4;
			offsetAngle = uACount * 0.02 - 0.15;
			//按镜面修改组修正角度值
			//重新计算ABC面俯仰值 fabs(value1 - value2) < 0.001
			if (0 == faceIndex && fabs(ScopeA_Elevation_A_Correct - (ScopeA_Elevation_A + offsetAngle)) > 0.001)
			{
				ScopeA_Elevation_A_Correct = ScopeA_Elevation_A + offsetAngle;
				m_skewing_sin_scope[0] = sin(ScopeA_Elevation_A_Correct * m_calRA);
				m_skewing_cos_scope[0] = cos(ScopeA_Elevation_A_Correct * m_calRA);
				//std::cout << "A: " << ScopeA_Elevation_A_Correct << std::endl;
			}
			else if (1 == faceIndex && fabs(ScopeB_Elevation_A_Correct - (ScopeB_Elevation_A + offsetAngle)) > 0.001)
			{
				ScopeB_Elevation_A_Correct = ScopeB_Elevation_A + offsetAngle;
				m_skewing_sin_scope[1] = sin(ScopeB_Elevation_A * m_calRA);
				m_skewing_cos_scope[1] = cos(ScopeB_Elevation_A * m_calRA);
				//std::cout << "B: " << ScopeB_Elevation_A_Correct << std::endl;
			}
			else if (2 == faceIndex && fabs(ScopeC_Elevation_A_Correct - (ScopeC_Elevation_A + offsetAngle)) > 0.001)
			{
				ScopeC_Elevation_A_Correct = ScopeC_Elevation_A + offsetAngle;
				m_skewing_sin_scope[2] = sin(ScopeC_Elevation_A * m_calRA);
				m_skewing_cos_scope[2] = cos(ScopeC_Elevation_A * m_calRA);
				//std::cout << "C: " << ScopeC_Elevation_A_Correct << std::endl;
			}


			double hA = 0.5 * (horizontalAngle + 10.0) * m_calRA;
			double hA_sin = sin(hA);
			double hA_cos = cos(hA);

			x_cal_1 = 2.0 * m_skewing_cos_scope[faceIndex] * m_skewing_cos_scope[faceIndex] * hA_cos*hA_cos - 1;
			x_cal_2 = 2.0 * m_skewing_sin_scope[faceIndex] * m_skewing_cos_scope[faceIndex] * hA_cos;

			y_cal_1 = 2.0 * m_skewing_cos_scope[faceIndex] * m_skewing_cos_scope[faceIndex] * hA_sin * hA_cos;
			y_cal_2 = 2.0 * m_skewing_sin_scope[faceIndex] * m_skewing_cos_scope[faceIndex] * hA_sin;

			z_cal_1 = 2.0 * m_skewing_sin_scope[faceIndex] * m_skewing_cos_scope[faceIndex] * hA_cos;
			z_cal_2 = 2.0 * m_skewing_sin_scope[faceIndex] * m_skewing_sin_scope[faceIndex] - 1;
		}

		if (horizontalAngle < m_startAngle && 0 == faceIndex && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(360);
			continue;
		}
		if (horizontalAngle <m_startAngle || horizontalAngle > m_endAngle) continue;


		//separate index
		unsigned char  hexSepIndex = udpData[offset + 136];
		unsigned short sepIndex = hexSepIndex >> 6;

		int seq = 0;
		while (seq < 16)
		{
			//echo 1
			float hexL1 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 0], udpData[offset + seq * 8 + 1]));
			float hexPulseWidth1 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 2], udpData[offset + seq * 8 + 3]));

			//echo 2
			float hexL2 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 4], udpData[offset + seq * 8 + 5]));
			float hexPulseWidth2 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 6], udpData[offset + seq * 8 + 7]));

			//channel
			int channel = 65 - (16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1);

			//calculate
			if (hexL1 * m_calSimple <= 10.0)
			{
				if ((blocks_num == 0 || blocks_num == 4) && seq == 0)
				{
					//next channel
					int hexL_Next = TwoHextoInt(udpData[offset + (seq + 1) * 8 + 0], udpData[offset + (seq + 1) * 8 + 1]);
					if (abs(hexL_Next - hexL1) > 10000) hexL1 = 0;
				}
				else if ((blocks_num == 3 || blocks_num == 7) && seq == 15)
				{
					//previous channel
					int hexL_Prev = TwoHextoInt(udpData[offset + (seq - 1) * 8 + 0], udpData[offset + (seq - 1) * 8 + 1]);
					if (abs(hexL_Prev - hexL1) > 10000) hexL1 = 0;
				}
				else
				{
					//previous channel
					int hexL_Prev = 0;
					//next channel
					int hexL_Next = 0;

					if (seq == 0)
						hexL_Prev = TwoHextoInt(udpData[offset - 140 + 15 * 8 + 0], udpData[offset - 140 + 15 * 8 + 1]);
					else
						hexL_Prev = TwoHextoInt(udpData[offset + (seq - 1) * 8 + 0], udpData[offset + (seq - 1) * 8 + 1]);

					if (seq == 15)
						hexL_Next = TwoHextoInt(udpData[offset + 140 + 0 * 8 + 0], udpData[offset + 140 + 0 * 8 + 1]);
					else
						hexL_Next = TwoHextoInt(udpData[offset + (seq + 1) * 8 + 0], udpData[offset + (seq + 1) * 8 + 1]);


					if ((abs(hexL_Prev - hexL1) > 10000 || hexL_Prev <= 0) && (abs(hexL_Next - hexL1) > 10000 || hexL_Next <= 0)) hexL1 = 0;
				}
			}

			//using
			UseDecodePointScope_192(1, sepIndex, faceIndex, horizontalAngle, channel, hexL1, hexPulseWidth1);
			//UseDecodePointScope_192(2, sepIndex, faceIndex, horizontalAngle, channel, hexL2, hexPulseWidth2);

			seq++;
		}
	}
}


template <typename PointT>
void DecodePackage<PointT>::DecodeScopeMiniA2_192(char* udpDataOri)
{
	char udpData[UDP_MAX_LENGTH] = { 0 };
	int length = UDP_MAX_LENGTH;
	memcpy(udpData, udpDataOri, length);
	if (true)
	{
		if (m_cashUDPLength <= 0 || m_cashUDPLength != length)
		{
			memcpy(m_cashUDPData, udpDataOri, length);
			m_cashUDPLength = length;
			return;
		}
		else
		{
			//提取缓存UDP
			char prevUDPData[UDP_MAX_LENGTH] = { 0 };
			memcpy(prevUDPData, m_cashUDPData, length);
			//写入新缓存UDP
			memcpy(m_cashUDPData, udpDataOri, length);


			//计算上一个缓存UDP后一列
			FilterDisturbPoint(&(prevUDPData[0 * 140]), &(m_cashUDPData[0 * 140]), &(prevUDPData[4 * 140]), 102, 6.35);
			FilterDisturbPoint(&(prevUDPData[1 * 140]), &(m_cashUDPData[1 * 140]), &(prevUDPData[5 * 140]), 102, 6.35);
			FilterDisturbPoint(&(prevUDPData[2 * 140]), &(m_cashUDPData[2 * 140]), &(prevUDPData[6 * 140]), 102, 6.35);
			FilterDisturbPoint(&(prevUDPData[3 * 140]), &(m_cashUDPData[3 * 140]), &(prevUDPData[7 * 140]), 102, 6.35);
			//计算当前UDP的前一列
			FilterDisturbPoint(&(prevUDPData[4 * 140]), &(m_cashUDPData[4 * 140]), &(m_cashUDPData[0 * 140]), 102, 6.35);
			FilterDisturbPoint(&(prevUDPData[5 * 140]), &(m_cashUDPData[5 * 140]), &(m_cashUDPData[1 * 140]), 102, 6.35);
			FilterDisturbPoint(&(prevUDPData[6 * 140]), &(m_cashUDPData[6 * 140]), &(m_cashUDPData[2 * 140]), 102, 6.35);
			FilterDisturbPoint(&(prevUDPData[7 * 140]), &(m_cashUDPData[7 * 140]), &(m_cashUDPData[3 * 140]), 102, 6.35);
			//提取UDP数据用于计算
			memcpy(udpData, prevUDPData, length);
		}
	}

	double horizontalAngle = 0;
	//face id
	unsigned short faceIndex = 0;

	for (int blocks_num = 0; blocks_num < 8; blocks_num++)
	{
		int offset = blocks_num * 140;
		if (0 == blocks_num || 4 == blocks_num)
		{
			//horizontal angle index: 128-131
			int HextoAngle = FourHexToInt(udpData[offset + 128], udpData[offset + 129], udpData[offset + 130], udpData[offset + 131]);
			horizontalAngle = HextoAngle  * 0.00001;

			unsigned char  hexMirror = udpData[offset + 136];
			hexMirror = hexMirror << 2;
			faceIndex = hexMirror >> 6;

			//offset angle m_skewing_scopeMiniA2_angle 
			double offsetAngle = 0;
			unsigned char  hexACount = udpData[offset + 136];
			hexACount = hexACount << 4;
			unsigned short uACount = hexACount >> 4;
			offsetAngle = uACount * 0.04 - 0.3;

			//calculate 
			if (faceIndex < 3 && fabs(m_skewing_scopeMiniA2_Angle_Correct[faceIndex] - (m_skewing_scopeMiniA2_Angle[faceIndex] + offsetAngle)) > 0.001)
			{
				m_skewing_scopeMiniA2_Angle_Correct[faceIndex] = m_skewing_scopeMiniA2_Angle[faceIndex] + offsetAngle;
				m_skewing_sin_scopeMiniA2_192[faceIndex] = sin(m_skewing_scopeMiniA2_Angle_Correct[faceIndex] * m_calRA);
				m_skewing_cos_scopeMiniA2_192[faceIndex] = cos(m_skewing_scopeMiniA2_Angle_Correct[faceIndex] * m_calRA);
			}

			double hA = 0.5 * (horizontalAngle ) * m_calRA;
			double hA_sin = sin(hA);
			double hA_cos = cos(hA);

			x_cal_1 = 2.0 * m_skewing_cos_scopeMiniA2_192[faceIndex] * m_skewing_cos_scopeMiniA2_192[faceIndex] * hA_cos*hA_cos - 1;
			x_cal_2 = 2.0 * m_skewing_sin_scopeMiniA2_192[faceIndex] * m_skewing_cos_scopeMiniA2_192[faceIndex] * hA_cos;

			y_cal_1 = 2.0 * m_skewing_cos_scopeMiniA2_192[faceIndex] * m_skewing_cos_scopeMiniA2_192[faceIndex] * hA_sin * hA_cos;
			y_cal_2 = 2.0 * m_skewing_sin_scopeMiniA2_192[faceIndex] * m_skewing_cos_scopeMiniA2_192[faceIndex] * hA_sin;

			z_cal_1 = 2.0 * m_skewing_sin_scopeMiniA2_192[faceIndex] * m_skewing_cos_scopeMiniA2_192[faceIndex] * hA_cos;
			z_cal_2 = 2.0 * m_skewing_sin_scopeMiniA2_192[faceIndex] * m_skewing_sin_scopeMiniA2_192[faceIndex] - 1;
		}

		if (horizontalAngle < m_startAngle && 0 == faceIndex && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(360);
			continue;
		}
		if (horizontalAngle <m_startAngle || horizontalAngle > m_endAngle) continue;


		//separate index
		unsigned char  hexSepIndex = udpData[offset + 136];
		unsigned short sepIndex = hexSepIndex >> 6;

		int seq = 0;
		while (seq < 16)
		{
			//echo 1
			float hexL1 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 0], udpData[offset + seq * 8 + 1]));
			float hexPulseWidth1 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 2], udpData[offset + seq * 8 + 3]));

			//echo 2
			float hexL2 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 4], udpData[offset + seq * 8 + 5]));
			float hexPulseWidth2 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 6], udpData[offset + seq * 8 + 7]));

			//channel
			int channel = 65 - (16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1);

			//calculate
			if (hexL1 * m_calSimple <= 10.0)
			{
				if ((blocks_num == 0 || blocks_num == 4) && seq == 0)
				{
					//next channel
					int hexL_Next = TwoHextoInt(udpData[offset + (seq + 1) * 8 + 0], udpData[offset + (seq + 1) * 8 + 1]);
					if (abs(hexL_Next - hexL1) > 10000) hexL1 = 0;
				}
				else if ((blocks_num == 3 || blocks_num == 7) && seq == 15)
				{
					//previous channel
					int hexL_Prev = TwoHextoInt(udpData[offset + (seq - 1) * 8 + 0], udpData[offset + (seq - 1) * 8 + 1]);
					if (abs(hexL_Prev - hexL1) > 10000) hexL1 = 0;
				}
				else
				{
					//previous channel
					int hexL_Prev = 0;
					//next channel
					int hexL_Next = 0;

					if (seq == 0)
						hexL_Prev = TwoHextoInt(udpData[offset - 140 + 15 * 8 + 0], udpData[offset - 140 + 15 * 8 + 1]);
					else
						hexL_Prev = TwoHextoInt(udpData[offset + (seq - 1) * 8 + 0], udpData[offset + (seq - 1) * 8 + 1]);

					if (seq == 15)
						hexL_Next = TwoHextoInt(udpData[offset + 140 + 0 * 8 + 0], udpData[offset + 140 + 0 * 8 + 1]);
					else
						hexL_Next = TwoHextoInt(udpData[offset + (seq + 1) * 8 + 0], udpData[offset + (seq + 1) * 8 + 1]);


					if ((abs(hexL_Prev - hexL1) > 10000 || hexL_Prev <= 0) && (abs(hexL_Next - hexL1) > 10000 || hexL_Next <= 0)) hexL1 = 0;
				}
			}

			//using
			UseDecodePointScopeMiniA2_192(1, sepIndex, faceIndex, horizontalAngle, channel, hexL1, hexPulseWidth1);

			seq++;
		}
	}
}
