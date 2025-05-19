/** 
 * @file
 * @brief	类 RobotModel 的实现
 * @author	罗灿威，王雷
 * @date		2013/12/23
 * @version	2.0
 * - 00a, 2015/08/10, wl : 修改本文件
 */
#include "RobotModel/RobotModel.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include<Eigen/Eigen>
using namespace Eigen;
#if (defined VXWORKS)
 #include <logLib.h>
#endif /* VXWORKS */
#include "calibration/calibration.h"

unsigned char jointLimitNum = 0;
ErrorID rectLimit = RANGE_NOOVERLIMIT;
ErrorID speedLimit = SPEED_NOOVERLIMIT;
bool  optimalSolution = false;

bool doubleHighPrecisionCompareEqu(double one, double two)
{
	return ((one-two < std::numeric_limits<double>::epsilon())
		&& (one-two > -std::numeric_limits<double>::epsilon()));
}

RobotModel::RobotModel(const std::string& _name, const std::string& _eulerAngle, ROBOT_TYPE _rbt_type, Decare_Para Decare) :
	AxesGroup(_name), RobotName(_name), eulerAngle(_eulerAngle), rbt_type(_rbt_type),_Decare(Decare),
			baseMatrix(Matrix4::Identity( 4, 4)), cylinderMatrix(Matrix4::Identity( 4, 4)),
			toolMatrix(Matrix4::Identity(4, 4)),
			ODD_HANDLE_MARGIN(0.001), ODD_MARGIN(deg2rad(1))
{
	robotConfigFile = "config/" + RobotName + ".json";
	cylinderIntersectLine.R = 0;
	cylinderIntersectLine.r = 0;
	cylinderIntersectLine.e = 0;
	cylinderIntersectLine.alpha = 0;
	cancelCal = false;
	upsideDown = false;
	sixAxisAngleFor5S = 0;
	dynamicLimit.max = 0;
	dynamicLimit.min = 0;
    coupleCoe.Couple_Coe_1_2 = 0.0;
    coupleCoe.Couple_Coe_2_3 = 0.0;
    coupleCoe.Couple_Coe_3_2 = 0.0;
    coupleCoe.Couple_Coe_3_4 = 0.0;
    coupleCoe.Couple_Coe_4_5 = 0.0;
    coupleCoe.Couple_Coe_4_6 = 0.0;
    coupleCoe.Couple_Coe_5_6 = 0.0;
	pitch = 0;
    syncGroupSum = 0;
    curSyncNum = 0;
    for (int i = 0 ;i < 3; i++ )
    {
        syncType[i] = 0;
    }
    trackPara.existTrack = false;
    trackPara.trackCooperation = false;
    trackPara.trackSyncLocation = 0;
    trackPara.xConversionRatio = 9999;
    trackPara.yConversionRatio = 9999;
    trackPara.zConversionRatio = 9999;
    minTrajectTime.minAccTime = 0.1;
    minTrajectTime.minDecTime = 0.1;
}
RobotModel::~RobotModel()
{
}

ErrorID RobotModel::UpdateToolTransMatrixFromDate(double * dataBuff)
{
    Position rpy(6);

    rpy<<dataBuff[0], dataBuff[1], dataBuff[2], dataBuff[3], dataBuff[4], dataBuff[5];
    toolMatrix=rpy2tr(rpy);
    return RBTINTERF_NO_ERROR;
}

ErrorID RobotModel::UpdateUserTransMatrixFromDate(double * dataBuff)
{
    Position rpy(6);

    rpy<<dataBuff[0], dataBuff[1], dataBuff[2], dataBuff[3], dataBuff[4], dataBuff[5];
    baseMatrix=rpy2tr(rpy);
    return RBTINTERF_NO_ERROR;
}

void RobotModel::setSyncGroupAndType(size_t groupSum, const size_t* type)
{
    syncGroupSum = groupSum;
    for (unsigned int i = 0 ;i < syncGroupSum; i++ )
    {
        syncType[i] = type[i];
    }
}

ErrorID RobotModel::RobotRangeLimit(const Position_MCS_rad& posMCS)
{
	for (int i = 0; i < 99; i++)
	{
		if (range_limit[i].enable)
		{
			if (range_limit[i].isInInterfer)//生效区域在干涉区内
			{
				if (posMCS[0]>=range_limit[i].minX && posMCS[0]<=range_limit[i].maxX
						&& posMCS[1]>=range_limit[i].minY && posMCS[1]<=range_limit[i].maxY
						&& posMCS[2]>=range_limit[i].minZ && posMCS[2]<=range_limit[i].maxZ)
				{
					return RANGE_OVERLIMIT_IS_IN_INTERFER | ( i << 8);
				}
			}
			else
			{
				if (posMCS[0]<range_limit[i].minX || posMCS[0]>range_limit[i].maxX)
				{
					return RANGE_OVERLIMIT_X | ( i << 8);
				}
				if (posMCS[1]<range_limit[i].minY || posMCS[1]>range_limit[i].maxY)
				{
					return RANGE_OVERLIMIT_Y | ( i << 8);
				}
				if (posMCS[2]<range_limit[i].minZ || posMCS[2]>range_limit[i].maxZ)
				{
					return RANGE_OVERLIMIT_Z | ( i << 8);
				}
			}
		}
	}
	return RANGE_NOOVERLIMIT;
}

ErrorID RobotModel::RobotSpeedLimit(const Position_ACS_rad& pos,const Position_ACS_rad& posACS, double Ts, int num) const
{
	Position_ACS_rad firstPos(num);
	Position_ACS_rad secondPos(num);

	firstPos = pos;
	secondPos = posACS;

	for(int i = 0; i < num; i++)
	{
	    firstPos[i] = firstPos[i] * axes[i]->direction * (rbt_type == R_TYPE_SIZE_ ? 1 : axes[i]->axisDirection);
	    secondPos[i] = secondPos[i] * axes[i]->direction * (rbt_type == R_TYPE_SIZE_ ? 1 : axes[i]->axisDirection);
	}

    if (rbt_type == R_SCARA)
    {
        firstPos[num - 2] = firstPos[num - 2] - firstPos[num - 1] * coupleCoe.Couple_Coe_3_4;
        firstPos[1] = firstPos[1] - firstPos[0] * coupleCoe.Couple_Coe_1_2;
        secondPos[num - 2] = secondPos[num - 2] - secondPos[num - 1] * coupleCoe.Couple_Coe_3_4;
        secondPos[1] = secondPos[1] - secondPos[0] * coupleCoe.Couple_Coe_1_2;
    }
    else
    {
        if ((rbt_type != R_GENERAL_1S) && (rbt_type != R_SCARA_TWOAXIS)
                && (rbt_type != R_SCARA_THREEAXIS) && (rbt_type != R_TYPE_SIZE_))
        {
            firstPos[2] = firstPos[2] - firstPos[1] * coupleCoe.Couple_Coe_2_3;
            firstPos[num - 2] = firstPos[num - 2]  - firstPos[num - 3] * coupleCoe.Couple_Coe_4_5;
            firstPos[num - 1] = firstPos[num - 1]  - firstPos[num - 2] * coupleCoe.Couple_Coe_5_6  - firstPos[num - 3] * coupleCoe.Couple_Coe_4_6;
            secondPos[2] = secondPos[2] - secondPos[1] * coupleCoe.Couple_Coe_2_3;
            secondPos[num - 2] = secondPos[num - 2]  - secondPos[num - 3] * coupleCoe.Couple_Coe_4_5;
            secondPos[num - 1] = secondPos[num - 1]  - secondPos[num - 2] * coupleCoe.Couple_Coe_5_6  - secondPos[num - 3] * coupleCoe.Couple_Coe_4_6;
        }
    }

	for (int i = 0; i < num; i++)
	{
		double speed = rad2deg(firstPos[i]-secondPos[i]) / Ts;
		if ((fabs(speed) - axes[i]->paramLmt.RatedVel * axes[i]->paramLmt.maxRotSpeed) > 0.005 * axes[i]->paramLmt.RatedVel * axes[i]->paramLmt.maxRotSpeed)
		{
			return SPEED_OVERLIMIT+i;
		}
	}

	return SPEED_NOOVERLIMIT;
}

ErrorID RobotModel::RobotSpeedLimitMultiple (const Position_ACS_rad& pos,const Position_ACS_rad& posACS, double& multiple, double Ts, int num)
{
    Position_ACS_rad firstPos(num);
    Position_ACS_rad secondPos(num);

    firstPos = pos;
    secondPos = posACS;

    for(int i = 0; i < num; i++)
    {
        firstPos[i] = firstPos[i] * axes[i]->direction * (rbt_type == R_TYPE_SIZE_ ? 1 : axes[i]->axisDirection);
        secondPos[i] = secondPos[i] * axes[i]->direction * (rbt_type == R_TYPE_SIZE_ ? 1 : axes[i]->axisDirection);
    }

    if (rbt_type == R_SCARA)
    {
        firstPos[num - 2] = firstPos[num - 2] - firstPos[num - 1] * coupleCoe.Couple_Coe_3_4;
        firstPos[1] = firstPos[1] - firstPos[0] * coupleCoe.Couple_Coe_1_2;
        secondPos[num - 2] = secondPos[num - 2] - secondPos[num - 1] * coupleCoe.Couple_Coe_3_4;
        secondPos[1] = secondPos[1] - secondPos[0] * coupleCoe.Couple_Coe_1_2;
    }
    else
    {
        if ((rbt_type != R_GENERAL_1S) && (rbt_type != R_SCARA_TWOAXIS)
                && (rbt_type != R_SCARA_THREEAXIS) && (rbt_type != R_TYPE_SIZE_))
        {
            firstPos[2] = firstPos[2] - firstPos[1] * coupleCoe.Couple_Coe_2_3;
            firstPos[num - 2] = firstPos[num - 2]  - firstPos[num - 3] * coupleCoe.Couple_Coe_4_5;
            firstPos[num - 1] = firstPos[num - 1]  - firstPos[num - 2] * coupleCoe.Couple_Coe_5_6  - firstPos[num - 3] * coupleCoe.Couple_Coe_4_6;
            secondPos[2] = secondPos[2] - secondPos[1] * coupleCoe.Couple_Coe_2_3;
            secondPos[num - 2] = secondPos[num - 2]  - secondPos[num - 3] * coupleCoe.Couple_Coe_4_5;
            secondPos[num - 1] = secondPos[num - 1]  - secondPos[num - 2] * coupleCoe.Couple_Coe_5_6  - secondPos[num - 3] * coupleCoe.Couple_Coe_4_6;
        }
    }
    double multipleTmp = 0;
    ErrorID speedLimitError = SPEED_NOOVERLIMIT;
    for (int i = 0; i < num; i++)
    {
        double speed = rad2deg(firstPos[i]-secondPos[i]) / Ts;
		if ((fabs(speed) - axes[i]->paramLmt.RatedVel * axes[i]->paramLmt.maxRotSpeed) > 10e-9)
		{
			if (multipleTmp < fabs(speed) / (axes[i]->paramLmt.RatedVel * axes[i]->paramLmt.maxRotSpeed))
			{
				multipleTmp = fabs(speed) / (axes[i]->paramLmt.RatedVel * axes[i]->paramLmt.maxRotSpeed);
				speedLimitError = SPEED_OVERLIMIT + i;
			}
		}
    }
    multiple = multipleTmp;
    return speedLimitError;
}

ErrorID RobotModel::robot_acceleration_limit_multiple(const Position_ACS_rad &pos_time0,
		const Position_ACS_rad &pos_time1, const Position_ACS_rad &pos_time2, const double &Ts, const int &axes_num,
		double &acc_multiple)
{
	ErrorID lRet(RBTINTERF_NO_ERROR);
	Position_ACS_rad pos0(axes_num);
	Position_ACS_rad pos1(axes_num);
	Position_ACS_rad pos2(axes_num);
	pos0 = pos_time0;
	pos1 = pos_time1;
	pos2 = pos_time2;
	for (int i = 0; i < axes_num; i++)
	{
		pos0[i] = pos0[i] * axes[i]->direction * (rbt_type == R_TYPE_SIZE_ ? 1 : axes[i]->axisDirection);
		pos1[i] = pos1[i] * axes[i]->direction * (rbt_type == R_TYPE_SIZE_ ? 1 : axes[i]->axisDirection);
		pos2[i] = pos2[i] * axes[i]->direction * (rbt_type == R_TYPE_SIZE_ ? 1 : axes[i]->axisDirection);
	}
	if (rbt_type == R_SCARA)
	{
		pos0[axes_num - 2] = pos0[axes_num - 2] - pos0[axes_num - 1] * coupleCoe.Couple_Coe_3_4;
		pos0[1] = pos0[1] - pos0[0] * coupleCoe.Couple_Coe_1_2;
		pos1[axes_num - 2] = pos1[axes_num - 2] - pos1[axes_num - 1] * coupleCoe.Couple_Coe_3_4;
		pos1[1] = pos1[1] - pos1[0] * coupleCoe.Couple_Coe_1_2;
		pos2[axes_num - 2] = pos2[axes_num - 2] - pos2[axes_num - 1] * coupleCoe.Couple_Coe_3_4;
		pos2[1] = pos2[1] - pos2[0] * coupleCoe.Couple_Coe_1_2;
	}
	else
	{
		if ((rbt_type != R_GENERAL_1S) && (rbt_type != R_SCARA_TWOAXIS) && (rbt_type != R_SCARA_THREEAXIS)
				&& (rbt_type != R_TYPE_SIZE_))
		{
			pos0[2] = pos0[2] - pos0[1] * coupleCoe.Couple_Coe_2_3;
			pos0[axes_num - 2] = pos0[axes_num - 2] - pos0[axes_num - 3] * coupleCoe.Couple_Coe_4_5;
			pos0[axes_num - 1] = pos0[axes_num - 1] - pos0[axes_num - 2] * coupleCoe.Couple_Coe_5_6
					- pos0[axes_num - 3] * coupleCoe.Couple_Coe_4_6;
			pos1[2] = pos1[2] - pos1[1] * coupleCoe.Couple_Coe_2_3;
			pos1[axes_num - 2] = pos1[axes_num - 2] - pos1[axes_num - 3] * coupleCoe.Couple_Coe_4_5;
			pos1[axes_num - 1] = pos1[axes_num - 1] - pos1[axes_num - 2] * coupleCoe.Couple_Coe_5_6
					- pos1[axes_num - 3] * coupleCoe.Couple_Coe_4_6;
			pos2[2] = pos2[2] - pos2[1] * coupleCoe.Couple_Coe_2_3;
			pos2[axes_num - 2] = pos2[axes_num - 2] - pos2[axes_num - 3] * coupleCoe.Couple_Coe_4_5;
			pos2[axes_num - 1] = pos2[axes_num - 1] - pos2[axes_num - 2] * coupleCoe.Couple_Coe_5_6
					- pos2[axes_num - 3] * coupleCoe.Couple_Coe_4_6;
		}
	}
	for (int i = 0; i < axes_num; i++)
	{
		double acc = (rad2deg(pos1[i]-pos0[i]) / Ts - rad2deg(pos2[i]-pos1[i]) / Ts) / Ts;
		if (acc_multiple < fabs(acc) / (axes[i]->paramLmt.RatedVel * axes[i]->paramLmt.maxAcc))
		{
			acc_multiple = fabs(acc) / (axes[i]->paramLmt.RatedVel * axes[i]->paramLmt.maxAcc);
		}
	}
	return lRet;
}
ErrorID RobotModel::robot_jerk_limit_multiple(const Position_ACS_rad &pos_time0, const Position_ACS_rad &pos_time1,
		const Position_ACS_rad &pos_time2, const Position_ACS_rad &pos_time3, const double &Ts, const int &axis_sum,
		double &jerk_multiple)
{
	Position_ACS_rad pos0(axis_sum);
	Position_ACS_rad pos1(axis_sum);
	Position_ACS_rad pos2(axis_sum);
	Position_ACS_rad pos3(axis_sum);
	pos0 = pos_time0;
	pos1 = pos_time1;
	pos2 = pos_time2;
	pos3 = pos_time3;
	for (int i = 0; i < axis_sum; i++)
	{
		pos0[i] = pos0[i] * axes[i]->direction * (rbt_type == R_TYPE_SIZE_ ? 1 : axes[i]->axisDirection);
		pos1[i] = pos1[i] * axes[i]->direction * (rbt_type == R_TYPE_SIZE_ ? 1 : axes[i]->axisDirection);
		pos2[i] = pos2[i] * axes[i]->direction * (rbt_type == R_TYPE_SIZE_ ? 1 : axes[i]->axisDirection);
		pos3[i] = pos3[i] * axes[i]->direction * (rbt_type == R_TYPE_SIZE_ ? 1 : axes[i]->axisDirection);
	}
	if (rbt_type == R_SCARA)
	{
		pos0[axis_sum - 2] = pos0[axis_sum - 2] - pos0[axis_sum - 1] * coupleCoe.Couple_Coe_3_4;
		pos0[1] = pos0[1] - pos0[0] * coupleCoe.Couple_Coe_1_2;
		pos1[axis_sum - 2] = pos1[axis_sum - 2] - pos1[axis_sum - 1] * coupleCoe.Couple_Coe_3_4;
		pos1[1] = pos1[1] - pos1[0] * coupleCoe.Couple_Coe_1_2;
		pos2[axis_sum - 2] = pos2[axis_sum - 2] - pos2[axis_sum - 1] * coupleCoe.Couple_Coe_3_4;
		pos2[1] = pos2[1] - pos2[0] * coupleCoe.Couple_Coe_1_2;
		pos3[axis_sum - 2] = pos3[axis_sum - 2] - pos3[axis_sum - 1] * coupleCoe.Couple_Coe_3_4;
		pos3[1] = pos3[1] - pos3[0] * coupleCoe.Couple_Coe_1_2;
	}
	else
	{
		if ((rbt_type != R_GENERAL_1S) && (rbt_type != R_SCARA_TWOAXIS) && (rbt_type != R_SCARA_THREEAXIS)
				&& (rbt_type != R_TYPE_SIZE_))
		{
			pos0[2] = pos0[2] - pos0[1] * coupleCoe.Couple_Coe_2_3;
			pos0[axis_sum - 2] = pos0[axis_sum - 2] - pos0[axis_sum - 3] * coupleCoe.Couple_Coe_4_5;
			pos0[axis_sum - 1] = pos0[axis_sum - 1] - pos0[axis_sum - 2] * coupleCoe.Couple_Coe_5_6
					- pos0[axis_sum - 3] * coupleCoe.Couple_Coe_4_6;
			pos1[2] = pos1[2] - pos1[1] * coupleCoe.Couple_Coe_2_3;
			pos1[axis_sum - 2] = pos1[axis_sum - 2] - pos1[axis_sum - 3] * coupleCoe.Couple_Coe_4_5;
			pos1[axis_sum - 1] = pos1[axis_sum - 1] - pos1[axis_sum - 2] * coupleCoe.Couple_Coe_5_6
					- pos1[axis_sum - 3] * coupleCoe.Couple_Coe_4_6;
			pos2[2] = pos2[2] - pos2[1] * coupleCoe.Couple_Coe_2_3;
			pos2[axis_sum - 2] = pos2[axis_sum - 2] - pos2[axis_sum - 3] * coupleCoe.Couple_Coe_4_5;
			pos2[axis_sum - 1] = pos2[axis_sum - 1] - pos2[axis_sum - 2] * coupleCoe.Couple_Coe_5_6
					- pos2[axis_sum - 3] * coupleCoe.Couple_Coe_4_6;
			pos3[2] = pos3[2] - pos3[1] * coupleCoe.Couple_Coe_2_3;
			pos3[axis_sum - 2] = pos3[axis_sum - 2] - pos3[axis_sum - 3] * coupleCoe.Couple_Coe_4_5;
			pos3[axis_sum - 1] = pos3[axis_sum - 1] - pos3[axis_sum - 2] * coupleCoe.Couple_Coe_5_6
					- pos3[axis_sum - 3] * coupleCoe.Couple_Coe_4_6;
		}
	}
	double multiple_temp = 0;
	ErrorID speedLimitError = SPEED_NOOVERLIMIT;
	for (int i = 0; i < axis_sum; i++)
	{
		double vel0 = rad2deg(pos1[i]-pos0[i]) / Ts;
		double vel1 = rad2deg(pos2[i]-pos1[i]) / Ts;
		double vel2 = rad2deg(pos3[i]-pos2[i]) / Ts;
		double acc0 = (vel1 - vel0) / Ts;
		double acc1 = (vel2 - vel1) / Ts;
		double jerk = (acc1 - acc0) / Ts;
		if ((fabs(jerk) - 2 * 10 * (axes[i]->paramLmt.RatedVel) * axes[i]->paramLmt.maxJerkAcc) > 10e-9)
		{
			if (multiple_temp < fabs(jerk) / (2 * 10 * (axes[i]->paramLmt.RatedVel) * axes[i]->paramLmt.maxJerkAcc))
			{
				multiple_temp = fabs(jerk) / (2 * 10 * (axes[i]->paramLmt.RatedVel) * axes[i]->paramLmt.maxJerkAcc);
				speedLimitError = SPEED_OVERLIMIT + i;
			}
		}
	}
	jerk_multiple = multiple_temp;
	return speedLimitError;
}

bool RobotModel::robotDynamicLimit(const Position& pos,bool isRad) const
{
    if (rbt_type == R_FOURAXIS_PALLET)
    {
        if (isRad)
        {
            if ((pos[1] + pos[2] > dynamicLimit.max * M_PI / 180.0L) || (pos[1] + pos[2] < dynamicLimit.min * M_PI / 180.0L))
            {
                return true;
            }
        }
        else
        {
            if ((pos[1] + pos[2] > dynamicLimit.max) || (pos[1] + pos[2] < dynamicLimit.min))
            {
                return true;
            }
        }
    }
    else
    {
        return false;
    }
    return false;
}

double RobotModel::dynamicAdjustDec(const Position_ACS_rad& startPos,const Position_ACS_rad& endPos, bool rad) const
{
	if (rbt_type == R_SCARA || rbt_type == R_SCARA_TWOAXIS || rbt_type == R_SCARA_THREEAXIS)
	{
		double startTwoAxis = rad ? startPos[1] : startPos[1]  * M_PI / 180.0L ;
		double endTwoAxis = rad ? endPos[1] : endPos[1]  * M_PI / 180.0L ;

		while (startTwoAxis > M_PI)
		{
			startTwoAxis -= 2 * M_PI;
		}
		while (startTwoAxis < -M_PI)
		{
			startTwoAxis += 2 * M_PI;
		}

		while (endTwoAxis > M_PI)
		{
			endTwoAxis -= 2 * M_PI;
		}
		while (endTwoAxis < -M_PI)
		{
			endTwoAxis += 2 * M_PI;
		}

		if (startTwoAxis * endTwoAxis < 10e-13)
		{
			return 1.5;
		}
		else
		{
			double decTime = (fabs(startTwoAxis) > fabs(endTwoAxis) ? fabs(startTwoAxis) : fabs(endTwoAxis));
			return ((M_PI - decTime) / M_PI * 0.5 + 1);
		}
	}
	else if (rbt_type == R_SCARA_1 || rbt_type == R_SCARA_3_)
	{
		double startTwoAxis = startPos[2];
		double endTwoAxis = endPos[2];

		while (startTwoAxis > M_PI)
		{
			startTwoAxis -= 2 * M_PI;
		}
		while (startTwoAxis < -M_PI)
		{
			startTwoAxis += 2 * M_PI;
		}

		while (endTwoAxis > M_PI)
		{
			endTwoAxis -= 2 * M_PI;
		}
		while (endTwoAxis < -M_PI)
		{
			endTwoAxis += 2 * M_PI;
		}

		if (startTwoAxis * endTwoAxis < 10e-13)
		{
			return 1.5;
		}
		else
		{
			double decTime = (fabs(startTwoAxis) > fabs(endTwoAxis) ? fabs(startTwoAxis) : fabs(endTwoAxis));
			return ((M_PI - decTime) / M_PI * 0.5 + 1);
		}
	}
	else
	{
		return 1.0;
	}
}


//存储在DHJ的Theta是临时量，没有用
//世界坐标系
ErrorID RobotModel::calcTransMatrix(const Position_ACS_rad& posACS,
        Matrix4& transMatrix,double* swivel_angle, int beginAxisNum, bool inverse_check) const
{
	if ( (static_cast<std::deque<AXIS_REF>::size_type>(posACS.rows())
			!= axes.size()) || (transMatrix.rows()!=4) || (transMatrix.cols()
			!=4) || (unsigned int)beginAxisNum >= axes.size())
	{
		return FORWARD_KIN_TRANS_ARG_ERROR;
	}
	Position_MCS_rad pos(6);
	Position_ACS_rad q(posACS);
	Matrix4 currentMatrix(Matrix4::Identity( 4, 4));
    for (unsigned int i = 0; i < axes.size(); i++)
    {
        q[i] = q[i] * axes[i]->axisDirection;
    }

	// For each joint in this model
	if ((rbt_type == R_NULL) || (rbt_type == R_GENERAL_6S)
			|| (rbt_type == R_SCARA) || (rbt_type == R_GENERAL_6S_1)
			|| (rbt_type == R_GENERAL_5S) || (rbt_type == R_SCARA_TWOAXIS)
			|| (rbt_type == R_SCARA_THREEAXIS) || (rbt_type == R_GENERAL_7S_RBT)||(rbt_type == R_GENERAL_5S_COLLABORATIVE_)
			||(rbt_type == R_SCARA_1) || (rbt_type == R_SIXAXIS_SPRAY_BBR) || (rbt_type == R_GENERAL_6S_2) || (rbt_type == R_SCARA_3_))
	{
		for (std::deque<AXIS_REF>::size_type sz=beginAxisNum; sz<axes.size(); ++sz)
		{
			// Compute the joint using forward kinematics
			if ((rbt_type == R_NULL) || (rbt_type == R_GENERAL_6S)
			        || (rbt_type == R_GENERAL_6S_1) || (rbt_type == R_GENERAL_5S)
			        || (rbt_type == R_GENERAL_7S_RBT) || (rbt_type == R_SIXAXIS_SPRAY_BBR)
			        || (rbt_type == R_GENERAL_6S_2)||(rbt_type == R_GENERAL_5S_COLLABORATIVE_))
			{
				if (axes[sz])
				{
				    DH_Parameters dhParam;
					dhParam = axes[sz]->DHJ;

					dhParam.Theta = q(sz) + dhParam.Theta;// theta = posACS + offset
					currentMatrix = AllCompute(dhParam,currentMatrix);
					if ((sz == 4) && (rbt_type == R_GENERAL_5S))
					{
						dhParam.A = 0;
						dhParam.D = 0;
						dhParam.Alpha = 0;
						dhParam.Theta = sixAxisAngleFor5S;
						currentMatrix = AllCompute(dhParam,currentMatrix);
					}
				}
				else
				{
					std::cerr<<"DHJ can not be null.";
					return PLCOPEN_NULL_POINTER;
				}
			}
			else if (rbt_type == R_SCARA_THREEAXIS)
			{
				if (axes[sz])
				{
				    DH_Parameters dhParam;
					if (sz != 2)
					{
						dhParam = axes[sz]->DHJ;
						dhParam.Theta = q(sz) + dhParam.Theta;// theta = posACS + offset
						currentMatrix = AllCompute(dhParam, currentMatrix);
					}
					else
					{
						dhParam = axes[sz]->DHJ;
						dhParam.D = q(sz) / 2 / M_PI * pitch; // d = posACS/2/3.14*20 + d
						currentMatrix = AllCompute(dhParam,currentMatrix);
					}
				}
				else
				{
					std::cerr<<"DHJ can not be null.";
					return PLCOPEN_NULL_POINTER;
				}
			}
			else if ((rbt_type == R_SCARA) || (rbt_type == R_SCARA_TWOAXIS))
			{
				if (axes[sz])
				{
				    DH_Parameters dhParam;
					if (sz != 2)
					{
						dhParam = axes[sz]->DHJ;
						dhParam.Theta = q(sz) + dhParam.Theta;// theta = posACS + offset
						currentMatrix = AllCompute(dhParam, currentMatrix);
					}
					else
					{
						dhParam = axes[sz]->DHJ;
						dhParam.D = q(sz) / 2 / M_PI * pitch + 2*axes[3]->DHJ.D; // d = posACS/2/3.14*20 + d
						currentMatrix = AllCompute(dhParam,currentMatrix);
					}
				}
				else
				{
					std::cerr<<"DHJ can not be null.";
					return PLCOPEN_NULL_POINTER;
				}
			}
			else if(rbt_type == R_SCARA_1)
			{
				if (axes[sz])
				{
				    DH_Parameters dhParam;
					if (sz != 0)
					{
						dhParam = axes[sz]->DHJ;
						dhParam.Theta = q(sz) + dhParam.Theta;// theta = posACS + offset
						currentMatrix = AllCompute(dhParam, currentMatrix);
					}
					else
					{
						dhParam = axes[sz]->DHJ;
						dhParam.D = q(sz) / 2 / M_PI * pitch + 2*axes[3]->DHJ.D; // d = posACS/2/3.14*20 + d
						currentMatrix = AllCompute(dhParam,currentMatrix);
					}
				}
				else
				{
					std::cerr<<"DHJ can not be null.";
					return PLCOPEN_NULL_POINTER;
				}
			}
            else if (rbt_type == R_SCARA_3_)
            {
                if (axes[sz])
                {
                    DH_Parameters dhParam;
                    if (sz == 0)
                    {
                        dhParam = axes[sz]->DHJ;
                        dhParam.D += q(sz) / 2 / M_PI * pitch; // d = posACS/2/3.14*20 + d
                        currentMatrix = AllCompute(dhParam, currentMatrix);
                    }
                    else
                    {
                        dhParam = axes[sz]->DHJ;
                        dhParam.Theta = q(sz) + dhParam.Theta; // theta = posACS + offset
                        currentMatrix = AllCompute(dhParam, currentMatrix);
                    }
                }
                else
                {
                    std::cerr << "DHJ can not be null.";
                    return PLCOPEN_NULL_POINTER;
                }
            }
		}
	}
	else if (rbt_type == R_FOURAXIS_PALLET)
	{
		Position_MCS_rad posMcs(6);
		calcForwardKin_FourAxis_StackingRobot(q,posMcs);
		currentMatrix = rpy2tr(posMcs);
	}
    else if (rbt_type == R_FOURAXIS_PALLET_1)
    {
        Position_MCS_rad posMcs(6);
        calcForwardKin_FourAxis_StackingRobot_1(q,posMcs);
        currentMatrix = rpy2tr(posMcs);
    }
	else if (rbt_type == R_FOUR_CARTESIAN_COORDINATE)
	{
		Position_MCS_rad posMcs(6);
		calcForwardKin_Four_CartesianCoordinateRobot(q,posMcs);
		currentMatrix = rpy2tr(posMcs);
	}
	else if (rbt_type == R_THREE_CARTESIAN_COORDINATE)
	{
		Position_MCS_rad posMcs(6);
		calcForwardKin_Three_CartesianCoordinateRobot(q,posMcs);
		currentMatrix = rpy2tr(posMcs);
	}
    else if (rbt_type == R_THREE_CARTESIAN_COORDINATE_1)
    {
        Position_MCS_rad posMcs(6);
        calcForwardKin_Three_CartesianCoordinateRobot_1(q,posMcs);
        currentMatrix = rpy2tr(posMcs);
    }
    else if (rbt_type == R_FOUR_POLAR_COORDINATE_1)
    {
        Position_MCS_rad posMcs(6);
        calcForwardKin_FOUR_POLAR_COORDINATE_1(q,posMcs);
        currentMatrix = rpy2tr(posMcs);
    }
	else if (rbt_type == R_FOURAXIS)
	{
		Position_MCS_rad posMcs(6);
		calcForwardKin_FourAxisRobot(q,posMcs);
		currentMatrix = rpy2tr(posMcs);
	}
	else if (rbt_type == R_GANTRY_WELD)
	{
		Position_MCS_rad posMcs(6);
		calcForwardKin_GANTRY_WELD(q,posMcs);
		currentMatrix = rpy2tr(posMcs);
	}
	else if(rbt_type == R_GANTRY_WELD_2_)
	{
		Position_MCS_rad posMcs(6);
		calcForwardKin_GANTRY_WELD_2(q, posMcs);
		currentMatrix = rpy2tr(posMcs);
	}
	else if (rbt_type == R_DELTA)
	{
		Position_MCS_rad posMcs(6);
		calcForwardKin_DeltaRobot(q,posMcs);
		currentMatrix = rpy2tr(posMcs);
	}
	else if (rbt_type == R_WINE_CHAMFER)
	{
		Position_MCS_rad posMcs(6);
		calcForwardKin_WINE_CHAMFER(q,posMcs);
		currentMatrix = rpy2tr(posMcs);
	}
	else
	{
		return INVERSE_KIN_ARG_ERROR;
	}
	// Update the model transformation matrix
	for (int i=0;i<6;i++)
	{
	    pos[i]=0;
	}
	if (upsideDown == true)
	{
        pos[4]=M_PI;
        pos[5]=M_PI;
	}
	transMatrix = rpy2tr(pos)*currentMatrix * toolMatrix;

	if (axes.size() == 7 && !inverse_check)//臂角
	{
		calcSwivelAngle(posACS,swivel_angle);
	}

	return RBTINTERF_NO_ERROR;
}

ErrorID RobotModel::calcForwardKin(const Position_ACS_rad& posACS,
		Position_MCS_rad& posMCS, int beginAxisNum)
{
    AXIS_REFS_INDEX mcsDimension = (axes.size() > 6)?7:6;
	if ( (static_cast<std::deque<AXIS_REF>::size_type>(posACS.rows())
			!= axes.size()) || (static_cast<std::deque<AXIS_REF>::size_type>(posMCS.rows()) != mcsDimension)
			|| (unsigned int)beginAxisNum >= axes.size())
	{
		return FORWARD_KIN_ARG_ERROR;
	}
	// Update the model transformation matrix
	Matrix4 currentMatrix(Matrix4::Identity( 4, 4));
	double angleValue = 0;
	double* swivel_angle = &angleValue;
	calcTransMatrix(posACS, currentMatrix,swivel_angle, beginAxisNum);
	// tcp
	posMCS.head(6) << currentMatrix.topRightCorner(3, 1),
			tr2rpy(currentMatrix);
	if (mcsDimension == 7)//臂角
	{
	    posMCS[6] = *swivel_angle;
	}
	return RBTINTERF_NO_ERROR;
}

void RobotModel::calculate_theta4_when_zero_pos()
{
	double d3=axes[2]->DHJ.D/1000;
	double d5=axes[4]->DHJ.D/1000;
	double a2=axes[1]->DHJ.A/1000;
	double a3=axes[2]->DHJ.A/1000;
	double a4=axes[3]->DHJ.A/1000;

	Vector3d X24,X46,X26;
	X24<< a3 + a2, 0, d3;
	X46<< d5, 0, a4;
	X26<< a3 + a2 + d5, 0, d3 + a4;

	double d24=X24.norm();
	double d46=X46.norm();
	double d26=X26.norm();

	theta4_when_zero_pos = acos((d24*d24+d46*d46-d26*d26)/(2*d24*d46));
}

double RobotModel::get_theta4_when_zero_pos()
{
	return theta4_when_zero_pos;
}

//当前角度、解一、解二
double RobotModel::calcRealAngle(double curAng, double candidateAng1,
		double candidateAng2) const
{
	double curAngCopy(curAng);
	double* allAng[3] =
	{ &candidateAng1, &candidateAng2, &curAngCopy };
	for (int i = 0; i < 3; ++i)
	{
		while (*allAng[i]> M_PI)
		{
			*allAng[i] -= 2*M_PI;
		}
		while (*allAng[i] < -M_PI)
		{
			*allAng[i] += 2*M_PI;
		}
	}
	double gap[2] =
	{ 0, 0 };
	bool dir[2] =
	{ true, true };
	for (int i = 0; i < 2; ++i)
	{
		if (*allAng[i] >= curAngCopy)/*=*/
		{
			gap[i] = *allAng[i]-curAngCopy;
			dir[i] = true;
		}
		else
		{
			gap[i] = curAngCopy-*allAng[i];
			dir[i] = false;
		}
		if (gap[i]> M_PI)
		{
			gap[i] = 2*M_PI-gap[i];
			dir[i] = !dir[i];
		}
	}
	if (gap[0] <= gap[1])/*=*/
	{
		return dir[0] ? curAng+gap[0] : curAng-gap[0];
	}
	else
	{
		return dir[1] ? curAng+gap[1] : curAng-gap[1];
	}
}

double RobotModel::calcLargeAngle(double curAng, double candidateAng1,
        double candidateAng2) const
{
    double curAngCopy(curAng);
    double* allAng[3] =
    { &candidateAng1, &candidateAng2, &curAngCopy };
    for (int i = 0; i < 3; ++i)
    {
        while (*allAng[i]> M_PI)
        {
            *allAng[i] -= 2*M_PI;
        }
        while (*allAng[i] < -M_PI)
        {
            *allAng[i] += 2*M_PI;
        }
    }
    double gap[2] =
    { 0, 0 };
    bool dir[2] =
    { true, true };
    for (int i = 0; i < 2; ++i)
    {
        if (*allAng[i] >= curAngCopy)/*=*/
        {
            gap[i] = *allAng[i]-curAngCopy;
            dir[i] = true;
        }
        else
        {
            gap[i] = curAngCopy-*allAng[i];
            dir[i] = false;
        }
        if (gap[i]> M_PI)
        {
            gap[i] = 2*M_PI-gap[i];
            dir[i] = !dir[i];
        }
    }

    if (gap[0] > gap[1])/*=*/
    {
        return dir[0] ? curAng+gap[0] : curAng-gap[0];
    }
    else
    {
        return dir[1] ? curAng+gap[1] : curAng-gap[1];
    }
}
//is_Ntimes_90deg_Odd根据实际判断，因为处理过程中考虑了offset
bool RobotModel::is_Near_Odd(double theta_rad, double agl_offset_rad,
		bool is_Ntimes_90deg_Odd) const
{
	//比较的是实际的角度
	double agl_rad = theta_rad - agl_offset_rad;

	//-pi~pi
	if (is_Ntimes_90deg_Odd)
	{
		//-pi/2~pi/2
		while (agl_rad> M_PI/2)
		{
			agl_rad -= M_PI;
		}
		while (agl_rad < -M_PI/2)
		{
			agl_rad += M_PI;
		}
		if ((fabs(agl_rad-M_PI/2)<ODD_MARGIN) ||(fabs(agl_rad+M_PI/2)
				<ODD_MARGIN))
		{
			// DEBUG("ODD %f\r\n", rad2deg(agl_rad));
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		//0~pi
		while (agl_rad> M_PI)
		{
			agl_rad -= M_PI;
		}
		while (agl_rad < 0)
		{
			agl_rad += M_PI;
		}
		if ((fabs(agl_rad)<ODD_MARGIN)||(fabs(agl_rad-M_PI)<ODD_MARGIN))
		{
			//			DEBUG("ODD %f\r\n", rad2deg(agl_rad));
			return true;
		}
		else
		{
			return false;
		}
	}
}
ErrorID RobotModel::calcInverseKin_Trans_6s_and_5axis_offset(const Matrix4& transMatrix,
		const Position_ACS_rad& posLast, Position_ACS_rad& posACS , CalculateParameter& calculateParameter , bool optimize) const
{
    Position_ACS_rad posLastTemp(posLast);
	int flag = 1;
	Matrix4 T(transMatrix);
	T *= toolMatrix.inverse();
	Vector3 temp;
	ErrorID lRet = calcInverseKin_Trans_6s(T, posLastTemp, posACS,temp,flag, calculateParameter, optimize);
	double d5=axes[4]->DHJ.D;
	if(fabs(d5) > 10e-13)
	{
		Position_MCS_rad mscTemp;
	    DH_Parameters dhParam;
	    Matrix4 currentMatrix(Matrix4::Identity( 4, 4));
		Position_ACS_rad q(posACS);
		for (std::deque<AXIS_REF>::size_type sz=0; sz<axes.size(); ++sz)
		{
			dhParam = axes[sz]->DHJ;

			dhParam.Theta = q(sz) + dhParam.Theta;// theta = posACS + offset
			currentMatrix = AllCompute(dhParam,currentMatrix);
		}

		double distance = pow(T(0,3) - currentMatrix(0,3),2) + pow(T(1,3) - currentMatrix(1,3),2) + pow(T(2,3) - currentMatrix(2,3),2);
		Matrix4 temp_matriax;
		flag++;
		while(distance>0.0001)
		{
			dhParam = axes[5]->DHJ;
			dhParam.Theta = posACS(5) + dhParam.Theta;// theta = posACS + offset
			Matrix4 T56 = AllCompute(dhParam,Matrix4::Identity(4, 4));
			Matrix4 invT56;
			invT56 = T56.inverse();
//			pseudoInverseConst(T56,invT56);

			Matrix4 offset_matriax(Matrix4::Identity(4, 4));
			offset_matriax<<1 , 0 , 0 , 0,
					0 , 1 , 0 , d5,
					0 , 0 , 1 , 0,
					0 , 0 , 0 , 1;

			temp_matriax = T * invT56 * offset_matriax;
			temp(0) = temp_matriax(0,3);
			temp(1) = temp_matriax(1,3);
			temp(2) = temp_matriax(2,3);
		    lRet = calcInverseKin_Trans_6s(T, posLastTemp, posACS,temp,flag, calculateParameter, optimize);

		    currentMatrix = Matrix4::Identity(4, 4);

			for (std::deque<AXIS_REF>::size_type sz=0; sz<axes.size(); ++sz)
			{
				dhParam = axes[sz]->DHJ;

				dhParam.Theta = posACS(sz) + dhParam.Theta;// theta = posACS + offset
				currentMatrix = AllCompute(dhParam,currentMatrix);
			}

			distance = pow(T(0,3) - currentMatrix(0,3),2) + pow(T(1,3) - currentMatrix(1,3),2) + pow(T(2,3) - currentMatrix(2,3),2);
			flag++;
			if(flag == 50)
				break;
		}
	}
	return lRet;
}
ErrorID RobotModel::calcInverseKin_Trans_6s(const Matrix4& transMatrix,
		const Position_ACS_rad& posLast, Position_ACS_rad& posACS ,Vector3 temp,int flag,
		CalculateParameter& calculateParameter , bool optimize) const
{
	double d1 = 0.0;
	double d2 = 0.0;
	double d4 = 0.0;
	double d5 = 0.0;
	double d6 = 0.0;
	double a1 = 0.0;
	double a2 = 0.0;
	double a3 = 0.0;
	double smax = 0.0;
	double smin = 0.0;
	double lmax = 0.0;
	double lmin = 0.0;
	double umax = 0.0;
	double umin = 0.0;
	double rmax  = 0.0;
	double rmin = 0.0;
	double bmax = 0.0;
	double bmin = 0.0;
	double tmax = 0.0;
	double tmin = 0.0;

	if(!optimize)
		calculateParameter.configuration = 0;
	d5 = 0;
	if (rbt_type == R_GENERAL_6S)
	{
		if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
					!=6)||(posACS.rows() !=6) ||(axes.size()!=6))
		{
			return INVERSE_KIN_TRANS_ARG_ERROR;
		}
		//vars defined temporarily
		d1=axes[0]->DHJ.D/1000;
		d2=axes[1]->DHJ.D/1000;
		d4=axes[3]->DHJ.D/1000;
		d5=axes[4]->DHJ.D/1000;
		d6=axes[5]->DHJ.D/1000;
		a1=axes[0]->DHJ.A/1000;
		a2=axes[1]->DHJ.A/1000;
		a3=axes[2]->DHJ.A/1000;

		//第一轴角度限制范围
		smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
		smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
		//第二轴角度限制范围
		lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
		lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
		//第三轴角度限制范围
		umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
		umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
		//第四轴角度限制范围
		rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
		rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);
		//第五轴角度限制范围
		bmax=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.posSWLimit):(-1 * axes[4]->paramLmt.negSWLimit);
	    bmin=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.negSWLimit):(-1 * axes[4]->paramLmt.posSWLimit);
		//第六轴角度限制范围
		tmax=(axes[5]->axisDirection == 1)?(axes[5]->paramLmt.posSWLimit):(-1 * axes[5]->paramLmt.negSWLimit);
		tmin=(axes[5]->axisDirection == 1)?(axes[5]->paramLmt.negSWLimit):(-1 * axes[5]->paramLmt.posSWLimit);
	}
	else if (rbt_type == R_GENERAL_5S)
	{
		if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
					!=5)||(posACS.rows() !=5) ||(axes.size()!=5))
		{
			return INVERSE_KIN_TRANS_ARG_ERROR;
		}
		d1=axes[0]->DHJ.D/1000;
		d2=axes[1]->DHJ.D/1000;
		d4=axes[3]->DHJ.D/1000;
		d6=0;
		a1=axes[0]->DHJ.A/1000;
		a2=axes[1]->DHJ.A/1000;
		a3=axes[2]->DHJ.A/1000;

        //第一轴角度限制范围
        smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
        smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
        //第二轴角度限制范围
        lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
        lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
        //第三轴角度限制范围
        umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
        umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
        //第四轴角度限制范围
        rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
        rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);
        //第五轴角度限制范围
        bmax=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.posSWLimit):(-1 * axes[4]->paramLmt.negSWLimit);
        bmin=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.negSWLimit):(-1 * axes[4]->paramLmt.posSWLimit);
	}

	Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

	double nx(transMatrix(0,0)), ny(transMatrix(1,0)), nz(transMatrix(2,0));
	double ox(transMatrix(0,1)), oy(transMatrix(1,1)), oz(transMatrix(2,1));
	double ax(transMatrix(0,2)), ay(transMatrix(1,2)), az(transMatrix(2,2));
	double px(transMatrix(0,3)/1000.0), py(transMatrix(1,3)/1000.0), pz(transMatrix(2,3)/1000.0);

	//读取theat5偏移量
	double theta5_offset = axes[4]->DHJ.Theta;

	//solve for theta1
	double theta1_1,theta1_2;
	double n;
	double m;
	double q;
	if(flag == 1)
	{
		n = px-d6*ax;
		m = py-d6*ay;
		q = pz - d6*az;
	}
	else
	{
	    n=temp(0) / 1000;
	    m=temp(1) / 1000;
	    q=temp(2) / 1000;
	}
	double temp_var = pow(m,2) + pow(n,2) - pow(d2,2);
	if(temp_var < 0.0L)
	{
		if(fabs(d5) < 10e-13)
		{
			printf("Theta1 can not be solved, so can not reach the point!\n");
			jointLimitNum = 1;
			return INVERSE_KIN_NOT_REACHABLE;
		}
		else
			temp_var = 0;
	}
	if((fabs((py-d6*ay)) < 10e-13 ) && (fabs((px-d6*ax)) < 10e-13))
	{
		theta1_1 = pLast(0);
		theta1_2 = theta1_1;
	}
	else
	{
		theta1_1 = atan2(n*d2+m*sqrt(temp_var),-m*d2+n*sqrt(temp_var));
		theta1_2 = atan2(n*d2-m*sqrt(temp_var),-m*d2-n*sqrt(temp_var));
	}
	double theta1;

	/*if (theta1_1 <= 0.0L)
		theta1_2 = theta1_1 + M_PI;
	else
		theta1_2 = theta1_1 - M_PI;*/
	if(1 <= calculateParameter.configuration && calculateParameter.configuration < 5)
	{
		if(-M_PI/2 < theta1_1 && theta1_1 < M_PI/2)
		{
			if(-M_PI/2 < theta1_2 && theta1_2 < M_PI/2)
			{
				theta1 = calcRealAngle(pLast(0), theta1_1, theta1_2);
			}
			else
				theta1 = theta1_2;
		}
		else
		{
			if(-M_PI/2 < theta1_2 && theta1_2 < M_PI/2)
			{
				theta1 = theta1_1;
			}
			else
				theta1 = calcRealAngle(pLast(0), theta1_1, theta1_2);
		}
	}
	else if(5 <= calculateParameter.configuration && calculateParameter.configuration< 9)
	{
		if(-M_PI/2 < theta1_1 && theta1_1< M_PI/2)
		{
			if(-M_PI/2 < theta1_2 && theta1_2< M_PI/2)
			{
				theta1 = calcRealAngle(pLast(0), theta1_1, theta1_2);
			}
			else
				theta1 = theta1_1;
		}
		else
		{
			if(-M_PI/2 < theta1_2 && theta1_2< M_PI/2)
			{
				theta1 = theta1_2;
			}
			else
				theta1 = calcRealAngle(pLast(0), theta1_1, theta1_2);
		}
	}
	else
		theta1 = calcRealAngle(pLast(0), theta1_1, theta1_2);

	// the limit of theta1 according to the reference
	if((theta1 < M_PI*smin/180.0L) || (theta1 > M_PI*smax/180.0L))
	{
		if(fabs(d5) < 10e-13)
		{
			theta1 = pLast(0);
			printf("theta1 exceeds pos limit.\n");
			jointLimitNum = 1;
			return INVERSE_KIN_NOT_REACHABLE;
		}
	}
	posACS(0) = theta1;
	//solve for theta3

//	double k1 = cos(theta1)*px+sin(theta1)*py-a1-d6*(cos(theta1)*ax+sin(theta1)*ay);
	double k1 = n*cos(theta1)+m*sin(theta1)-a1;

	double k2 =  - d1 + q;
	double k3 = pow(k1, 2) + pow(k2, 2) - pow(a2, 2) - pow(a3, 2) - pow(d4, 2);
	double k4 = k3/(2*a2);
	temp_var = pow(a3, 2) + pow(d4, 2) - pow(k4, 2);
	if(temp_var < 0.0L)
	{
		if(fabs(d5) < 10e-13)
		{
			printf("Theta3 can not be solved, so can not reach the point!\n");
			jointLimitNum = 3;
			return INVERSE_KIN_NOT_REACHABLE;
		}
		else
			temp_var = 0;
	}
	double delta = sqrt(temp_var);
	double theta3_1 = atan2(d4, a3) + atan2(delta, k4);
	double theta3_2 = atan2(d4, a3) - atan2(delta, k4);
	double theta3;

	if(calculateParameter.configuration == 1 ||calculateParameter.configuration == 2 ||
			calculateParameter.configuration == 5 ||calculateParameter.configuration == 6 )
	{
		if(-M_PI/2 < theta3_1 && theta3_1< M_PI/2)
		{
			if(-M_PI/2 < theta3_2 && theta3_2< M_PI/2)
			{
				theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);
			}
			else
				theta3 = theta3_2;
		}
		else
		{
			if(-M_PI/2 < theta3_2 && theta3_2< M_PI/2)
			{
				theta3 = theta3_1;
			}
			else
				theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);
		}
	}
	else if(calculateParameter.configuration == 3 ||calculateParameter.configuration == 4 ||
			calculateParameter.configuration == 7 ||calculateParameter.configuration == 8 )
	{
		if(-M_PI/2 < theta3_1 && theta3_1< M_PI/2)
		{
			if(-M_PI/2 < theta3_2 && theta3_2< M_PI/2)
				theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);
			else
				theta3 = theta3_1;
		}
		else
		{
			if(-M_PI/2 < theta3_2 && theta3_2< M_PI/2)
				theta3 = theta3_2;
			else
				theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);
		}
	}
	else
		theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);

	// the limit of theta3 according to the reference
	if((theta3 < umin/180.0L*M_PI) || (theta3 > umax/180.0L*M_PI))
	{
		if(fabs(d5) < 10e-13)
		{
			theta3 = pLast(2);
			printf("theta3 exceeds pos limit.\n");
			jointLimitNum = 3;
			return INVERSE_KIN_NOT_REACHABLE;
		}
	}
	posACS(2) = theta3;

	//solve for theta2
	k1 = n*cos(theta1)+m*sin(theta1)-a1;
	k2 = -d1 + q;
	double a = d4*cos(theta3) - a3*sin(theta3);
	double b = d4*sin(theta3) + a2 +a3*cos(theta3);
	//已经加入了theta2的offset -pi/2    theta(运算) = theta(电机) - pi/2
	double theta2_1;
	if((fabs(a*k1 + b*k2) < 10e-13)  && (fabs(b*k1 - a*k2) < 10e-13))
		theta2_1 = pLast(1);
	else
		theta2_1 = atan2((a*k1 + b*k2),(b*k1 - a*k2)) - M_PI/2.0;
	double theta2;

	theta2 = calcRealAngle(pLast(1), theta2_1, theta2_1);

	// the limit of theta2 according to the reference
	if((theta2 < lmin/180.0L*M_PI) || (theta2 > lmax/180.0L*M_PI))
	{
		if(fabs(d5) < 10e-13)
		{
			theta2 = pLast(1);
			printf("theta2 exceeds pos limit.\n");
			jointLimitNum = 2;
			return INVERSE_KIN_NOT_REACHABLE;
		}
	}
	posACS(1) = theta2;
	//solve for theta4

	k1 = sin(theta1)*ax - cos(theta1)*ay;
	k2 = cos(theta1)*cos(theta2 + M_PI/2.0 + theta3)*ax + sin(theta1)*
			cos(theta2 + M_PI/2.0 + theta3)*ay + sin(theta2 + M_PI/2.0 + theta3)*az;

	double theta4;
	double theta4_1;
	double theta4_2;
	//此处的判断阈值不能过小，过小的话，当0/0时，它无法识别出来

	if(((fabs(k1) < 10e-7) && (fabs(k2) < 10e-7)))
	{
		theta4 = pLast(3);
		//cout << "A" << endl;
	}
	else
	{
		theta4_1 = atan2(k1,k2);
		if(theta4_1 > 0.0L)
			theta4_2 = theta4_1 - M_PI;
		else
			theta4_2 = theta4_1 + M_PI;
		theta4 = calcRealAngle(pLast(3), theta4_1, theta4_2);
	}
	posACS(3) = theta4;

	//solve for theta5
	double k1_1 = sin(theta1)*sin(theta4) + cos(theta1)*cos(theta4)*cos(theta2 + M_PI/2.0 + theta3);
	double k1_2 = -cos(theta1)*sin(theta4) + sin(theta1)*cos(theta4)*cos(theta2 + M_PI/2.0 + theta3);
	double k1_3 = cos(theta4)*sin(theta2 + M_PI/2.0 + theta3);
	k1 = k1_1*ax + k1_2*ay + k1_3*az;
	k2 = cos(theta1)*sin(theta2 + M_PI/2.0 + theta3)*ax + sin(theta1)*sin(theta2 + M_PI/2.0 + theta3)*
			ay - cos(theta2 + M_PI/2.0 + theta3)*az;
	double theta5_1;
	if ((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
	{
		theta5_1 = pLast(4);
	}
	else
	{
		if (fabs(theta5_offset - M_PI / 2) < 10e-4)	//判断第五轴是垂直还是平行
		{
			theta5_1 = atan2(-k1, k2) - M_PI / 2.0;
		}
		else
		{
			theta5_1 = atan2(-k1, k2);
		}
	}

	double theta5;
	double minConfigurationLimit = -M_PI/2;
	double maxConfigurationLimit = M_PI/2;

	if (fabs(theta5_offset - M_PI / 2) < 10e-4)	//判断第五轴是垂直还是平行
	{
		minConfigurationLimit = -M_PI / 2;
		maxConfigurationLimit = M_PI / 2;
	}
	else
	{
		minConfigurationLimit = 0;
		maxConfigurationLimit = M_PI;
	}

	if(calculateParameter.configuration == 2 ||calculateParameter.configuration == 4 ||
				calculateParameter.configuration == 6 ||calculateParameter.configuration == 8 )
	{
		if(minConfigurationLimit < theta5_1 && theta5_1< maxConfigurationLimit)
		{
			theta5 = calcRealAngle(pLast(4), theta5_1, theta5_1);
		}
		else
		{
			theta4 = calcLargeAngle(pLast(3), theta4_1, theta4_2);
			if((theta4 < rmin*M_PI/180.0L) || (theta4 > rmax*M_PI/180.0L))
			{
				if(fabs(d5) < 10e-13)
				{
					theta4 = pLast(3);
					printf("theta4 exceeds pos limit.\n");
					jointLimitNum = 4;
					return INVERSE_KIN_NOT_REACHABLE;
				}
			}
			posACS(3) = theta4;
			k1_1 = sin(theta1)*sin(theta4) + cos(theta1)*cos(theta4)*cos(theta2 + M_PI/2.0 + theta3);
			k1_2 = -cos(theta1)*sin(theta4) + sin(theta1)*cos(theta4)*cos(theta2 + M_PI/2.0 + theta3);
			k1_3 = cos(theta4)*sin(theta2 + M_PI/2.0 + theta3);
			k1 = k1_1*ax + k1_2*ay + k1_3*az;
			k2 = cos(theta1)*sin(theta2 + M_PI/2.0 + theta3)*ax + sin(theta1)*sin(theta2 + M_PI/2.0 + theta3)*
					ay - cos(theta2 + M_PI/2.0 + theta3)*az;
			if ((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
			{
				theta5_1 = pLast(4);
			}
			else
			{
				if (fabs(theta5_offset - M_PI / 2) < 10e-4)	//判断第五轴是垂直还是平行
				{
					theta5_1 = atan2(-k1, k2) - M_PI / 2.0;
				}
				else
				{
					theta5_1 = atan2(-k1, k2);
				}
			}
			if(minConfigurationLimit < theta5_1 && theta5_1< maxConfigurationLimit)
			{
				theta5 = calcRealAngle(pLast(4), theta5_1, theta5_1);
			}
			else
			{
				theta5 = pLast(4);
				printf("theta5 exceeds pos limit.\n");
				jointLimitNum = 5;
				return INVERSE_KIN_NOT_REACHABLE;
			}
		}
	}
	else if(calculateParameter.configuration == 1 ||calculateParameter.configuration == 3 ||
				calculateParameter.configuration == 5 ||calculateParameter.configuration == 7 )
	{
		if(minConfigurationLimit < theta5_1 && theta5_1< maxConfigurationLimit)
		{
			theta4 = calcLargeAngle(pLast(3), theta4_1, theta4_2);
			if((theta4 < rmin*M_PI/180.0L) || (theta4 > rmax*M_PI/180.0L))
			{
				if(fabs(d5) < 10e-13)
				{
					theta4 = pLast(3);
					printf("theta4 exceeds pos limit.\n");
					jointLimitNum = 4;
					return INVERSE_KIN_NOT_REACHABLE;
				}
			}
			posACS(3) = theta4;
			k1_1 = sin(theta1)*sin(theta4) + cos(theta1)*cos(theta4)*cos(theta2 + M_PI/2.0 + theta3);
			k1_2 = -cos(theta1)*sin(theta4) + sin(theta1)*cos(theta4)*cos(theta2 + M_PI/2.0 + theta3);
			k1_3 = cos(theta4)*sin(theta2 + M_PI/2.0 + theta3);
			k1 = k1_1*ax + k1_2*ay + k1_3*az;
			k2 = cos(theta1)*sin(theta2 + M_PI/2.0 + theta3)*ax + sin(theta1)*sin(theta2 + M_PI/2.0 + theta3)*
					ay - cos(theta2 + M_PI/2.0 + theta3)*az;
			if ((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
			{
				theta5_1 = pLast(4);
			}
			else
			{
				if (fabs(theta5_offset - M_PI / 2) < 10e-4)	//判断第五轴是垂直还是平行
				{
					theta5_1 = atan2(-k1, k2) - M_PI / 2.0;
				}
				else
				{
					theta5_1 = atan2(-k1, k2);
				}
			}
			if(minConfigurationLimit < theta5_1 && theta5_1< maxConfigurationLimit)
			{
				theta5 = pLast(4);
				printf("theta5 exceeds pos limit.\n");
				jointLimitNum = 5;
				return INVERSE_KIN_NOT_REACHABLE;
			}
			else
			{
				theta5 = calcRealAngle(pLast(4), theta5_1, theta5_1);
			}
		}
		else
		{
			theta5 = calcRealAngle(pLast(4), theta5_1, theta5_1);
		}
	}
	else
	{
		theta5 = calcRealAngle(pLast(4), theta5_1, theta5_1);
	}

	// the limit of theta4 according to the reference
	if((theta4 < rmin*M_PI/180.0L) || (theta4 > rmax*M_PI/180.0L))
	{
		if(fabs(d5) < 10e-13)
		{
			theta4 = pLast(3);
			printf("theta4 exceeds pos limit.\n");
			jointLimitNum = 4;
			return INVERSE_KIN_NOT_REACHABLE;
		}
	}

	// the limit of theta5 according to the reference
	if((theta5 < bmin/180.0L*M_PI) || (theta5 > bmax/180.0L*M_PI))
	{
		if(fabs(d5) < 10e-13)
		{
			theta5 = pLast(4);
			printf("theta5 exceeds pos limit.\n");
			jointLimitNum = 5;
			return INVERSE_KIN_NOT_REACHABLE;
		}
	}
	posACS(4) = theta5; 
		
//	if (fabs(theta5_offset - M_PI / 2) < 10e-4)	//判断第五轴是垂直还是平行
//	{
//		if (((fabs(theta5 + M_PI / 2.0) < 1 /180.0L*M_PI && fabs(theta4 - pLast(3)) > fabs(theta5 - pLast(4))) || (fabs(theta4 - pLast(3)) > 0.2 /180.0L*M_PI)) && !optimize)
//		{
//			theta4 = pLast(3) + (theta5 - pLast(4)) * ((theta4 * (theta5 + M_PI / 2.0) > 0) ? 1 : -1);
//			posACS(3) = theta4;
//		}
//	}
//	else
//	{
//		if (((fabs(theta5) < 1 /180.0L*M_PI && fabs(theta4 - pLast(3)) > fabs(theta5 - pLast(4))) || (fabs(theta4 - pLast(3)) > 0.2 /180.0L*M_PI)) && !optimize)
//		{
//			theta4 = pLast(3) + (theta5 - pLast(4)) * ((theta4 * theta5 > 0) ? 1 : -1);
//			posACS(3) = theta4;
//		}
//	}


	if (rbt_type == R_GENERAL_6S)
	{
		//solve for theta6
		double k1_4=sin(theta4)*cos(theta2+ M_PI/2.0+theta3)*cos(theta1)-cos(theta4)*sin(theta1);
		double k1_5=sin(theta4)*cos(theta2+ M_PI/2.0+theta3)*sin(theta1)+cos(theta4)*cos(theta1);
		double k1_6=sin(theta4)*sin(theta2+ M_PI/2.0+theta3);
		 k2=-k1_4*ox - k1_5*oy - k1_6*oz;
		 k1=-k1_4*nx - k1_5*ny - k1_6*nz;

		double theta6_1;
		if((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
			theta6_1 = pLast(5);
		else
			theta6_1 = atan2(k1, k2);
		double theta6;

		theta6 = calcRealAngle(pLast(5), theta6_1, theta6_1);
		// the limit of theta6 according to the reference
		if((theta6 < tmin/180.0L*M_PI) || (theta6 > tmax/180.0L*M_PI))
		{
		    if (optimize)
		    {
                if  (theta6 > 0.0)
                {
                    theta6 -= 2*M_PI;
                }
                else
                {
                    theta6 += 2*M_PI;
                }
                if((theta6 < tmin/180.0L*M_PI) || (theta6 > tmax/180.0L*M_PI))
                {
                    optimize = false;
                }
		    }
		    if (!optimize)
		    {
				if(fabs(d5) < 10e-13)
				{
			    	theta6 = pLast(5);
	                printf("theta6 exceeds pos limit.\n");
	                jointLimitNum = 6;
	                return INVERSE_KIN_NOT_REACHABLE;
				}
		    }
		}
		posACS(5) = theta6;
		if (robotDynamicLimit(posACS, true))
		{
		    jointLimitNum = 23;
		    return INVERSE_KIN_NOT_REACHABLE;
		}
	}
	else if (rbt_type == R_GENERAL_5S)
	{
		//solve for theta6
		double k1_4=sin(theta4)*cos(theta2+ M_PI/2.0+theta3)*cos(theta1)-cos(theta4)*sin(theta1);
		double k1_5=sin(theta4)*cos(theta2+ M_PI/2.0+theta3)*sin(theta1)+cos(theta4)*cos(theta1);
		double k1_6=sin(theta4)*sin(theta2+ M_PI/2.0+theta3);
		 k2=-k1_4*ox - k1_5*oy - k1_6*oz;
		 k1=-k1_4*nx - k1_5*ny - k1_6*nz;

		double theta6_1;
		if((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
			theta6_1 = sixAxisAngleFor5S;
		else
			theta6_1 = atan2(k1, k2);
		double theta6;

		theta6 = calcRealAngle(sixAxisAngleFor5S, theta6_1, theta6_1);
		// the limit of theta6 according to the reference
//		if((theta6 < tmin/180.0L*M_PI) || (theta6 > tmax/180.0L*M_PI))
//		{
//			theta6 = pLast(5);
//			printf("theta6 exceeds pos limit.\n");
//			jointLimitNum = 6;
//			return INVERSE_KIN_NOT_REACHABLE;
//		}
		sixAxisAngleFor5S = theta6;
	}
	return RBTINTERF_NO_ERROR;
}

ErrorID RobotModel::calcInverseKin_Trans_6s_1(const Matrix4& transMatrix,
		const Position_ACS_rad& posLast, Position_ACS_rad& posACS , bool optimize) const
{

	if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
				!=6)||(posACS.rows() !=6) ||(axes.size()!=6))
	{
		return INVERSE_KIN_TRANS_ARG_ERROR;
	}

	Matrix4 T(transMatrix);
	T *= toolMatrix.inverse();

	Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

	double nx(T(0,0)), ny(T(1,0)), nz(T(2,0));
	double ox(T(0,1)), oy(T(1,1)), oz(T(2,1));
	double ax(T(0,2)), ay(T(1,2)), az(T(2,2));
	double px(T(0,3)/1000.0), py(T(1,3)/1000.0), pz(T(2,3)/1000.0);

	//vars defined temporarily
	double l1=axes[0]->DHJ.D/1000;
	double l2=axes[1]->DHJ.A/1000;
	double l3=axes[2]->DHJ.A/1000;
	double l4=axes[4]->DHJ.D/1000;
	double l5=axes[5]->DHJ.D/1000;
	double l6=axes[0]->DHJ.A/1000;
	double l7=axes[1]->DHJ.D/1000;
	double theta2_offset = axes[1]->DHJ.Theta;
	double theta3_offset = axes[2]->DHJ.Theta;
	double theta5_offset = axes[4]->DHJ.Theta;
	double smax,smin,lmax,lmin,umax,umin,rmax,rmin,bmax,bmin,tmax,tmin;
    //第一轴角度限制范围
    smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
    smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
    //第二轴角度限制范围
    lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
    lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
    //第三轴角度限制范围
    umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
    umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
    //第四轴角度限制范围
    rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
    rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);
    //第五轴角度限制范围
    bmax=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.posSWLimit):(-1 * axes[4]->paramLmt.negSWLimit);
    bmin=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.negSWLimit):(-1 * axes[4]->paramLmt.posSWLimit);
    //第六轴角度限制范围
    tmax=(axes[5]->axisDirection == 1)?(axes[5]->paramLmt.posSWLimit):(-1 * axes[5]->paramLmt.negSWLimit);
    tmin=(axes[5]->axisDirection == 1)?(axes[5]->paramLmt.negSWLimit):(-1 * axes[5]->paramLmt.posSWLimit);

	//solve for theta1
	double theta1_1,theta1_2;
	double k1,k2;
	double theta1;
	k1 = px - l5 * ax;
	k2 = l5 * ay - py;
	
	double temp = pow(k1,2) + pow(k2,2) - pow(l7,2);
	if(temp < 0.0L)
	{
		printf("Theta1 can not be solved, so can not reach the point!\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	theta1_1 = atan2(l7,sqrt(temp)) - atan2(k2,k1);
	theta1_2 = atan2(l7,-sqrt(temp)) - atan2(k2,k1);
	theta1 = calcRealAngle(pLast(0), theta1_1, theta1_2);

//	printf("theta1 %f\n",theta1);
	_Optimal:
	// the limit of theta1 according to the reference
	if((theta1 < M_PI*smin/180.0L) || (theta1 > M_PI*smax/180.0L))
	{
		theta1 = pLast(0);
		printf("theta1 exceeds pos limit.\n");
		if (optimize == true)
		{
		    optimize = false;
		    theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
		    goto _Optimal;
		}
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(0) = theta1;

	//solve for theta3
	k1 =  pz - l1 - l5*az;
	k2 = cos(theta1)*px+sin(theta1)*py-l6-l5*(cos(theta1)*ax+sin(theta1)*ay);
	double k3 = pow(az,2) + pow((ax*cos(theta1)+ay*sin(theta1)),2);
	if (fabs(k3)<10e-5)
	{
		printf("theta3 exceeds pos limit.5axis==90\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	double k3_1 = sqrt(k3);
	double k3_2 = -sqrt(k3);

    double theta2;
    double theta4;
    double theta3,theta3_1,theta3_2,theta3_3,theta3_4,theta3_5,theta3_6;
    double theta3Cache[4];

    k3 = (pow((k1+l4*(cos(theta1)*ax+sin(theta1)*ay)/(-k3_1)), 2) + pow((k2-l4*az/(-k3_1)), 2) - pow(l2, 2) - pow(l3, 2))/(2*l2*l3);
    int  theta3CacheNum = 0;
    if (fabs(k3)<=1)
    {
        theta3_1 = atan2(sqrt(1-pow(k3,2)), k3) - theta3_offset;
        theta3_2 = -atan2(sqrt(1-pow(k3,2)), k3) - theta3_offset;
//        printf("31111=%f,%f\n",theta3_1/M_PI * 180,theta3_2/M_PI * 180);
        theta3_5 = calcRealAngle(pLast(2), theta3_1, theta3_2);
        theta3Cache[0] = theta3_5;
        theta3Cache[1] = theta3_5;
        theta3CacheNum = 1;

        k3 = (pow((k1+l4*(cos(theta1)*ax+sin(theta1)*ay)/(-k3_2)), 2) + pow((k2-l4*az/(-k3_2)), 2) - pow(l2, 2) - pow(l3, 2))/(2*l2*l3);
        if (fabs(k3)<=1)
        {
            theta3_3 = atan2(sqrt(1-pow(k3,2)), k3) - theta3_offset;
            theta3_4 = -atan2(sqrt(1-pow(k3,2)), k3) - theta3_offset;
//            printf("32222=%f,%f\n",theta3_3/M_PI * 180,theta3_4/M_PI * 180);
            theta3_6 = calcRealAngle(pLast(2), theta3_3, theta3_4);
            theta3Cache[2] = theta3_6;
            theta3Cache[3] = theta3_6;
            theta3CacheNum = 2;
//            theta3 = calcRealAngle(pLast(2), theta3_5, theta3_6);
        }
//        else
//        {
//            theta3 = theta3_5;
//        }
    }
    else
    {
        k3 = (pow((k1+l4*(cos(theta1)*ax+sin(theta1)*ay)/(-k3_2)), 2) + pow((k2-l4*az/(-k3_2)), 2) - pow(l2, 2) - pow(l3, 2))/(2*l2*l3);
        if (fabs(k3)<=1)
        {
            theta3_3 = atan2(sqrt(1-pow(k3,2)), k3) - theta3_offset;
            theta3_4 = -atan2(sqrt(1-pow(k3,2)), k3) - theta3_offset;
            theta3 = calcRealAngle(pLast(2), theta3_3, theta3_4);
            theta3Cache[0] = theta3;
            theta3Cache[1] = theta3;
            theta3CacheNum = 1;
        }
        else
        {
            printf("theta3 exceeds pos limit.5axis==90\n");
            if (optimize == true)
            {
                optimize = false;
                theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
                goto _Optimal;
            }
            jointLimitNum = 3;
            return INVERSE_KIN_NOT_REACHABLE;
        }
    }
    double theta2Cache[4];
    double theta4Cache[4];

    for(int i = 0; i < theta3CacheNum; i++)
    {
         k3 = l3 + l2 * cos(theta3Cache[2*i] + theta3_offset);
         double k4 = l2 * sin(theta3Cache[2*i] + theta3_offset);
         double theta2_1, theta2_2;
         double k5 = 2*k1*k4-2*k2*k3;
         double k6 = 2*k1*k3+2*k2*k4;
         double k7 = pow(l4,2) - pow(k1,2) - pow(k2,2) - pow(k3,2) - pow(k4,2);

         temp = pow(k5,2) + pow(k6,2) - pow(k7,2);
         if(temp < 0.0L)
           {
             theta2Cache[2*i] = 999;
             theta2Cache[2*i+1] = 999;

             theta4Cache[2*i] = 999;
             theta4Cache[2*i+1] = 999;
           }
         else
         {
           theta2_1 = -atan2(k7,sqrt(temp))+atan2(k5,k6) - theta3Cache[2*i] - theta2_offset - theta3_offset;
           theta2_2 = -atan2(k7,-sqrt(temp))+atan2(k5,k6) - theta3Cache[2*i] - theta2_offset - theta3_offset;
           theta2_1 = calcRealAngle(pLast(1), theta2_1, theta2_1);
           theta2_2 = calcRealAngle(pLast(1), theta2_2, theta2_2);

           theta2Cache[2*i] = theta2_1;
           {
               //solve for theta4
             double k8 = k1*sin(theta2_1+theta3Cache[2*i] + theta2_offset + theta3_offset)+k2*cos(theta2_1+theta3Cache[2*i] + theta2_offset + theta3_offset)-k3;
             double k9 = -k1*cos(theta2_1+theta3Cache[2*i] + theta2_offset + theta3_offset)+k2*sin(theta2_1+theta3Cache[2*i] + theta2_offset + theta3_offset)-k4;
             double theta4_1;
               if((fabs(k8) < 10e-13)  && (fabs(k9) < 10e-13))
                   theta4_1 = pLast(3);
               else
                   theta4_1 = atan2(k8,k9)  - M_PI/2.0;
               theta4_1 = calcRealAngle(pLast(3), theta4_1, theta4_1);
               theta4Cache[2*i] = theta4_1;
           }
           theta2Cache[2*i+1] = theta2_2;
           {
               //solve for theta4
             double k8 = k1*sin(theta2_2+theta3Cache[2*i] + theta2_offset + theta3_offset)+k2*cos(theta2_2+theta3Cache[2*i] + theta2_offset + theta3_offset)-k3;
             double k9 = -k1*cos(theta2_2+theta3Cache[2*i] + theta2_offset + theta3_offset)+k2*sin(theta2_2+theta3Cache[2*i] + theta2_offset + theta3_offset)-k4;
             double theta4_1;
               if((fabs(k8) < 10e-13)  && (fabs(k9) < 10e-13))
                   theta4_1 = pLast(3);
               else
                   theta4_1 = atan2(k8,k9)  - M_PI/2.0;
               theta4_1 = calcRealAngle(pLast(3), theta4_1, theta4_1);
               theta4Cache[2*i+1] = theta4_1;
           }
         }
//          printf("2=%f,%f\n",theta2_1/M_PI * 180,theta2_2/M_PI * 180);
    }
    double length = 0;
    int selectNum = -1;
    for(int i = 0; i < 2*theta3CacheNum; i++)
    {
        double temp1 =  (cos(theta1)*ax+sin(theta1)*ay)*sin(theta2Cache[i] + theta3Cache[i] + theta4Cache[i] + M_PI/2.0 + theta2_offset + theta3_offset)-az*cos(theta2Cache[i] + theta3Cache[i] + theta4Cache[i] + M_PI/2.0 + theta2_offset + theta3_offset);
        if (fabs(temp1) < 10e-7)
        {
            if (selectNum == -1)
            {
                length = fabs(theta2Cache[i] - pLast(1)) + fabs(theta3Cache[i] - pLast(2)) + fabs(theta4Cache[i] - pLast(3));
                selectNum = i;
            }
            else
            {
                if (length > fabs(theta2Cache[i] - pLast(1)) + fabs(theta3Cache[i] - pLast(2)) + fabs(theta4Cache[i] - pLast(3)))
                {
                    length = fabs(theta2Cache[i] - pLast(1)) + fabs(theta3Cache[i] - pLast(2)) + fabs(theta4Cache[i] - pLast(3));
                    selectNum = i;
                }
            }
        }
    }
    if (selectNum == -1)
    {
        if (optimize == true)
        {
            optimize = false;
            theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
            goto _Optimal;
        }
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    theta2 = theta2Cache[selectNum];

    theta3 = theta3Cache[selectNum];
    theta4 = theta4Cache[selectNum];

//    printf("33333333333333=%f,%f,%f\n",theta2/M_PI * 180,theta3/M_PI * 180,theta4/M_PI * 180);

    // the limit of theta3 according to the reference
    if((theta3 < umin/180.0L*M_PI) || (theta3 > umax/180.0L*M_PI))
    {
        theta3 = pLast(2);
        printf("theta3 exceeds pos limit.\n");
        if (optimize == true)
        {
            optimize = false;
            theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
            goto _Optimal;
        }
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(2) = theta3;

        // the limit of theta2 according to the reference
        if((theta2 < lmin/180.0L*M_PI) || (theta2 > lmax/180.0L*M_PI))
        {
            theta2 = pLast(1);
            printf("theta2 exceeds pos limit.\n");
            if (optimize == true)
            {
                optimize = false;
                theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
                goto _Optimal;
            }
            jointLimitNum = 2;
            return INVERSE_KIN_NOT_REACHABLE;
        }

        posACS(1) = theta2;
        // the limit of theta4 according to the reference
        if((theta4 < rmin*M_PI/180.0L) || (theta4 > rmax*M_PI/180.0L))
        {
            theta4 = pLast(3);
            printf("theta4 exceeds pos limit.\n");
            if (optimize == true)
            {
                optimize = false;
                theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
                goto _Optimal;
            }
            jointLimitNum = 4;
            return INVERSE_KIN_NOT_REACHABLE;
        }
        posACS(3) = theta4;

//    printf("theta2 = %lf,theta2_1 =%lf,theta2_1=%lf,ax=%lf,ay=%lf,az=%lf,temp=%lf,temp1=%.10lf,temp2=%.10lf\n",theta2,theta2_1,theta2_2,ax,ay,az,atan2(k7,sqrt(temp)),temp1,temp2);
    //solve for theta5
	double k1_1 = az*sin(theta2 + theta3 + theta4 + M_PI/2.0 + theta2_offset + theta3_offset) + ax*cos(theta1)*cos(theta2 + theta3 + theta4 + M_PI/2.0 + theta2_offset + theta3_offset)
			      + ay*sin(theta1)*cos(theta2 + theta3 + theta4 + M_PI/2.0 + theta2_offset + theta3_offset);
	double k1_2 = ax*sin(theta1)-ay*cos(theta1);

	double theta5_1;
	if ((fabs(k1_1) < 10e-13) && (fabs(k1_2) < 10e-13))
	{
		theta5_1 = pLast(4);
	}
	else
	{
		theta5_1 = atan2(-k1_1, k1_2) - theta5_offset;
	}
	double theta5;
	
	theta5 = calcRealAngle(pLast(4), theta5_1, theta5_1);	
	// the limit of theta5 according to the reference
	if((theta5 < bmin/180.0L*M_PI) || (theta5 > bmax/180.0L*M_PI))
	{
		theta5 = pLast(4);
		printf("theta5 exceeds pos limit.\n");
        if (optimize == true)
        {
            optimize = false;
            theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
            goto _Optimal;
        }
		jointLimitNum = 5;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(4) = theta5; 
		
	//solve for theta6
	double k1_3=nz*cos(theta2 + theta3 + theta4 + M_PI/2.0 + theta2_offset + theta3_offset) - nx*cos(theta1)*sin(theta2 + theta3 + theta4 + M_PI/2.0 + theta2_offset + theta3_offset)
			    - ny*sin(theta1)*sin(theta2 + theta3 + theta4 + M_PI/2.0 + theta2_offset + theta3_offset);
	double k1_4=oz*cos(theta2 + theta3 + theta4 + M_PI/2.0 + theta2_offset + theta3_offset) - ox*cos(theta1)*sin(theta2 + theta3 + theta4 + M_PI/2.0 + theta2_offset + theta3_offset)
				- oy*sin(theta1)*sin(theta2 + theta3 + theta4 + M_PI/2.0 + theta2_offset + theta3_offset);

	double theta6_1;
	if((fabs(k1_3) < 10e-13) && (fabs(k1_4) < 10e-13))
		theta6_1 = pLast(5);
	else
		theta6_1 = atan2(k1_3, k1_4);
	double theta6;
	
	theta6 = calcRealAngle(pLast(5), theta6_1, theta6_1);
	// the limit of theta6 according to the reference
	if((theta6 < tmin/180.0L*M_PI) || (theta6 > tmax/180.0L*M_PI))
	{
		theta6 = pLast(5);
		printf("theta6 exceeds pos limit.\n");
        if (optimize == true)
        {
            optimize = false;
            theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
            goto _Optimal;
        }
		jointLimitNum = 6;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(5) = theta6;

	return RBTINTERF_NO_ERROR;
}

ErrorID RobotModel::calcInverseKin_Trans_6s_2(const Matrix4& transMatrix,
		const Position_ACS_rad& posLast, Position_ACS_rad& posACS , CalculateParameter& calculateParameter, bool optimize) const
{

	if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
				!=6)||(posACS.rows() !=6) ||(axes.size()!=6))
	{
		return INVERSE_KIN_TRANS_ARG_ERROR;
	}

	Matrix4 T(transMatrix);
	T *= toolMatrix.inverse();

	Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

	double nx(T(0,0)), ny(T(1,0)), nz(T(2,0));
	double ox(T(0,1)), oy(T(1,1)), oz(T(2,1));
	double ax(T(0,2)), ay(T(1,2)), az(T(2,2));
	double px(T(0,3)/1000.0), py(T(1,3)/1000.0), pz(T(2,3)/1000.0);

	//vars defined temporarily
	double l1=axes[0]->DHJ.A/1000;
	double l2=axes[0]->DHJ.D/1000;
	double l3=axes[1]->DHJ.A/1000;
	double l4=axes[2]->DHJ.A/1000;
	double l5=axes[3]->DHJ.D/1000;
	double l6=axes[5]->DHJ.D/1000;

	double smax,smin,lmax,lmin,umax,umin,rmax,rmin,bmax,bmin,tmax,tmin;
    //第一轴角度限制范围
    smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
    smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
    //第二轴角度限制范围
    lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
    lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
    //第三轴角度限制范围
    umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
    umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
    //第四轴角度限制范围
    rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
    rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);
    //第五轴角度限制范围
    bmax=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.posSWLimit):(-1 * axes[4]->paramLmt.negSWLimit);
    bmin=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.negSWLimit):(-1 * axes[4]->paramLmt.posSWLimit);
    //第六轴角度限制范围
    tmax=(axes[5]->axisDirection == 1)?(axes[5]->paramLmt.posSWLimit):(-1 * axes[5]->paramLmt.negSWLimit);
    tmin=(axes[5]->axisDirection == 1)?(axes[5]->paramLmt.negSWLimit):(-1 * axes[5]->paramLmt.posSWLimit);

	//solve for theta3
	double theta3_1,theta3_2;
	double k1;
	double theta3;
	k1 = l6*az + l2 - pz;

	double temp = pow(l4,2) + pow(l5,2) - pow(k1,2);
	if(temp < 0.0L)
	{
		printf("Theta3 can not be solved, so can not reach the point!\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	theta3_1 = atan2(k1,sqrt(temp)) - atan2(l5,l4) + M_PI/2.0;
	theta3_2 = atan2(k1,-sqrt(temp)) - atan2(l5,l4)+ M_PI/2.0;
	theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);

	// the limit of theta3 according to the reference
	if((theta3 < M_PI*umin/180.0L) || (theta3 > M_PI*umax/180.0L))
	{
		theta3 = pLast(2);
		printf("theta3 exceeds pos limit.\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(2) = theta3;

	//solve for theta2
	double theta2_1,theta2_2;
	double theta2;
	double k2,k3,k4;
	k2 =  px  - l6*ax;
	k3 =  py  - l6*ay;
	k4 = l4*cos(theta3 - M_PI/2.0) - l5*sin(theta3 - M_PI/2.0) + l3;
	temp = (pow(k2,2) + pow(k3,2) - pow(k4,2) - pow(l1,2)) / 2 / l1 / k4;

	if (fabs(temp)>1.0)
	{
		printf("theta2 exceeds pos lim\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	theta2_1 = atan2(sqrt(1-pow(temp,2)), temp);
	theta2_2 = atan2(-sqrt(1-pow(temp,2)), temp);

	if (calculateParameter.configuration == 2)
	{
	    theta2 = (theta2_1 > 0.0)?(theta2_1):(theta2_2);
	}
	else if (calculateParameter.configuration == 1)
	{
	    theta2 = (theta2_1 < 0.0)?(theta2_1):(theta2_2);
	}
	else
	{
	    theta2 = calcRealAngle(pLast(1), theta2_1, theta2_2);
	}

	if((theta2 < M_PI*lmin/180.0L) || (theta2 > M_PI*lmax/180.0L))
	{
		theta2 = pLast(1);
		printf("theta2 exceeds pos limit.\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(1) = theta2;


	//solve for theta1
	double theta1_1,theta1_2;
	double theta1;
	double k1_1 = k4 + l1*cos(theta2);
	double k1_2 = pow(k2,2) + pow(k3,2) - pow(k1_1,2);
	theta1_1 = atan2(k1_1, sqrt(k1_2)) - atan2(k2,k3)-theta2;
	theta1_2 = atan2(k1_1, -sqrt(k1_2)) - atan2(k2,k3)-theta2;
	theta1 = calcRealAngle(pLast(0), theta1_1, theta1_2);
    if((theta1 < smin/180.0L*M_PI) || (theta1 > smax/180.0L*M_PI))
    {
        theta1 = pLast(0);
        printf("theta1 exceeds pos limit.\n");

        jointLimitNum = 1;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(0) = theta1;

    double k4_1 = sin(theta1 + theta2)*ax - cos(theta1 + theta2)*ay;
	double k4_2 = cos(theta1 + theta2)*cos(theta3 - M_PI/2.0)*ax + sin(theta1 + theta2)*
			cos(theta3 - M_PI/2.0)*ay - sin(theta3 - M_PI/2.0)*az;

	double theta4;
	//此处的判断阈值不能过小，过小的话，当0/0时，它无法识别出来

	if((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
	{
		theta4 = pLast(3);
		//cout << "A" << endl;
	}
	else
	{
		double theta4_1 = atan2(k4_1,k4_2);
		double theta4_2;
		if(theta4_1 > 0.0L)
			theta4_2 = theta4_1 - M_PI;
		else
			theta4_2 = theta4_1 + M_PI;
		theta4 = calcRealAngle(pLast(3), theta4_1, theta4_2);
	}

	// the limit of theta4 according to the reference
	if((theta4 < rmin*M_PI/180.0L) || (theta4 > rmax*M_PI/180.0L))
	{
		theta4 = pLast(3);
		printf("theta4 exceeds pos limit.\n");
		jointLimitNum = 4;
	}
	posACS(3) = theta4;

	//solve for theta5
	double k5_1 = sin(theta2)*sin(theta4) + cos(theta2)*cos(theta4)*cos(theta3 - M_PI/2.0);
	double k5_2 = cos(theta2)*sin(theta4) -sin(theta2)*cos(theta4)*cos(theta3 - M_PI/2.0);
	double k5_3 = k5_1*(ax * cos(theta1) + ay * sin(theta1)) + k5_2*(ax * sin(theta1) - ay * cos(theta1)) - cos(theta4)*sin(theta3 - M_PI/2.0)*az;
	double k5_4 = cos(theta1 + theta2) * sin(theta3 - M_PI/2.0)*ax + sin(theta1 + theta2) * sin(theta3 - M_PI/2.0)*	ay + cos(theta3 - M_PI/2.0)*az;
	double theta5_1;
	if ((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
	{
		theta5_1 = pLast(4);
	}
	else
	{
		theta5_1 = atan2(k5_3, k5_4) - M_PI / 2.0;
	}
	double theta5;

	theta5 = calcRealAngle(pLast(4), theta5_1, theta5_1);
	// the limit of theta5 according to the reference
	if((theta5 < bmin/180.0L*M_PI) || (theta5 > bmax/180.0L*M_PI))
	{
			theta5 = pLast(4);
			printf("theta5 exceeds pos limit.\n");
			jointLimitNum = 5;
			return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(4) = theta5;

	//solve for theta6
	double k6_1=cos(theta1)*(cos(theta2)*cos(theta4) + sin(theta2) * cos(theta3-M_PI/2.0) * sin(theta4)) - sin(theta1)*(sin(theta2)*cos(theta4) - cos(theta2) * cos(theta3-M_PI/2.0) * sin(theta4));
	double k6_2=cos(theta1)*(sin(theta2)*cos(theta4) - cos(theta2) * cos(theta3-M_PI/2.0) * sin(theta4)) + sin(theta1)*(cos(theta2)*cos(theta4) + sin(theta2) * cos(theta3-M_PI/2.0) * sin(theta4));
	double k6_3=sin(theta3-M_PI/2.0) * sin(theta4);
	double k6_4=ny*k6_1 - nx*k6_2 - nz*k6_3;
	double k6_5=oy*k6_1 - ox*k6_2 - oz*k6_3;

	double theta6_1;
	if((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
		theta6_1 = pLast(5);
	else
		theta6_1 = atan2(k6_4, k6_5);
	double theta6;

	theta6 = calcRealAngle(pLast(5), theta6_1, theta6_1);
	// the limit of theta6 according to the reference
	if((theta6 < tmin/180.0L*M_PI) || (theta6 > tmax/180.0L*M_PI))
	{
	    if (optimize)
	    {
            if  (theta6 > 0.0)
            {
                theta6 -= 2*M_PI;
            }
            else
            {
                theta6 += 2*M_PI;
            }
            if((theta6 < tmin/180.0L*M_PI) || (theta6 > tmax/180.0L*M_PI))
            {
                optimize = false;
            }
	    }
	    if (!optimize)
	    {
			theta6 = pLast(5);
			printf("theta6 exceeds pos limit.\n");
			jointLimitNum = 6;
			return INVERSE_KIN_NOT_REACHABLE;
	    }
	}
	posACS(5) = theta6;

	return RBTINTERF_NO_ERROR;
}
/*
 * @note  五轴协作机器人逆解  R_GENERAL_5S_COLLABORATIVE_
 * author: niupengshuai
 * date: 2024.03.15
 */
ErrorID RobotModel::calcInverseKin_Trans_GENERAL_5S_COLLABORATIVE(const Matrix4& transMatrix,
        const Position_ACS_rad& posLast, Position_ACS_rad& posACS , CalculateParameter& calculateParameter, bool optimize) const
{
    if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
            !=5)||(posACS.rows() !=5) ||(axes.size()!=5))
    {
        return INVERSE_KIN_TRANS_ARG_ERROR;
    }
    Matrix4 T(transMatrix);
    T *= toolMatrix.inverse();

    Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

    double NX(T(0,0)), NY(T(1,0)), NZ(T(2,0));
    double OX(T(0,1)), OY(T(1,1)), OZ(T(2,1));
    double AX(T(0,2)), AY(T(1,2)), AZ(T(2,2));
    double PX(T(0,3)/1000.0), PY(T(1,3)/1000.0), PZ(T(2,3)/1000.0);

    //vars defined temporarily
    double l1=axes[0]->DHJ.D/1000;
    double l2= - axes[1]->DHJ.D/1000;
    double l3=axes[1]->DHJ.A/1000;
    double l4=axes[2]->DHJ.D/1000;
    double l5=axes[2]->DHJ.A/1000;
    double l6= - axes[3]->DHJ.D/1000;
    double l7=axes[4]->DHJ.D/1000;

//     注意检查杆长的正负，下面算法中的杆长为正的,故上面L2和L6取反，DH参数页面这两个值填的负值

    double smax,smin,lmax,lmin,umax,umin,rmax,rmin,bmax,bmin,tmax,tmin;
    //第一轴角度限制范围
    smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
    smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
    //第二轴角度限制范围
    lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
    lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
    //第三轴角度限制范围
    umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
    umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
    //第四轴角度限制范围
    rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
    rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);
    //第五轴角度限制范围
    bmax=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.posSWLimit):(-1 * axes[4]->paramLmt.negSWLimit);
    bmin=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.negSWLimit):(-1 * axes[4]->paramLmt.posSWLimit);

    //solve for theta1
    double a1 = 0, b1 = 0, c1 = 0, d1 = 0,theta1_1 = 0, theta1_2 = 0 , theta1 = 0;
    a1 = - PY;
    b1 = PX;
    c1 = l4 - l2 - l6;
    d1 = pow(a1, 2) + pow(b1, 2) - pow(c1, 2);
    if((d1 < 0)&&(fabs(0.0 - d1) < 10e-13))
    {
        d1 = 0.0;
    }
    if(d1 < 0)
    {
        printf("Theta1 can not be solved, so can not reach the point!\n");
        jointLimitNum = 1;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    theta1_1 = atan2(c1, sqrt(d1)) - atan2(a1, b1);
//    theta1_2 = atan2(c1, - sqrt(pow(a1, 2) + pow(b1, 2) - pow(c1, 2))) - atan2(a1, b1); //一轴二轴平面后面的解，多数情况用不上
//    std::cout<<"theta1_1 = "<<theta1_1<<std::endl;
    theta1 = theta1_1;
    // the limit of theta1 according to the reference
    if((theta1 < M_PI*smin/180.0L) || (theta1 > M_PI*smax/180.0L))
    {
        if  (theta1 > 0.0)
        {
            theta1 -= 2*M_PI;
        }
        else
        {
            theta1 += 2*M_PI;
        }
        if((theta1 < M_PI*smin/180.0L) || (theta1 > M_PI*smax/180.0L))
        {
            theta1 = pLast(0);
            printf("theta1 exceeds pos limit.\n");
            jointLimitNum = 1;
            return INVERSE_KIN_NOT_REACHABLE;
        }
    }
    posACS(0) = theta1;

    //solve for theta5
    double a5 = 0, b5 = 0, theta5_1 = 0,theta5 = 0;
    a5 = NX * sin(theta1) - NY * cos(theta1);
    b5 = OX * sin(theta1) - OY * cos(theta1);
    theta5_1 = atan2(a5,b5);
    theta5 = calcRealAngle(pLast(4), theta5_1, theta5_1);
    // the limit of theta5 according to the reference
    if((theta5 < bmin/180.0L*M_PI) || (theta5 > bmax/180.0L*M_PI))
    {
        if (optimize)
        {
            if  (theta5 > 0.0)
            {
                theta5 -= 2*M_PI;
            }
            else
            {
                theta5 += 2*M_PI;
            }
            if((theta5 < bmin/180.0L*M_PI) || (theta5 > bmax/180.0L*M_PI))
            {
                optimize = false;
            }
        }
        if (!optimize)
        {
            theta5 = pLast(4);
            printf("theta5 exceeds pos limit.\n");
            jointLimitNum = 5;
            return INVERSE_KIN_NOT_REACHABLE;
        }
    }
    posACS(4) = theta5;

    //solve for theta3
    double PXx = 0, PYy = 0, PZz = 0, k1 = 0 ,k2 = 0, b3 = 0,theta3_1 = 0, theta3_2 = 0,theta3 = 0;
    PXx = PX - AX*l7;
    PYy = PY - AY*l7;
    PZz = PZ - AZ*l7;
    k1 = PYy * sin(theta1) + PXx * cos(theta1);
    k2 = PZz - l1;
    b3 = (pow(k1, 2) + pow(k2, 2) - pow(l3, 2) - pow(l5, 2)) / (2 * l3 * l5);
    if((b3 > 1.0)&&(fabs(b3 - 1.0) < 10e-13))
    {
        b3 = 1.0;
    }
    else if((b3 < -1.0)&&(fabs(-1.0 - b3) < 10e-13))
    {
        b3 = -1.0;
    }
    if(b3 > 1)
    {
        printf("Theta3 can not be solved, so can not reach the point!\n");
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    theta3_1 = atan2(sqrt(1 - pow(b3, 2)), b3) + M_PI/2;  // +M_PI/2为零点位置的补偿
    theta3_2 = atan2(-sqrt(1 - pow(b3, 2)), b3) + M_PI/2;
    theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);
    if((theta3 < umin/180.0L*M_PI) || (theta3 > umax/180.0L*M_PI))
    {
        theta3 = pLast(2);
        printf("theta3 exceeds pos limit.\n");
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(2) = theta3;
    theta3 = theta3 - M_PI/2; //后面计算使用的theta3是补偿之前的，需要再去掉M_PI/2;

    //solve for theta2
    double a2 = 0, b2 = 0, c2 = 0,d2 = 0, theta2_1 = 0, theta2_2 = 0, theta2 = 0;
    a2 = l3 + l5 * cos(theta3);
    b2 = -l5 * sin(theta3);
    c2 =  PYy * sin(theta1) + PXx * cos(theta1);
    d2 = pow(a2, 2) + pow(b2, 2) - pow(c2, 2);
    if((d2 < 0)&&(fabs(0.0 - d2) < 10e-13))
    {
        d2 = 0.0;
    }
    if(d2 < 0)
    {
        printf("Theta2 can not be solved, so can not reach the point!\n");
        jointLimitNum = 2;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    theta2_1 = atan2(c2,sqrt(d2)) - atan2(a2,b2) - M_PI/2;// -M_PI/2为零点位置的补偿
    theta2_2 = atan2(c2,-sqrt(d2)) - atan2(a2,b2) - M_PI/2;
    if(PZz >= l1 ) //四轴和二轴同水平线时
    {
        theta2 = theta2_2;
    }
    else
    {
        theta2 = theta2_1;
    }
    if((theta2 < lmin/180.0L*M_PI) || (theta2 > lmax/180.0L*M_PI))
    {
        theta2 = pLast(1);
        printf("theta2 exceeds pos limit.\n");
        jointLimitNum = 2;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(1) = theta2;
    theta2 = theta2 + M_PI/2; //后面计算使用的theta2是补偿之前的，需要再加回M_PI/2;

    //solve for theta4
    long double m12 = 0, m7 = 0, m8 = 0, b4 = 0,theta4_1 = 0, theta4_2 = 0, theta4 = 0,J234_1 = 0, J234_2 = 0;
    m12 = AX * cos(theta1) + AY * sin(theta1);
    m7 = AZ * cos(theta2) - m12 * sin(theta2);
    m8 = m12 * cos(theta2) + AZ * sin(theta2);
    b4 = m8 * sin(theta3) - m7 * cos(theta3);

    if((b4 > 1.0)&&fabs(b4 - 1.0) < 10e-13)
    {
        b4 = 1.0;
    }
    else if((b4 < -1.0)&&(fabs(-1.0 - b4) < 10e-13))
    {
        b4 = -1.0;
    }

    if((pow(b4, 2)) > 1.0)
    {
        printf("Theta4 can not be solved, so can not reach the point!\n");
        jointLimitNum = 4;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    theta4_1 = atan2(sqrt(1 - pow(b4, 2)), b4);
    theta4_2 = atan2(-sqrt(1 - pow(b4, 2)), b4);
    J234_1 = theta2 + theta3 + theta4_1;
    J234_2 = theta2 + theta3 + theta4_2;

    double theta234 = atan2(AX*cos(theta1)+AY*sin(theta1),AZ);

    if((fabs(J234_1 + theta234 - M_PI) < 10e-5)||(fabs(J234_1 + theta234 + M_PI) < 10e-5))
    {
        theta4 = theta4_1;
    }
    else if((fabs(J234_2 + theta234 - M_PI) < 10e-5)||(fabs(J234_2 + theta234 + M_PI) < 10e-5))
    {
        theta4 = theta4_2;
    }
    else
    {
        theta4 = calcRealAngle(pLast(3), theta4_1, theta4_2);
    }

    if((theta4 < rmin*M_PI/180.0L) || (theta4 > rmax*M_PI/180.0L))
    {
        if (optimize)
        {
            if  (theta4 > 0.0)
            {
                theta4 -= 2*M_PI;
            }
            else
            {
                theta4 += 2*M_PI;
            }
            if((theta4 < rmin/180.0L*M_PI) || (theta4 > rmax/180.0L*M_PI))
            {
                optimize = false;
            }
        }
        if (!optimize)
        {
            theta4 = pLast(3);
            printf("theta4 exceeds pos limit.\n");
            jointLimitNum = 4;
            return INVERSE_KIN_NOT_REACHABLE;
        }
    }
    posACS(3) = theta4;

    return RBTINTERF_NO_ERROR;

}

ErrorID RobotModel::calcInverseKin_Trans_SCARA(const Matrix4& transMatrix,
		const Position_ACS_rad& posLast, Position_ACS_rad& posACS, CalculateParameter& calculateParameter , bool optimize) const
{
	double d4 = 0.0;
	double a1 = 0.0;
	double a2 = 0.0;
	double smax = 0.0;
    double smin = 0.0;
    double lmax = 0.0;
    double lmin = 0.0;
    double umax = 0.0;
    double umin = 0.0;
    double rmax  = 0.0;
    double rmin = 0.0;
	if (rbt_type == R_SCARA)
	{
		if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
					!=4)||(posACS.rows() !=4) ||(axes.size()!=4))
		{
			return INVERSE_KIN_TRANS_ARG_ERROR;
		}

		//vars defined temporarily
		d4=axes[3]->DHJ.D/1000;
		a1=axes[0]->DHJ.A/1000;
		a2=axes[1]->DHJ.A/1000;

        //第一轴角度限制范围
        smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
        smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
        //第二轴角度限制范围
        lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
        lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
        //第三轴角度限制范围
        umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
        umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
        //第四轴角度限制范围
        rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
        rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);
	}
	else if (rbt_type == R_SCARA_THREEAXIS)
    {
        if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
                    !=3)||(posACS.rows() !=3) ||(axes.size()!=3))
        {
            return INVERSE_KIN_TRANS_ARG_ERROR;
        }

        //vars defined temporarily
        d4=0;
        a1=axes[0]->DHJ.A/1000;
        a2=axes[1]->DHJ.A/1000;

        //第一轴角度限制范围
        smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
        smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
        //第二轴角度限制范围
        lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
        lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
        //第三轴角度限制范围
        umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
        umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
    }
	else if (rbt_type == R_SCARA_TWOAXIS)
	{
		if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
					!=2)||(posACS.rows() !=2) ||(axes.size()!=2))
		{
			return INVERSE_KIN_TRANS_ARG_ERROR;
		}

		//vars defined temporarily
		a1=axes[0]->DHJ.A/1000;
		a2=axes[1]->DHJ.A/1000;

        //第一轴角度限制范围
        smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
        smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
        //第二轴角度限制范围
        lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
        lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
	}

	Matrix4 T(transMatrix);
    double theta_tool;
    if(rbt_type == R_SCARA_THREEAXIS)
    {
        double tool_x = toolMatrix(0, 3)/1000;
        double tool_y = -toolMatrix(1, 3)/1000;
        theta_tool = atan2(tool_y, a2 + tool_x);
        a2 = sqrt(pow(a2 + tool_x, 2) + pow(tool_y, 2));
    }
    else
    {
        T *= toolMatrix.inverse();
        theta_tool = 0;
    }
	Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值
	double nx(T(0,0));
	double ox(T(0,1));
	double px(T(0,3)/1000.0), py(T(1,3)/1000.0), pz(T(2,3)/1000.0);

	//solve for theta2
	double k1 = pow(px,2) + pow(py,2) - pow(a1,2) - pow(a2,2);
	double k2 = k1/(2*a1*a2);
	double theta2;
	double theta2Large = 0;
	double theta2_1;
	double theta2_2;
	if (doubleHighPrecisionCompareEqu(k2, 1))
		k2 = 1;
	if (doubleHighPrecisionCompareEqu(k2, -1))
		k2 = -1;

	if(fabs(k2)>1)
	{
		printf("Theta2 can not be solved, so can not reach the point!\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	theta2_1 = atan2(sqrt(1-pow(k2,2)),k2);
	theta2_2 = atan2((-sqrt(1-pow(k2,2))),k2);
    if (calculateParameter.configuration == 2)
    {
        theta2 = (theta2_1 - theta_tool > 0.0) ? (theta2_1) : (theta2_2);
    }
    else if (calculateParameter.configuration == 1)
    {
        theta2 = (theta2_1 - theta_tool < 0.0) ? (theta2_1) : (theta2_2);
    }
    else
    {
        theta2 = calcRealAngle(pLast(1) + theta_tool, theta2_1, theta2_2);
        theta2Large = calcLargeAngle(pLast(1) + theta_tool, theta2_1, theta2_2);
    }

	// the limit of theta1 according to the reference
    if((theta2 - theta_tool < M_PI*lmin/180.0L) || (theta2 - theta_tool > M_PI*lmax/180.0L))
	{
		theta2 = pLast(1);
		printf("theta2 exceeds pos limit.\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}
    posACS(1) = theta2 - theta_tool;
	//solve for theta1
	double theta1;
	double theta1Large;
	double theta1_1;
	double theta1_2;
	if((fabs(px) < 10e-13) && (fabs(py) < 10e-13))
		theta1_1 = pLast(0);
	else
		theta1_1 = atan2(py,px) - atan2((a2*sin(theta2)),(a1+a2*cos(theta2)));
	theta1_2 = theta1_1;
	theta1 = calcRealAngle(pLast(0), theta1_1, theta1_2);

	if (!optimize)
	{
		if (calculateParameter.configuration == 0)
		{
			theta1Large = atan2(py,px) - atan2((a2*sin(theta2Large)),(a1+a2*cos(theta2Large)));
			theta1Large = calcRealAngle(pLast(0), theta1Large, theta1Large);
            double velChange = fabs(theta2 - theta_tool - pLast(1)) + fabs(theta1 - pLast(0));
            double velChangeLarge = fabs(theta2Large - theta_tool - pLast(1)) + fabs(theta1Large - pLast(0));

			if (velChange > velChangeLarge)
			{
				theta1 = theta1Large;
				theta2 = theta2Large;
                posACS(1) = theta2 - theta_tool;
			}
		}
	}
	// the limit of theta1 according to the reference
	if((theta1 < M_PI*smin/180.0L) || (theta1 > M_PI*smax/180.0L))
	{
		theta1 = pLast(0);
		printf("theta1 exceeds pos limit.\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(0) = theta1;

	if (rbt_type == R_SCARA || rbt_type == R_SCARA_THREEAXIS)
	{
	    if (rbt_type == R_SCARA)
	    {
            //solve for theta4
            double theta4;
            double theta4_1;
            double theta4_2;
            theta4_1 = + theta1 + theta2 + atan2(-ox, nx);
            theta4_2 = theta4_1;
            theta4 = calcRealAngle(pLast(3), theta4_1, theta4_2);
            // the limit of theta1 according to the reference
            if((theta4 < M_PI*rmin/180.0L) || (theta4 > M_PI*rmax/180.0L))
            {
                if (optimize)
               {
                   if  (theta4 > 0.0)
                   {
                       theta4 -= 2*M_PI;
                   }
                   else
                   {
                       theta4 += 2*M_PI;
                   }
                   if((theta4 < M_PI*rmin/180.0L) || (theta4 > M_PI*rmax/180.0L))
                   {
                       optimize = false;
                   }
               }
               if (!optimize)
               {
                   theta4 = pLast(3);
                   printf("theta4 exceeds pos limit.\n");
                   jointLimitNum = 4;
                   return INVERSE_KIN_NOT_REACHABLE;
               }
            }
            posACS(3) = theta4;
	    }
		//solve for theta3
		double theta3;
		double theta3_1;
		theta3_1 = (pz-d4)/pitch*1000*2*M_PI;
		theta3 = theta3_1;
//		printf("pz=%f,%f,%f",pz,theta3_1,theta3);
		// the limit of theta1 according to the reference
		if((theta3 < M_PI*umin/180.0L) || (theta3 > M_PI*umax/180.0L))
		{
			theta3 = pLast(2);
			printf("theta3 exceeds pos limit.\n");
			jointLimitNum = 3;
			return INVERSE_KIN_NOT_REACHABLE;
		}
		posACS(2) = theta3;
	}
	return RBTINTERF_NO_ERROR;
}

ErrorID RobotModel::calcInverseKin_Trans_SCARA_1(const Matrix4& transMatrix,
		const Position_ACS_rad& posLast, Position_ACS_rad& posACS, CalculateParameter& calculateParameter , bool optimize) const
{
	double d4,a2,a3;
	double smax,smin,lmax,lmin,umax,umin,rmax,rmin;
	if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
			!=4)||(posACS.rows() !=4) ||(axes.size()!=4))
	{
		return INVERSE_KIN_TRANS_ARG_ERROR;
	}

	//vars defined temporarily
	d4=axes[3]->DHJ.D/1000;
	a2=axes[1]->DHJ.A/1000;
	a3=axes[2]->DHJ.A/1000;

	//第一轴角度限制范围
	smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
	smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
	//第二轴角度限制范围
	lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
	lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
	//第三轴角度限制范围
	umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
	umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
	//第四轴角度限制范围
	rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
	rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);

	Matrix4 T(transMatrix);
	T *= toolMatrix.inverse();
	Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值
	double nx(T(0,0));
	double ox(T(0,1));
	double px(T(0,3)/1000.0), py(T(1,3)/1000.0), pz(T(2,3)/1000.0);

	//solve for theta2
	double k1 = pow(px,2) + pow(py,2) - pow(a2,2) - pow(a3,2);
	double k2 = k1/(2*a2*a3);
	double theta3;
	double theta3_1;
	double theta3_2;
	if (doubleHighPrecisionCompareEqu(k2, 1) || k2 > 1)
		k2 = 1;
	if(fabs(k2)>1)
	{
		printf("Theta3 can not be solved, so can not reach the point!\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	theta3_1 = atan2(sqrt(1-pow(k2,2)),k2);
	theta3_2 = atan2((-sqrt(1-pow(k2,2))),k2);
    if (calculateParameter.configuration == 2)
    {
        theta3 = (theta3_1 > 0.0) ? (theta3_1) : (theta3_2);
    }
    else if (calculateParameter.configuration == 1)
    {
        theta3 = (theta3_1 < 0.0) ? (theta3_1) : (theta3_2);
    }
    else
    {
        theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);
    }
	// the limit of theta1 according to the reference
	if((theta3 < M_PI * umin / 180.0L) || (theta3 > M_PI * umax / 180.0L))
	{
        theta3 = pLast(2);
        printf("theta3 exceeds pos limit.\n");
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(2) = theta3;

	//solve for theta1
	double theta2;
	double theta2_1;
	double theta2_2;
	if((fabs(px) < 10e-13) && (fabs(py) < 10e-13))
		theta2_1 = pLast(1);
	else
		theta2_1 = atan2(py,px) - atan2((a3*sin(theta3)),(a2+a3*cos(theta3)));

	theta2_2 = theta2_1;
	theta2 = calcRealAngle(pLast(1), theta2_1, theta2_2);
	// the limit of theta1 according to the reference
	if((theta2 < M_PI*lmin/180.0L) || (theta2 > M_PI*lmax/180.0L))
	{
		theta2 = pLast(1);
		printf("theta2 exceeds pos limit.\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(1) = theta2;
	//solve for theta4
	double theta4;
	double theta4_1;
	double theta4_2;
	theta4_1 = theta2 + theta3 + atan2(-ox, nx);
	theta4_2 = theta4_1;
	theta4 = calcRealAngle(pLast(3), theta4_1, theta4_2);
	// the limit of theta1 according to the reference
	if((theta4 < M_PI*rmin/180.0L) || (theta4 > M_PI*rmax/180.0L))
	{
		if (optimize)
		{
			if  (theta4 > 0.0)
			{
				theta4 -= 2*M_PI;
			}
			else
			{
				theta4 += 2*M_PI;
			}
			if((theta4 < M_PI*rmin/180.0L) || (theta4 > M_PI*rmax/180.0L))
			{
				optimize = false;
			}
		}
		if (!optimize)
		{
			theta4 = pLast(3);
			printf("theta4 exceeds pos limit.\n");
			jointLimitNum = 4;
			return INVERSE_KIN_NOT_REACHABLE;
		}
	}
	posACS(3) = theta4;
	//solve for theta3
	double theta1;
	double theta1_1;
	theta1_1 = (pz-d4)/pitch*1000*2*M_PI;
	theta1 = theta1_1;
//		printf("pz=%f,%f,%f",pz,theta3_1,theta3);
	// the limit of theta1 according to the reference
	if((theta1 < M_PI*smin/180.0L) || (theta1 > M_PI*smax/180.0L))
	{
		theta1 = pLast(0);
		printf("theta1 exceeds pos limit.\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(0) = theta1;
	return RBTINTERF_NO_ERROR;
}
/*
 * @note 晶圆机器人：scara异形1的机型，零点位置不同，3轴零点位置旋转180度，与2轴重合;
 */
ErrorID RobotModel::calcInverseKin_Trans_SCARA_3(const Matrix4& transMatrix, const Position_ACS_rad& posLast,
        Position_ACS_rad& posACS, CalculateParameter& calculateParameter, bool optimize) const
{
    double d1, a2, a3;
    double smax, smin, lmax, lmin, umax, umin, rmax, rmin;
    if ((transMatrix.rows() != 4) || (transMatrix.cols() != 4) || (posLast.rows() != 4) || (posACS.rows() != 4)
        || (axes.size() != 4))
    {
        return INVERSE_KIN_TRANS_ARG_ERROR;
    }

    //vars defined temporarily
    d1 = axes[0]->DHJ.D / 1000;
    a2 = axes[1]->DHJ.A / 1000;
    a3 = axes[2]->DHJ.A / 1000;

    //第一轴角度限制范围
    smax = (axes[0]->axisDirection == 1) ? (axes[0]->paramLmt.posSWLimit) : (-1 * axes[0]->paramLmt.negSWLimit);
    smin = (axes[0]->axisDirection == 1) ? (axes[0]->paramLmt.negSWLimit) : (-1 * axes[0]->paramLmt.posSWLimit);
    //第二轴角度限制范围
    lmax = (axes[1]->axisDirection == 1) ? (axes[1]->paramLmt.posSWLimit) : (-1 * axes[1]->paramLmt.negSWLimit);
    lmin = (axes[1]->axisDirection == 1) ? (axes[1]->paramLmt.negSWLimit) : (-1 * axes[1]->paramLmt.posSWLimit);
    //第三轴角度限制范围
    umax = (axes[2]->axisDirection == 1) ? (axes[2]->paramLmt.posSWLimit) : (-1 * axes[2]->paramLmt.negSWLimit);
    umin = (axes[2]->axisDirection == 1) ? (axes[2]->paramLmt.negSWLimit) : (-1 * axes[2]->paramLmt.posSWLimit);
    //第四轴角度限制范围
    rmax = (axes[3]->axisDirection == 1) ? (axes[3]->paramLmt.posSWLimit) : (-1 * axes[3]->paramLmt.negSWLimit);
    rmin = (axes[3]->axisDirection == 1) ? (axes[3]->paramLmt.negSWLimit) : (-1 * axes[3]->paramLmt.posSWLimit);

    Matrix4 T(transMatrix);
    T *= toolMatrix.inverse();
    Position_ACS_rad pLast(posLast);	//读到的角度，经下面处理后，成为theta值
    double nx(T(0, 0));
    double ox(T(0, 1));
    double px(T(0, 3) / 1000.0), py(T(1, 3) / 1000.0), pz(T(2, 3) / 1000.0);
    //solve for theta2
    double k1 = pow(a2, 2) + pow(a3, 2) - pow(px, 2) - pow(py, 2);
    double k2 = k1 / (2 * a2 * a3);
    double theta3;
    double theta3_1;
    double theta3_2;
    if (doubleHighPrecisionCompareEqu(k2, 1) || k2 > 1) k2 = 1;
    if (fabs(k2) > 1)
    {
        printf("Theta3 can not be solved, so can not reach the point!\n");
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
    }

    theta3_1 = atan2(sqrt(1 - pow(k2, 2)), k2);
    theta3_2 = atan2((-sqrt(1 - pow(k2, 2))), k2);
    if (calculateParameter.configuration == 1)
    {
        theta3 = (theta3_1 > 0.0) ? (theta3_1) : (theta3_2);
    }
    else if (calculateParameter.configuration == 2)
    {
        theta3 = (theta3_1 < 0.0) ? (theta3_1) : (theta3_2);
    }
    else
    {
        theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);
    }
    // the limit of theta1 according to the reference
    if ((theta3 < M_PI * umin / 180.0L) || (theta3 > M_PI * umax / 180.0L))
    {
        theta3 = pLast(2);
        printf("theta3 exceeds pos limit.\n");
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(2) = theta3;

    //solve for theta1
    double theta2;
    double theta2_1;
    double theta2_2;
    bool theta2_calc = false;
    //theta3角度较小时，晶圆机器人特殊需求：运动方向为2,3轴打开的时候，沿着等腰三角形的底边延长方向运动; (仅仅适应a2==a3 或 a2 > a3的情况)
    // (!optimize) 指连续运动模式 one-two < std::numeric_limits<double>::epsilon()
    if (!optimize && (fabs(pLast(2)) < 0.01 || fabs(theta3) < 0.01) && (a2 - a3 > -std::numeric_limits<double>::epsilon()))
    {
        //运动方向要求：应在当前位置(或目标位置)到 二三轴重合时末端位置的连线上
        Matrix4 last_mtx;
        calcTransMatrix(pLast, last_mtx);
        Vector3 dir_motion(T(0, 3) - last_mtx(0, 3), T(1, 3) - last_mtx(1, 3), 0);
        //2轴连杆方向
        Vector3 vec_l2(0, 1, 0);
        vec_l2 = Eigen::AngleAxisd(pLast(1), Vector3::UnitZ()) * vec_l2;

        Vector3 zero_pos = (a2 - a3) * 1000 * vec_l2; //二三轴重合时的末端位置
        Vector3 dir_from_zero_point(last_mtx(0, 3) - zero_pos[0], last_mtx(1, 3) - zero_pos[1], 0);
        if (dir_from_zero_point.norm() < std::numeric_limits<double>::epsilon())
        {
            dir_from_zero_point << T(0, 3) - zero_pos[0], T(1, 3) - zero_pos[1], 0;
        }

        if (dir_motion.norm() > std::numeric_limits<double>::epsilon()
                && dir_from_zero_point.norm() > std::numeric_limits<double>::epsilon()
                && fabs(fabs(dir_motion.dot(dir_from_zero_point)) - fabs(dir_motion.norm() * dir_from_zero_point.norm())) < std::numeric_limits<double>::epsilon())
        {
            //判断theta2和theta3应该加（还是 减）变化量: 根据运动方向与2轴连杆法向方向的夹角判断
            Vector3 vec_l2_normal = vec_l2.cross(Vector3::UnitZ());
            double theta3_change = fabs(theta3 - pLast(2));
            if (dir_motion.dot(vec_l2_normal) > -std::numeric_limits<double>::epsilon())
            {
                posACS(2) = theta3 = pLast(2) + theta3_change;
                theta2_1 = pLast(1) - 0.5 * theta3_change;
            }
            else
            {
                posACS(2) = theta3 = pLast(2) - theta3_change;
                theta2_1 = pLast(1) + 0.5 * theta3_change;
            }
            theta2_calc = true;
        }
    }
    if (!theta2_calc)
    {
        if ((fabs(px) < 10e-13) && (fabs(py) < 10e-13)) theta2_1 = pLast(1);
        else theta2_1 = atan2((a3 * sin(theta3)), (a2 - a3 * cos(theta3))) - atan2(px, py);
    }
    theta2_2 = theta2_1;
    theta2 = calcRealAngle(pLast(1), theta2_1, theta2_2);

    // the limit of theta1 according to the reference
    if ((theta2 < M_PI * lmin / 180.0L) || (theta2 > M_PI * lmax / 180.0L))
    {
        theta2 = pLast(1);
        printf("theta2 exceeds pos limit.\n");
        jointLimitNum = 2;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(1) = theta2;
    //solve for theta4
    double theta4;
    double theta4_1;
    double theta4_2;
    theta4_1 = atan2(-ox, nx) - theta2 - theta3 + 0.5 * M_PI;
    theta4_2 = theta4_1;
    theta4 = calcRealAngle(pLast(3), theta4_1, theta4_2);

    // the limit of theta1 according to the reference
    if ((theta4 < M_PI * rmin / 180.0L) || (theta4 > M_PI * rmax / 180.0L))
    {
        if (optimize)
        {
            if (theta4 > 0.0)
            {
                theta4 -= 2 * M_PI;
            }
            else
            {
                theta4 += 2 * M_PI;
            }
            if ((theta4 < M_PI * rmin / 180.0L) || (theta4 > M_PI * rmax / 180.0L))
            {
                optimize = false;
            }
        }
        if (!optimize)
        {
            theta4 = pLast(3);
            printf("theta4 exceeds pos limit.\n");
            jointLimitNum = 4;
            return INVERSE_KIN_NOT_REACHABLE;
        }
    }
    posACS(3) = theta4;
    //solve for theta3
    double theta1;
    double theta1_1;
    theta1_1 = (pz - d1) / pitch * 1000 * 2 * M_PI;
    theta1 = theta1_1;
    // the limit of theta1 according to the reference
    if ((theta1 < M_PI * smin / 180.0L) || (theta1 > M_PI * smax / 180.0L))
    {
        theta1 = pLast(0);
        printf("theta1 exceeds pos limit.\n");
        jointLimitNum = 1;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(0) = theta1;
    return RBTINTERF_NO_ERROR;
}
/**
 * @note 四轴码垛机器人正解
 */
ErrorID RobotModel::calcForwardKin_FourAxis_StackingRobot(const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) const
{
	if ( (posACS.rows() !=4)||(posMCS.rows() !=6) ||(axes.size()!=4))
	{
		return FORWARD_KIN_ARG_ERROR;
	}

	//vars defined temporarily
	double d1=axes[0]->DHJ.A;
	double d2=axes[1]->DHJ.A;
	double d3=axes[2]->DHJ.A;
	double d4=axes[3]->DHJ.A;
	double d5=axes[3]->DHJ.D;
	double d6=axes[2]->DHJ.D;

	double theta1 = posACS(0);
	double theta2 = M_PI/2 + posACS(1);
	double theta3 = M_PI/2 - posACS(2);
	double theta4 = posACS(3);

	posMCS[0] = (d1+(d2*cos(theta2)+d3*cos(theta3-theta2))+d4)*cos(theta1) + d6 * sin(theta1);
	posMCS[1] = (d1+(d2*cos(theta2)+d3*cos(theta3-theta2))+d4)*sin(theta1) - d6 * cos(theta1);
	posMCS[2] = d2*sin(theta2)-d3*sin(theta3-theta2)-d5;
	posMCS[3] = M_PI;
	posMCS[4] = 0;
	posMCS[5] = theta4 - theta1;

	return RBTINTERF_NO_ERROR;
}

/**
 * @note 四轴码垛机器人逆解
 */
ErrorID RobotModel::calcInverseKin_Trans_FourAxis_StackingRobot(const Matrix4& transMatrix,
		const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const
{
	if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
				!=4)||(posACS.rows() !=4) ||(axes.size()!=4))
	{
		return INVERSE_KIN_TRANS_ARG_ERROR;
	}
	Matrix4 T(transMatrix);
	T *= toolMatrix.inverse();
	Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

	double nx(T(0,0));
	double ox(T(0,1));
	double px(T(0,3)/1000.0), py(T(1,3)/1000.0), pz(T(2,3)/1000.0);

	//vars defined temporarily
	double d1=axes[0]->DHJ.A/1000;
	double d2=axes[1]->DHJ.A/1000;
	double d3=axes[2]->DHJ.A/1000;
	double d4=axes[3]->DHJ.A/1000;
	double d5=axes[3]->DHJ.D/1000;
	double d6=axes[2]->DHJ.D/1000;
	double smax,smin,lmax,lmin,umax,umin,rmax,rmin;
    //第一轴角度限制范围
    smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
    smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
    //第二轴角度限制范围
    lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
    lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
    //第三轴角度限制范围
    umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
    umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
    //第四轴角度限制范围
    rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
    rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);

	//solve for theta1
    double k0 = sqrt(pow(px,2) + pow(py,2) - pow(d6,2));
	double k1 = px * k0 - d6 * py;
	double k2 = py * k0 + d6 * px;
	double theta1;

	theta1 = atan2(k2,k1);
	if((theta1 < M_PI*smin/180.0L) || (theta1 > M_PI*smax/180.0L))
	{
		theta1 = pLast(0);
		printf("theta1 exceeds pos limit.\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(0) = theta1;

	//solve for theta4
	double theta4,theta4_1,theta4_2;
	theta4_1 = theta1 + atan2(-ox, nx);
	theta4_2 = theta4_1;

	theta4 = calcRealAngle(pLast(3), theta4_1, theta4_2);
	// the limit of theta1 according to the reference
	if((theta4 < M_PI*rmin/180.0L) || (theta4 > M_PI*rmax/180.0L))
	{
		theta4 = pLast(3);
		printf("theta4 exceeds pos limit.\n");
		jointLimitNum = 4;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(3) = theta4;

	//solve for theta3
	double k3,k4,k5;
	double theta3;
	double theta3_1;
	double theta3_2;
	if(fabs(sin(theta1)) < 10e-3)
	{
		k3 = (px - d6 * sin(theta1)) / cos(theta1) - d1 - d4;
	}
	else
	{
		k3 = (py + d6 * cos(theta1)) / sin(theta1) - d1 - d4;
	}
	k4 = pz + d5;
	k5 = (pow(k3,2) + pow(k4,2) - pow(d2,2) - pow(d3,2))/(2*d2*d3);

	if(fabs(k5)>1)
	{
		printf("Theta3 can not be solved, so can not reach the point!\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	theta3_1 = M_PI / 2 - atan2(sqrt(1-pow(k5,2)),k5);
	theta3_2 = M_PI / 2 - atan2((-sqrt(1-pow(k5,2))),k5);

	theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);

	if((theta3 < M_PI*umin/180.0L) || (theta3 > M_PI*umax/180.0L))
	{
		theta3 = pLast(2);
		printf("theta3 exceeds pos limit.\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(2) = theta3;

	//solve for theta2
	double theta2;
	double k6,k7;
	k6 = k3*(d2+d3*cos(M_PI / 2 - theta3)) - d3*k4*sin(M_PI / 2 - theta3);
	k7 = k4*(d2+d3*cos(M_PI / 2 - theta3)) + k3*d3*sin(M_PI / 2 - theta3);
	if((fabs(k6) < 10e-13) && (fabs(k7) < 10e-13))
	{
		theta2 = pLast(1);
	}
	else
	{
		theta2 = atan2(k7,k6) - M_PI / 2;
	}
	if((theta2 < M_PI*lmin/180.0L) || (theta2 > M_PI*lmax/180.0L))
	{
		theta2 = pLast(1);
		printf("theta2 exceeds pos limit.\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(1) = theta2;

	if (robotDynamicLimit(posACS, true))
	{
	    jointLimitNum = 23;
	    return INVERSE_KIN_NOT_REACHABLE;
	}
	return RBTINTERF_NO_ERROR;
}

/**
 * @note 四轴码垛机器人异形1正解
 */
ErrorID RobotModel::calcForwardKin_FourAxis_StackingRobot_1(const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) const
{
    if ( (posACS.rows() !=4)||(posMCS.rows() !=6) ||(axes.size()!=4))
    {
        return FORWARD_KIN_ARG_ERROR;
    }

    //vars defined temporarily
    double a1=axes[0]->DHJ.A;
    double a2=axes[1]->DHJ.A;
    double d1=axes[0]->DHJ.D;
    double d2=axes[1]->DHJ.D;
    double d3=axes[2]->DHJ.D;


    double theta1 = posACS(0);
    double theta2 = posACS(1);
    double theta3 = posACS(2);
    double theta4 = posACS(3);

    posMCS[0] = (a2+theta3 / 2 / M_PI * d2 * d3)*cos(theta1);
    posMCS[1] = (a2+theta3 / 2 / M_PI * d2 * d3)*sin(theta1);
    posMCS[2] = a1 + theta2 / 2 / M_PI * d1 * d3;
    posMCS[3] = M_PI;
    posMCS[4] = 0;
    posMCS[5] = theta4 - theta1;

    return RBTINTERF_NO_ERROR;
}

/**
 * @note 四轴码垛机器人逆解
 */
ErrorID RobotModel::calcInverseKin_Trans_FourAxis_StackingRobot_1(const Matrix4& transMatrix,
        const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const
{
    if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
                !=4)||(posACS.rows() !=4) ||(axes.size()!=4))
    {
        return INVERSE_KIN_TRANS_ARG_ERROR;
    }
    Matrix4 T(transMatrix);
    T *= toolMatrix.inverse();
    Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

    double nx(T(0,0));
    double ox(T(0,1));
    double px(T(0,3)/1000.0), py(T(1,3)/1000.0), pz(T(2,3)/1000.0);

    //vars defined temporarily
    double a1=axes[0]->DHJ.A/1000;
    double a2=axes[1]->DHJ.A/1000;

    double d1=axes[0]->DHJ.D/1000;
    double d2=axes[1]->DHJ.D/1000;
    double d3=axes[2]->DHJ.D;


    double smax,smin,lmax,lmin,umax,umin,rmax,rmin;
    //第一轴角度限制范围
    smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
    smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
    //第二轴角度限制范围
    lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
    lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
    //第三轴角度限制范围
    umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
    umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
    //第四轴角度限制范围
    rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
    rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);

    //solve for theta1
    double k1 = px;
    double k2 = py;
    double theta1;

    theta1 = atan2(k2,k1);
    theta1 = calcRealAngle(pLast(0), theta1, theta1);

    if((theta1 < M_PI*smin/180.0L) || (theta1 > M_PI*smax/180.0L))
    {
        theta1 = pLast(0);
        printf("theta1 exceeds pos limit.\n");
        jointLimitNum = 1;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(0) = theta1;

    //solve for theta4
    double theta4,theta4_1,theta4_2;
    theta4_1 = theta1 + atan2(-ox, nx);
    theta4_2 = theta4_1;

    theta4 = calcRealAngle(pLast(3), theta4_1, theta4_2);
    // the limit of theta1 according to the reference
    if((theta4 < M_PI*rmin/180.0L) || (theta4 > M_PI*rmax/180.0L))
    {
        theta4 = pLast(3);
        printf("theta4 exceeds pos limit.\n");
        jointLimitNum = 4;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(3) = theta4;

    //solve for theta3
    double k3;
    double theta3;

    if(fabs(sin(theta1)) < 10e-3)
    {
        k3 = px/ cos(theta1) - a2;
    }
    else
    {
        k3 = py / sin(theta1) - a2;
    }

    theta3 = k3 / d3 / d2 * 2 * M_PI;

    if((theta3 < M_PI*umin/180.0L) || (theta3 > M_PI*umax/180.0L))
    {
        theta3 = pLast(2);
        printf("theta3 exceeds pos limit.\n");
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(2) = theta3;

    //solve for theta2
    double theta2;

    theta2 = (pz-a1) / d3 / d1 * 2 * M_PI;
    if((theta2 < M_PI*lmin/180.0L) || (theta2 > M_PI*lmax/180.0L))
    {
        theta2 = pLast(1);
        printf("theta2 exceeds pos limit.\n");
        jointLimitNum = 2;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(1) = theta2;

    return RBTINTERF_NO_ERROR;
}

/**
 * @note 四轴机器人正解(六轴机器人无四六轴)
 */
ErrorID RobotModel::calcForwardKin_FourAxisRobot(const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) const
{
	if ( (posACS.rows() !=4)||(posMCS.rows() !=6) ||(axes.size()!=4))
	{
		return FORWARD_KIN_ARG_ERROR;
	}

	//vars defined temporarily
	double d1=axes[0]->DHJ.A;
	double d2=axes[1]->DHJ.A;
	double d3=axes[2]->DHJ.A;
	double d4=axes[3]->DHJ.A;
	double d5=axes[3]->DHJ.D;

	double theta1 = posACS(0);
	double theta2 = M_PI/2 + posACS(1);
	double theta3 = M_PI/2 - posACS(2);
	double theta4 = posACS(3) - (theta3 - theta2);
	double theta5 = theta2 - theta3 + M_PI/2;

	posMCS[0] = (d1+(d2*cos(theta2)+d3*cos(theta3-theta2))+d4*sin(theta4)+d5*cos(theta5))*cos(theta1);
	posMCS[1] = (d1+(d2*cos(theta2)+d3*cos(theta3-theta2))+d4*sin(theta4)+d5*cos(theta5))*sin(theta1);
	posMCS[2] = d2*sin(theta2)-d3*sin(theta3-theta2)-d4*cos(theta4)+d5*sin(theta5);
	posMCS[3] = M_PI;
	posMCS[4] = 0;
	posMCS[5] = theta4;

	return RBTINTERF_NO_ERROR;
}

/**
 * @note 四轴机器人逆解
 */
ErrorID RobotModel::calcInverseKin_Trans_FourAxisRobot(const Matrix4& transMatrix,
		const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const
{
	if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
				!=4)||(posACS.rows() !=4) ||(axes.size()!=4))
	{
		return INVERSE_KIN_TRANS_ARG_ERROR;
	}
	Matrix4 T(transMatrix);
	T *= toolMatrix.inverse();
	Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

	double nx(T(0,0));
	double ox(T(0,1));
	double px(T(0,3)/1000.0), py(T(1,3)/1000.0), pz(T(2,3)/1000.0);

	//vars defined temporarily
	double d1=axes[0]->DHJ.A/1000;
	double d2=axes[1]->DHJ.A/1000;
	double d3=axes[2]->DHJ.A/1000;
	double d4=axes[3]->DHJ.A/1000;
	double d5=axes[3]->DHJ.D/1000;
	double smax,smin,lmax,lmin,umax,umin,rmax,rmin;
    //第一轴角度限制范围
    smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
    smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
    //第二轴角度限制范围
    lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
    lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
    //第三轴角度限制范围
    umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
    umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
    //第四轴角度限制范围
    rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
    rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);

	//solve for theta1
	double k1 = px;
	double k2 = py;
	double theta1;
	double theta1_1;
	double theta1_2;

	theta1_1 = atan2(k2,k1);
	theta1_2 = atan2(-k2,-k1);
	theta1 = calcRealAngle(pLast(0), theta1_1, theta1_2);
	if((theta1 < M_PI*smin/180.0L) || (theta1 > M_PI*smax/180.0L))
	{
		theta1 = pLast(0);
		printf("theta1 exceeds pos limit.\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(0) = theta1;

	double theta4_1;
	double theta4_2;
	theta4_1 = atan2(-ox, nx);
	theta4_2 = atan2(ox, -nx);

	//solve for theta3
	double k3,k4,k5;
	double theta3;
	double theta3_1;
	double theta3_2;
	double theta3_3;
	if(fabs(sin(theta1)) < 10e-3)
	{
		k3 = px / cos(theta1) - d1 - d4 * sin(theta4_1);
	}
	else
	{
		k3 = py / sin(theta1) - d1 - d4 * sin(theta4_1);
	}
	k4 = pz + d4 * cos(theta4_1);
	k5 = (pow(k3,2) + pow(k4,2) - pow(d2,2) - pow(d3,2) - pow(d5,2))/(2*d2*sqrt(pow(d3,2)+pow(d5,2)));
	theta3_3 = atan2(d5,d3);
	if(fabs(k5)>1)
	{
		printf("Theta3 can not be solved, so can not reach the point!\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	theta3_1 = M_PI / 2 - (atan2(sqrt(1-pow(k5,2)),k5) + theta3_3);
	theta3_2 = M_PI / 2 - (atan2((-sqrt(1-pow(k5,2))),k5) + theta3_3);

	theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);
	if((theta3 < M_PI*umin/180.0L) || (theta3 > M_PI*umax/180.0L))
	{
		theta3 = pLast(2);
		printf("theta3 exceeds pos limit.\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(2) = theta3;

	//solve for theta2
	double theta2;
	double k6,k7;
	k6 = k3*(d2+sqrt(pow(d3,2)+pow(d5,2))*cos(M_PI / 2 - theta3 - theta3_3)) - sqrt(pow(d3,2)+pow(d5,2))*k4*sin(M_PI / 2 - theta3 - theta3_3);
	k7 = k4*(d2+sqrt(pow(d3,2)+pow(d5,2))*cos(M_PI / 2 - theta3 - theta3_3)) + k3*sqrt(pow(d3,2)+pow(d5,2))*sin(M_PI / 2 - theta3 - theta3_3);
	if((fabs(k6) < 10e-13) && (fabs(k7) < 10e-13))
	{
		theta2 = pLast(1);
	}
	else
	{
		theta2 = atan2(k7,k6) - M_PI / 2;
	}
	if((theta2 < M_PI*lmin/180.0L) || (theta2 > M_PI*lmax/180.0L))
	{
		theta2 = pLast(1);
		printf("theta2 exceeds pos limit.\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(1) = theta2;

	//solve for theta4
	double theta4;
	theta4_1 += (M_PI / 2 - theta3) - (theta2 + M_PI / 2);
	theta4_2 += (M_PI / 2 - theta3) - (theta2 + M_PI / 2);
	theta4 = calcRealAngle(pLast(3), theta4_1, theta4_2);
	// the limit of theta1 according to the reference
	if((theta4 < M_PI*rmin/180.0L) || (theta4 > M_PI*rmax/180.0L))
	{
		theta4 = pLast(3);
		printf("theta4 exceeds pos limit.\n");
		jointLimitNum = 4;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS(3) = theta4;

	return RBTINTERF_NO_ERROR;
}

/**
 * @note 四轴直角机器人正解
 */
ErrorID RobotModel::calcForwardKin_Four_CartesianCoordinateRobot(const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) const
{
	if ( (posACS.rows() !=4)||(posMCS.rows() !=6) ||(axes.size()!=4))
	{
		return FORWARD_KIN_ARG_ERROR;
	}
	double d1=axes[0]->DHJ.A;
	double d2=axes[1]->DHJ.A;
	double d3=axes[2]->DHJ.A;

	posMCS[0] = posACS[0]/2/M_PI*d1;
	posMCS[1] = posACS[1]/2/M_PI*d2;
	posMCS[2] = posACS[2]/2/M_PI*d3;
	posMCS[3] = M_PI;
	posMCS[4] = 0;
	posMCS[5] = posACS[3];

	return RBTINTERF_NO_ERROR;
}
/**
 * @note 四轴直角机器人逆解
 */
ErrorID RobotModel::calcInverseKin_Trans_Four_CartesianCoordinateRobot(const Matrix4& transMatrix,
		const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const
{
	if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
				!=4)||(posACS.rows() !=4) ||(axes.size()!=4))
	{
		return INVERSE_KIN_TRANS_ARG_ERROR;
	}
	Matrix4 T(transMatrix);
	T *= toolMatrix.inverse();
	Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

	//vars defined temporarily
	double d1=axes[0]->DHJ.A;
	double d2=axes[1]->DHJ.A;
	double d3=axes[2]->DHJ.A;

	double nx(T(0,0));
	double ox(T(0,1));
	double px(T(0,3)), py(T(1,3)), pz(T(2,3));
	double smax,smin,lmax,lmin,umax,umin,rmax,rmin;
    //第一轴角度限制范围
    smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
    smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
    //第二轴角度限制范围
    lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
    lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
    //第三轴角度限制范围
    umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
    umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
    //第四轴角度限制范围
    rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
    rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);

	posACS[0] = px * 2 * M_PI / d1;
	if((posACS[0] < M_PI*smin/180.0L) || (posACS[0] > M_PI*smax/180.0L))
	{
		posACS[0] = pLast(0);
		printf("theta4 exceeds pos limit.\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	posACS[1] = py * 2 * M_PI / d2;
	if((posACS[1] < M_PI*lmin/180.0L) || (posACS[1] > M_PI*lmax/180.0L))
	{
		posACS[1] = pLast(1);
		printf("theta4 exceeds pos limit.\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	posACS[2] = pz * 2 * M_PI / d3;
	if((posACS[2] < M_PI*umin/180.0L) || (posACS[2] > M_PI*umax/180.0L))
	{
		posACS[2] = pLast(2);
		printf("theta4 exceeds pos limit.\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	posACS[3] = atan2(-ox, nx);
	posACS[3] = calcRealAngle(pLast(3), posACS[3], posACS[3]);
	if((posACS[3] < M_PI*rmin/180.0L) || (posACS[3] > M_PI*rmax/180.0L))
	{
		posACS[3] = pLast(3);
		printf("theta4 exceeds pos limit.\n");
		jointLimitNum = 4;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	return RBTINTERF_NO_ERROR;
}

/**
 * @note 三轴直角机器人正解
 */
ErrorID RobotModel::calcForwardKin_Three_CartesianCoordinateRobot(const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) const
{
	if ( (posACS.rows() !=3)||(posMCS.rows() !=6) ||(axes.size()!=3))
	{
		return FORWARD_KIN_ARG_ERROR;
	}
	double d1=axes[0]->DHJ.A;
	double d2=axes[1]->DHJ.A;
	double d3=axes[2]->DHJ.A;


	posMCS[0] = posACS[0]/2/M_PI*d1;
	posMCS[1] = posACS[1]/2/M_PI*d2;
	posMCS[2] = posACS[2]/2/M_PI*d3;
	posMCS[3] = M_PI;
	posMCS[4] = 0;
	posMCS[5] = 0;

	return RBTINTERF_NO_ERROR;
}
/**
 * @note 三轴直角机器人逆解
 */
ErrorID RobotModel::calcInverseKin_Trans_Three_CartesianCoordinateRobot(const Matrix4& transMatrix,
		const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const
{
	if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
				!=3)||(posACS.rows() !=3) ||(axes.size()!=3))
	{
		return INVERSE_KIN_TRANS_ARG_ERROR;
	}
	Matrix4 T(transMatrix);
	T *= toolMatrix.inverse();
	Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

	//vars defined temporarily
	double d1=axes[0]->DHJ.A;
	double d2=axes[1]->DHJ.A;
	double d3=axes[2]->DHJ.A;

	double px(T(0,3)), py(T(1,3)), pz(T(2,3));
	double smax,smin,lmax,lmin,umax,umin;
    //第一轴角度限制范围
    smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
    smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
    //第二轴角度限制范围
    lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
    lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
    //第三轴角度限制范围
    umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
    umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);

	posACS[0] = px * 2 * M_PI / d1;
	if((posACS[0] < M_PI*smin/180.0L) || (posACS[0] > M_PI*smax/180.0L))
	{
		posACS[0] = pLast(0);
		printf("theta1 exceeds pos limit.\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	posACS[1] = py * 2 * M_PI / d2;
	if((posACS[1] < M_PI*lmin/180.0L) || (posACS[1] > M_PI*lmax/180.0L))
	{
		posACS[1] = pLast(1);
		printf("theta2 exceeds pos limit.\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	posACS[2] = pz * 2 * M_PI / d3;
	if((posACS[2] < M_PI*umin/180.0L) || (posACS[2] > M_PI*umax/180.0L))
	{
		posACS[2] = pLast(2);
		printf("theta3 exceeds pos limit.\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	return RBTINTERF_NO_ERROR;
}

/**
 * @note 三轴直角机器人异性1正解
 */
ErrorID RobotModel::calcForwardKin_Three_CartesianCoordinateRobot_1(const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) const
{
    if ( (posACS.rows() !=3)||(posMCS.rows() !=6) ||(axes.size()!=3))
    {
        return FORWARD_KIN_ARG_ERROR;
    }
    double d1=axes[0]->DHJ.D;
    double a2=axes[1]->DHJ.A;
    double d2=axes[1]->DHJ.D;
    double a3=axes[2]->DHJ.A;
    double d3=axes[2]->DHJ.D;

    posMCS[0] = posACS[1]/2/M_PI*a2*cos(posACS[0]) - d2 * sin(posACS[0]);
    posMCS[1] = posACS[1]/2/M_PI*a2*sin(posACS[0]) + d2 * cos(posACS[0]);
    posMCS[2] = d1-d3+posACS[2]/2/M_PI*a3;
    posMCS[3] = M_PI;
    posMCS[4] = 0;
    posMCS[5] = 0;

    return RBTINTERF_NO_ERROR;
}

/**
 * @note 三轴直角机器人异型1逆解
 */
ErrorID RobotModel::calcInverseKin_Trans_Three_CartesianCoordinateRobot_1(const Matrix4& transMatrix,
        const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const
{
    if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
                !=3)||(posACS.rows() !=3) ||(axes.size()!=3))
    {
        return INVERSE_KIN_TRANS_ARG_ERROR;
    }
    Matrix4 T(transMatrix);
    T *= toolMatrix.inverse();
    Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

    //vars defined temporarily
    double d1=axes[0]->DHJ.D;
    double a2=axes[1]->DHJ.A;
    double d2=axes[1]->DHJ.D;
    double a3=axes[2]->DHJ.A;
    double d3=axes[2]->DHJ.D;

    double px(T(0,3)), py(T(1,3)), pz(T(2,3));
    double smax,smin,lmax,lmin,umax,umin;
    //第一轴角度限制范围
    smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
    smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
    //第二轴角度限制范围
    lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
    lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
    //第三轴角度限制范围
    umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
    umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);


    double temp_var = pow(px,2) + pow(py,2) - pow(d2,2);
    if(temp_var < 0.0L)
    {
        printf("Theta2 can not be solved, so can not reach the point!\n");
        jointLimitNum = 2;
        return INVERSE_KIN_NOT_REACHABLE;
    }

    posACS[1] = sqrt(temp_var) * 2 * M_PI / a2;
    if((posACS[1] < M_PI*lmin/180.0L) || (posACS[1] > M_PI*lmax/180.0L))
    {
        posACS[1] = pLast(1);
        printf("theta2 exceeds pos limit.\n");
        jointLimitNum = 2;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    double k1 = posACS[1]/2/M_PI*a2;
    double k2 = d2;
    posACS[0] = atan2(k1*py-k2*px,k1*px+k2*py);

    if((posACS[0] < M_PI*smin/180.0L) || (posACS[0] > M_PI*smax/180.0L))
    {
        posACS[0] = pLast(0);
        printf("theta1 exceeds pos limit.\n");
        jointLimitNum = 1;
        return INVERSE_KIN_NOT_REACHABLE;
    }


    posACS[2] = (pz - d1 + d3)*2*M_PI/a3;
    if((posACS[2] < M_PI*umin/180.0L) || (posACS[2] > M_PI*umax/180.0L))
    {
        posACS[2] = pLast(2);
        printf("theta3 exceeds pos limit.\n");
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
    }

    return RBTINTERF_NO_ERROR;
}

ErrorID RobotModel::calcForwardKin_DeltaRobot(
		const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) const
{
	if ( (posACS.rows() !=4)||(posMCS.rows() !=6) ||(axes.size()!=4))
	{
		return FORWARD_KIN_ARG_ERROR;
	}
	double d1 = axes[0]->DHJ.D;//固定盘半径
	double d2 = axes[1]->DHJ.D;//移动盘半径
	double d3 = axes[2]->DHJ.A;//主动杆杆长
	double d4 = axes[2]->DHJ.D;//从动杆杆长
	double d5 = axes[3]->DHJ.D;//主动杆末端到从动杆起点的长度
	if(d3 >= d4)
	{
		posMCS[0] = 0;
		posMCS[1] = 0;
		posMCS[2] = 0;
		posMCS[3] = 0;
		posMCS[4] = 0;
		posMCS[5] = 00;
		return FORWARD_KIN_ARG_ERROR;
	}
	double R = d1;
	double r = d2;
	double alpha[3];
	alpha[0] = 0;
	alpha[1] = 2.0/3.0*M_PI;
	alpha[2] = -2.0/3.0*M_PI;
	double alpha_offset = M_PI / 2;
	Vector3 OE1,OE2,OE3,op1,op2,op3,OB,OC,OD,BC,CD,BD,EF,OF,FA,mcs;
	OE1<<(d3 * sin(posACS(0) + alpha_offset) - d5 * cos(posACS(0) + alpha_offset) + R) * cos(alpha[0]), (d3 * sin(posACS(0) + alpha_offset) - d5 * cos(posACS(0) + alpha_offset) + R) * sin(alpha[0]), d3 * cos(posACS(0) + alpha_offset) + d5 * sin(posACS(0) + alpha_offset);
	OE2<<(d3 * sin(posACS(1) + alpha_offset) - d5 * cos(posACS(1) + alpha_offset) + R) * cos(alpha[1]), (d3 * sin(posACS(1) + alpha_offset) - d5 * cos(posACS(1) + alpha_offset) + R) * sin(alpha[1]), d3 * cos(posACS(1) + alpha_offset) + d5 * sin(posACS(1) + alpha_offset);
	OE3<<(d3 * sin(posACS(2) + alpha_offset) - d5 * cos(posACS(2) + alpha_offset) + R) * cos(alpha[2]), (d3 * sin(posACS(2) + alpha_offset) - d5 * cos(posACS(2) + alpha_offset) + R) * sin(alpha[2]), d3 * cos(posACS(2) + alpha_offset) + d5 * sin(posACS(2) + alpha_offset);
	op1<<r * cos(alpha[0]), r * sin(alpha[0]), 0;
	op2<<r * cos(alpha[1]), r * sin(alpha[1]), 0;
	op3<<r * cos(alpha[2]), r * sin(alpha[2]), 0;

	OB=OE1-op1;
	OC=OE2-op2;
	OD=OE3-op3;
	BC=OC-OB;CD=OD-OC;BD=OD-OB;
	double a,b,c,p,bf,ef;
	a=BC.norm();
	b=CD.norm();
	c=BD.norm();
	p=(a+b+c)/2;
	bf=a*b*c/4/sqrt(p*(p-a)*(p-b)*(p-c));
	ef=sqrt(bf*bf-a*a/4);
	EF=(BC.cross(CD)).cross(BC)/((BC.cross(CD)).cross(BC)).norm()*ef;
	if(CD.dot(BD)<0)
	    EF=-EF;
	OF=(OB+OC)/2 + EF;
	FA=BC.cross(CD)/(BC.cross(CD)).norm()*sqrt(d4*d4-bf*bf);
	if(FA(2)<0)
	    FA=-FA;
	mcs=OF + FA;
	posMCS[0] = mcs(0);
	posMCS[1] = -mcs(1);
	posMCS[2] = -mcs(2);
	posMCS[3] = 0;
	posMCS[4] = 0;
	posMCS[5] = posACS(3);
    return RBTINTERF_NO_ERROR;
}
ErrorID RobotModel::calcInverseKin_Trans_DeltaRobot(
		const dmatrix& transMatrix, const Position_ACS_rad& posLast,
		Position_ACS_rad& posACS) const
{
    if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
                !=4)||(posACS.rows() !=4) ||(axes.size()!=4))
    {
        return INVERSE_KIN_TRANS_ARG_ERROR;
    }
	dmatrix T(transMatrix);
	T *= toolMatrix.inverse();
	double d1 = axes[0]->DHJ.D/1000;//固定盘边长
	double d2 = axes[1]->DHJ.D/1000;//移动盘边长
	double d3 = axes[2]->DHJ.A/1000;//主动杆杆长
	double d4 = axes[2]->DHJ.D/1000;//从动杆杆长
	double d5 = axes[3]->DHJ.D/1000;//主动杆末端到从动杆起点的长度

	double smax,smin,lmax,lmin,umax,umin,rmax,rmin;
	//第一轴角度限制范围
	smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
	smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
	//第二轴角度限制范围
	lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
	lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
	//第三轴角度限制范围
	umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
	umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
	//第四轴角度限制范围
	rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
	rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);

	double R = d1;
	double r = d2;
    double x = T(0,3)/1000;
    double y = -T(1,3)/1000;
    double z = -T(2,3)/1000;
    double nx = T(0,0);
    double ox = T(0,1);
    double theta4;
    theta4 = atan2(-ox, nx);
	if((theta4 < M_PI*rmin/180.0L) || (theta4 > M_PI*rmax/180.0L))
	{
		posACS(3) = posLast(3);
		printf("theta4 exceeds pos limit.\n");
		jointLimitNum = 4;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	posACS[3] = calcRealAngle(posLast(3), theta4, theta4);



	//包含d5的算法
	double alpha[3];
	alpha[0] = 0;
	alpha[1] = 2.0/3.0*M_PI;
	alpha[2] = -2.0/3.0*M_PI;
	double theta_result[3][2];
	double alpha_offset = M_PI / 2;
	for(int i = 0;i < 3;i++)
	{
		double temp_para_1 = (R - r - x * cos(alpha[i]) - y * sin(alpha[i])) * d3 - z * d5;
		double temp_para_2 = (R - r - x * cos(alpha[i]) - y * sin(alpha[i])) * d5 + z * d3;
		double temp_para_3 = (pow(d4, 2) - pow(d3, 2) - pow(d5, 2) - pow(z, 2) - pow((R - r) * cos(alpha[i]) - x, 2) - pow((R - r) * sin(alpha[i]) - y, 2)) / 2;

	    double a_3 = -temp_para_2;
	    double d_4 = -temp_para_1;
	    double k_4 = temp_para_3;
	    double temp_value = a_3 * a_3 +  d_4 * d_4 - k_4 * k_4;
	    if(temp_value < 0)
	    {
			printf("Theta%i can not be solved, so can not reach the point!\n", i + 1);
			jointLimitNum = i + 1;
			return INVERSE_KIN_NOT_REACHABLE;
	    }
	    double delta = sqrt(temp_value);
	    theta_result[i][0] = -(atan2(d_4, a_3) + atan2(delta, k_4)) - alpha_offset;
	    theta_result[i][1] = -(atan2(d_4, a_3) - atan2(delta, k_4)) - alpha_offset;
	}
	double theta1_1,theta1_2,theta2_1,theta2_2,theta3_1,theta3_2;
	theta1_1 = theta_result[0][0];
	theta1_2 = theta_result[0][1];

	theta2_1 = theta_result[1][0];
	theta2_2 = theta_result[1][1];

	theta3_1 = theta_result[2][0];
	theta3_2 = theta_result[2][1];

	posACS[0] = calcRealAngle(posLast(0), theta1_1, theta1_2);

	if((posACS(0) < M_PI*smin/180.0L) || (posACS(0) > M_PI*smax/180.0L))
	{
		posACS(0) = posLast(0);
		printf("theta1 exceeds pos limit.\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	posACS[1] = calcRealAngle(posLast(1), theta2_1, theta2_2);

	if((posACS(1) < M_PI*lmin/180.0L) || (posACS(1) > M_PI*lmax/180.0L))
	{
		posACS(1) = posLast(1);
		printf("theta2 exceeds pos limit.\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}

	posACS[2] = calcRealAngle(posLast(2), theta3_1, theta3_2);

	if((posACS(2) < M_PI*umin/180.0L) || (posACS(2) > M_PI*umax/180.0L))
	{
		posACS[2] = posLast(2);
		printf("theta3 exceeds pos limit.\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}

    if(!IS_VALID_DOUBLE(posACS[0]) || !IS_VALID_DOUBLE(posACS[1]) || !IS_VALID_DOUBLE(posACS[2]) || !IS_VALID_DOUBLE(posACS[3]))
    {
		printf("inverse calculate result is nan\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
    }

    return RBTINTERF_NO_ERROR;
}

/**
 * @note 四轴极坐标异型1正解
 */
ErrorID RobotModel::calcForwardKin_FOUR_POLAR_COORDINATE_1(const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) const
{
    if ( (posACS.rows() !=4)||(posMCS.rows() !=6) ||(axes.size()!=4))
    {
        return FORWARD_KIN_ARG_ERROR;
    }
    double d1=axes[0]->DHJ.D;
    double a2=axes[1]->DHJ.A;
    double d2=axes[1]->DHJ.D;
    double a3=axes[2]->DHJ.A;
    double d3=axes[2]->DHJ.D;

    posMCS[0] = (posACS[2]/2/M_PI*a3 + d1)*cos(posACS[0]) - d2 * sin(posACS[0]);
    posMCS[1] = (posACS[2]/2/M_PI*a3 + d1)*sin(posACS[0]) + d2 * cos(posACS[0]);
    posMCS[2] = -d3+posACS[1]/2/M_PI*a2;
    posMCS[3] = M_PI;
    posMCS[4] = 0;
    posMCS[5] = -(posACS[0] + posACS[3]);

    return RBTINTERF_NO_ERROR;
}

/**
 * @note 四轴极坐标异型1逆解
 */
ErrorID RobotModel::calcInverseKin_FOUR_POLAR_COORDINATE_1(const Matrix4& transMatrix,
        const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const
{
    if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
                !=4)||(posACS.rows() !=4) ||(axes.size()!=4))
    {
        return INVERSE_KIN_TRANS_ARG_ERROR;
    }
    Matrix4 T(transMatrix);
    T *= toolMatrix.inverse();
    Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

    //vars defined temporarily
    double d1=axes[0]->DHJ.D;
    double a2=axes[1]->DHJ.A;
    double d2=axes[1]->DHJ.D;
    double a3=axes[2]->DHJ.A;
    double d3=axes[2]->DHJ.D;

    double nx(T(0,0));
    double ox(T(0,1));
    double px(T(0,3)), py(T(1,3)), pz(T(2,3));
    double smax,smin,lmax,lmin,umax,umin,rmax,rmin;
    //第一轴角度限制范围
    smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
    smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
    //第二轴角度限制范围
    lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
    lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
    //第三轴角度限制范围
    umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
    umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
    //第四轴角度限制范围
    rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
    rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);


    double temp_var = pow(px,2) + pow(py,2) - pow(d2,2);
    if(temp_var < 0.0L)
    {
        printf("Theta2 can not be solved, so can not reach the point!\n");
        jointLimitNum = 2;
        return INVERSE_KIN_NOT_REACHABLE;
    }

    posACS[2] = (sqrt(temp_var) - d1) * 2 * M_PI / a3;
    if((posACS[2] < M_PI*umin/180.0L) || (posACS[2] > M_PI*umax/180.0L))
    {
        posACS[2] = pLast(2);
        printf("theta3 exceeds pos limit.\n");
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    double k1 = posACS[2]/2/M_PI*a3 + d1;
    double k2 = d2;
    posACS[0] = atan2(k1*py-k2*px,k1*px+k2*py);
    posACS[0] = calcRealAngle(pLast(0), posACS[0], posACS[0]);

    if((posACS[0] < M_PI*smin/180.0L) || (posACS[0] > M_PI*smax/180.0L))
    {
        posACS[0] = pLast(0);
        printf("theta1 exceeds pos limit.\n");
        jointLimitNum = 1;
        return INVERSE_KIN_NOT_REACHABLE;
    }


    posACS[1] = (pz + d3)*2*M_PI/a2;
    if((posACS[1] < M_PI*lmin/180.0L) || (posACS[1] > M_PI*lmax/180.0L))
    {
        posACS[1] = pLast(1);
        printf("theta2 exceeds pos limit.\n");
        jointLimitNum = 2;
        return INVERSE_KIN_NOT_REACHABLE;
    }

	posACS[3] = -atan2(-ox, nx) - posACS[0];
	posACS[3] = calcRealAngle(pLast(3), posACS[3], posACS[3]);

	if((posACS[3] < M_PI*rmin/180.0L) || (posACS[3] > M_PI*rmax/180.0L))
	{
		posACS[3] = pLast(3);
		printf("theta4 exceeds pos limit.\n");
		jointLimitNum = 4;
		return INVERSE_KIN_NOT_REACHABLE;
	}


    return RBTINTERF_NO_ERROR;
}

ErrorID RobotModel::calcForwardKin_GANTRY_WELD(
        const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) const
{
    if ( (posACS.rows() !=5)||(posMCS.rows() !=6) ||(axes.size()!=5))
    {
        return FORWARD_KIN_ARG_ERROR;
    }
    double d1=axes[0]->DHJ.D;
    double d2=axes[1]->DHJ.D;
    double d3=axes[2]->DHJ.D;
    DH_Parameters dhParam;
	Matrix4 currentMatrix(Matrix4::Identity( 4, 4));
	Position_ACS_rad q(posACS);
    for (unsigned int i = 0; i < axes.size(); i++)
    {
        q[i] = q[i] * axes[i]->axisDirection;
    }
	for(int sz = 3;sz < 5;sz++)
	{
		dhParam = axes[sz]->DHJ;
		dhParam.Theta = q(sz) + dhParam.Theta;// theta = posACS + offset
		currentMatrix = AllCompute(dhParam, currentMatrix);
	}
	Position_MCS_rad mcs_temp(6);
    mcs_temp = tr2MCS(currentMatrix);
    posMCS(0) = posACS(0) / M_PI * 180 * d1 / 360 + currentMatrix(0,3);
    posMCS(1) = posACS(1) / M_PI * 180 * d2 / 360 + currentMatrix(1,3);
    posMCS(2) = posACS(2) / M_PI * 180 * d3 / 360 + currentMatrix(2,3);
    posMCS(3) = mcs_temp(3);
    posMCS(4) = mcs_temp(4);
    posMCS(5) = mcs_temp(5);
    return RBTINTERF_NO_ERROR;
}

ErrorID RobotModel::calcInverseKin_GANTRY_WELD(
        const Matrix4& transMatrix, const Position_ACS_rad& posLast,
        Position_ACS_rad& posACS) const
{
    if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
                !=5)||(posACS.rows() !=5) ||(axes.size()!=5))
    {
        return INVERSE_KIN_TRANS_ARG_ERROR;
    }

	double smax = 0.0;
	double smin = 0.0;
	double lmax = 0.0;
	double lmin = 0.0;
	double umax = 0.0;
	double umin = 0.0;
	double rmax  = 0.0;
	double rmin = 0.0;
	double bmax = 0.0;
	double bmin = 0.0;
	//第一轴角度限制范围
	smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
	smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
	//第二轴角度限制范围
	lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
	lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
	//第三轴角度限制范围
	umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
	umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
	//第四轴角度限制范围
	rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
	rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);
	//第五轴角度限制范围
	bmax=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.posSWLimit):(-1 * axes[4]->paramLmt.negSWLimit);
	bmin=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.negSWLimit):(-1 * axes[4]->paramLmt.posSWLimit);

    Matrix4 T(transMatrix);
    T *= toolMatrix.inverse();
    Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

    //vars defined temporarily
    double d1=axes[0]->DHJ.D;
    double d2=axes[1]->DHJ.D;
    double d3=axes[2]->DHJ.D;
    double d4=axes[3]->DHJ.D;
    double d5=axes[4]->DHJ.D;

    double theta5 = atan2(T(2,0),T(2,1));
    double theta4 = atan2(T(0,2),-T(1,2)) - M_PI / 2;

    posACS(4) = calcRealAngle(pLast(4), theta5, theta5);
    posACS(3) = calcRealAngle(pLast(3), theta4, theta4);
    posACS(2) = (T(2,3) - d4) / d3 * 360 / 180 * M_PI;
    posACS(1) = (T(1,3) + d5 * cos(posACS(3) + M_PI / 2)) / d2 * 360 / 180 * M_PI;
    posACS(0) = (T(0,3) - d5 * sin(posACS(3) + M_PI / 2)) / d1 * 360 / 180 * M_PI;

	if((posACS[0] < M_PI*smin/180.0L) || (posACS[0] > M_PI*smax/180.0L))
	{
		posACS[0] = pLast(0);
		printf("theta1 exceeds pos limit.\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	if((posACS[1] < M_PI*lmin/180.0L) || (posACS[1] > M_PI*lmax/180.0L))
	{
		posACS[1] = pLast(1);
		printf("theta2 exceeds pos limit.\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	if((posACS[2] < M_PI*umin/180.0L) || (posACS[2] > M_PI*umax/180.0L))
	{
		posACS[2] = pLast(2);
		printf("theta3 exceeds pos limit.\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	if((posACS[3] < M_PI*rmin/180.0L) || (posACS[3] > M_PI*rmax/180.0L))
	{
		posACS[3] = pLast(3);
		printf("theta4 exceeds pos limit.\n");
		jointLimitNum = 4;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	if((posACS[4] < M_PI*bmin/180.0L) || (posACS[4] > M_PI*bmax/180.0L))
	{
		posACS[4] = pLast(4);
		printf("theta5 exceeds pos limit.\n");
		jointLimitNum = 5;
		return INVERSE_KIN_NOT_REACHABLE;
	}
    return RBTINTERF_NO_ERROR;
}

ErrorID RobotModel::calcForwardKin_GANTRY_WELD_2(
        const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) const
{
	if ( (posACS.rows() !=5)||(posMCS.rows() !=6) ||(axes.size()!=5))
	{
		return FORWARD_KIN_ARG_ERROR;
	}
	double d1 = axes[0]->DHJ.D;
	double d2 = axes[1]->DHJ.D;
	double d3 = axes[2]->DHJ.D;
	DH_Parameters dhParam;
	Matrix4 currentMatrix(Matrix4::Identity( 4, 4));
	Position_ACS_rad q(posACS);
	for (unsigned int i = 0; i < axes.size(); i++)
	{
		q[i] = q[i] * axes[i]->axisDirection;
	}
	DH_Parameters dh_temp;
	dh_temp.Theta = 0;
	dh_temp.D = 0;
	dh_temp.A = 0;
	dh_temp.Alpha = -M_PI / 2;
	currentMatrix = AllCompute(dh_temp, currentMatrix);
	for(int sz = 3;sz < 5;sz++)
	{
		dhParam = axes[sz]->DHJ;
		dhParam.Theta = q(sz) + dhParam.Theta;// theta = posACS + offset
		currentMatrix = AllCompute(dhParam, currentMatrix);
	}
	Position_MCS_rad mcs_temp(6);
	mcs_temp = tr2MCS(currentMatrix);
	posMCS(0) = posACS(0) / M_PI * 180 * d1 / 360 + currentMatrix(0,3);
	posMCS(1) = posACS(1) / M_PI * 180 * d2 / 360 + currentMatrix(1,3);
	posMCS(2) = posACS(2) / M_PI * 180 * d3 / 360 + currentMatrix(2,3);
	posMCS(3) = mcs_temp(3);
	posMCS(4) = mcs_temp(4);
	posMCS(5) = mcs_temp(5);
	return RBTINTERF_NO_ERROR;
}
ErrorID RobotModel::calcInverseKin_GANTRY_WELD_2(
        const Matrix4& transMatrix, const Position_ACS_rad& posLast,
        Position_ACS_rad& posACS) const
{
	if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
			!=5)||(posACS.rows() !=5) ||(axes.size()!=5))
	{
		return INVERSE_KIN_TRANS_ARG_ERROR;
	}

	double smax = 0.0;
	double smin = 0.0;
	double lmax = 0.0;
	double lmin = 0.0;
	double umax = 0.0;
	double umin = 0.0;
	double rmax  = 0.0;
	double rmin = 0.0;
	double bmax = 0.0;
	double bmin = 0.0;
	//第一轴角度限制范围
	smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
	smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
	//第二轴角度限制范围
	lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
	lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
	//第三轴角度限制范围
	umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
	umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
	//第四轴角度限制范围
	rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
	rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);
	//第五轴角度限制范围
	bmax=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.posSWLimit):(-1 * axes[4]->paramLmt.negSWLimit);
	bmin=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.negSWLimit):(-1 * axes[4]->paramLmt.posSWLimit);

	Matrix4 T(transMatrix);
	T *= toolMatrix.inverse();

	Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

	//vars defined temporarily
	double d1 = axes[0]->DHJ.D;
	double d2 = axes[1]->DHJ.D;
	double d3 = axes[2]->DHJ.D;
	double d5 = axes[4]->DHJ.D;
	double xxx = axes[3]->DHJ.Theta;
	double theta5 = atan2(-T(1,0),-T(1,1));
	double theta4 = atan2(-T(0,2),-T(2,2)) - xxx;


	posACS(4) = calcRealAngle(pLast(4), theta5, theta5);
	posACS(3) = calcRealAngle(pLast(3), theta4, theta4);
	posACS(2) = (T(2,3) + d5 * cos(posACS(3) + xxx)) / d3 * 360 / 180 * M_PI;
	posACS(1) = T(1,3) / d2 * 360 / 180 * M_PI;
	posACS(0) = (T(0,3) + d5 * sin(posACS(3) + xxx)) / d1 * 360 / 180 * M_PI;
	if((posACS[0] < M_PI*smin/180.0L) || (posACS[0] > M_PI*smax/180.0L))
	{
		posACS[0] = pLast(0);
		printf("theta1 exceeds pos limit.\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	if((posACS[1] < M_PI*lmin/180.0L) || (posACS[1] > M_PI*lmax/180.0L))
	{
		posACS[1] = pLast(1);
		printf("theta2 exceeds pos limit.\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	if((posACS[2] < M_PI*umin/180.0L) || (posACS[2] > M_PI*umax/180.0L))
	{
		posACS[2] = pLast(2);
		printf("theta3 exceeds pos limit.\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	if((posACS[3] < M_PI*rmin/180.0L) || (posACS[3] > M_PI*rmax/180.0L))
	{
		posACS[3] = pLast(3);
		printf("theta4 exceeds pos limit.\n");
		jointLimitNum = 4;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	if((posACS[4] < M_PI*bmin/180.0L) || (posACS[4] > M_PI*bmax/180.0L))
	{
		posACS[4] = pLast(4);
		printf("theta5 exceeds pos limit.\n");
		jointLimitNum = 5;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	return RBTINTERF_NO_ERROR;
}

ErrorID RobotModel::calcForwardKin_WINE_CHAMFER(
        const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) const
{
    if ( (posACS.rows() !=4)||(posMCS.rows() !=6) ||(axes.size()!=4))
    {
        return FORWARD_KIN_ARG_ERROR;
    }
    double d1 = axes[0]->DHJ.A; //L1
    double d2 = axes[0]->DHJ.D; //L2
    double d3 = axes[1]->DHJ.A; //L3
    double d4 = axes[1]->DHJ.D; //L4
    double d5 = axes[2]->DHJ.A; //L5
    double d6 = axes[2]->DHJ.D; //顶升电动缸导程L13
    double d7 = axes[3]->DHJ.A; //喷料距离L13
    double d8 = axes[3]->DHJ.D; //滑动电动缸导程L12
    double d9 = rad2deg(axes[2]->DHJ.Theta); //L6
    double d10 = rad2deg(axes[0]->DHJ.Alpha); //L7
    double d11 = rad2deg(axes[0]->DHJ.Theta); //L8
    double d12 = rad2deg(axes[1]->DHJ.Alpha); //L9
    double d13 = rad2deg(axes[1]->DHJ.Theta); //L10
    double d14 = rad2deg(axes[2]->DHJ.Alpha); //L11

    double delta_theta = 0;
    double delta_d5 = d5 + posACS(1) / (2*M_PI) * d6;
    double theta2_zero = acos((pow(d4,2) + pow(d3,2) - pow(d5,2)) / (2 * d3 * d4));
    double theta2_delta = acos((pow(d4,2) + pow(d3,2) - pow(delta_d5,2)) / (2 * d3 * d4));
    delta_theta = theta2_delta - theta2_zero;

    posMCS[0] = -d10 * sin(delta_theta) + d1 * cos(delta_theta) - d9 * sin(delta_theta) + posACS(0) / (2*M_PI) * d8 + (d2 * sin(posACS(3)) + d7) * cos(posACS(2));
    posMCS[1] = (d2 * sin(posACS(3)) + d7) * sin(posACS(2));
    posMCS[2] = d11 + d14 + d10 * cos(delta_theta) + d1 * sin(delta_theta) + d9 * cos(delta_theta) - d9 - d12 - d2 * cos(posACS(3)) - d13;
    posMCS[3] = 0;
    posMCS[4] = 0;
    posMCS[5] = posACS(2);
    return RBTINTERF_NO_ERROR;

}
ErrorID RobotModel::calcInverseKin_WINE_CHAMFER(
        const Matrix4& transMatrix, const Position_ACS_rad& posLast,
        Position_ACS_rad& posACS) const
{
    if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
                !=4)||(posACS.rows() !=4) ||(axes.size()!=4))
    {
        return INVERSE_KIN_TRANS_ARG_ERROR;
    }

	double smax = 0.0;
	double smin = 0.0;
	double lmax = 0.0;
	double lmin = 0.0;
	double umax = 0.0;
	double umin = 0.0;
	double rmax = 0.0;
	double rmin = 0.0;

	//第一轴角度限制范围
	smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
	smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
	//第二轴角度限制范围
	lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
	lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
	//第三轴角度限制范围
	umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
	umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
	//第四轴角度限制范围
	rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
	rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);

    Matrix4 T(transMatrix);
    T *= toolMatrix.inverse();
    Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值
	double x(T(0,3)), y(T(1,3)), z(T(2,3));
	double x_0 = wine_chamfer_circle_center.pos_x;
	double y_0 = wine_chamfer_circle_center.pos_y;
	double z_0 = wine_chamfer_circle_center.pos_z;

	//vars defined temporarily
    double d1 = axes[0]->DHJ.A; //L1
    double d2 = axes[0]->DHJ.D; //L2
    double d3 = axes[1]->DHJ.A; //L3
    double d4 = axes[1]->DHJ.D; //L4
    double d5 = axes[2]->DHJ.A; //L5
    double d6 = axes[2]->DHJ.D; //顶升电动缸导程L13
    double d7 = axes[3]->DHJ.A; //喷料距离L13
    double d8 = axes[3]->DHJ.D; //滑动电动缸导程L12
    double d9 = rad2deg(axes[2]->DHJ.Theta); //L6
    double d10 = rad2deg(axes[0]->DHJ.Alpha); //L7

    double theta3 =  atan2(y - y_0,x - x_0);
    double theta3_1 = theta3 + 2 * M_PI;
    double theta3_2 = theta3 - 2 * M_PI;
    if(fabs(theta3 - posLast(2)) < fabs(theta3_1 - posLast(2)))
    {
    	if(fabs(theta3 - posLast(2)) < fabs(theta3_2 - posLast(2)))
    		posACS(2) = theta3;
    	else
    		posACS(2) = theta3_2;
    }
    else
    {
    	if(fabs(theta3_1 - posLast(2)) < fabs(theta3_2 - posLast(2)))
    		posACS(2) = theta3_1;
    	else
    		posACS(2) = theta3_2;
    }
	if((posACS[2] < M_PI*umin/180.0L) || (posACS[2] > M_PI*umax/180.0L))
		posACS(2) = theta3;
    if(fabs(cos(posACS(2))) < 1e-10)
    {
    	if(fabs(((y - y_0) / sin(posACS(2)) - d7) / d2) > 1 )
    	{
    		printf("Theta4 can not be solved, so can not reach the point!\n");
    		jointLimitNum = 4;
    		return INVERSE_KIN_NOT_REACHABLE;
    	}
    	double theta4_1 = asin(((y - y_0) / sin(posACS(2)) - d7) / d2);
    	double theta4_2 = -asin(((y - y_0) / sin(posACS(2)) - d7) / d2);

        if(((y - y_0) / sin(posACS(2)) - d7) > 0)
        {
        	if(theta4_1 > 0)
        		posACS(3) = theta4_1;
        	else
        		posACS(3) = theta4_2;
        }
        else
        {
        	if(theta4_1 < 0)
        		posACS(3) = theta4_1;
        	else
        		posACS(3) = theta4_2;
        }
    }
    else
    {
    	if(fabs(((x - x_0) / cos(posACS(2)) - d7) / d2) > 1 )
    	{
    		printf("Theta4 can not be solved, so can not reach the point!\n");
    		jointLimitNum = 4;
    		return INVERSE_KIN_NOT_REACHABLE;
    	}
    	double theta4_1 = asin(((x - x_0) / cos(posACS(2)) - d7) / d2);
    	double theta4_2 = -asin(((x - x_0) / cos(posACS(2)) - d7) / d2);
        if(((x - x_0) / cos(posACS(2)) - d7) > 0)
        {
    		if(theta4_1 > 0)
    			posACS(3) = theta4_1;
    		else
    			posACS(3) = theta4_2;
        }
        else
        {
    		if(theta4_1 < 0)
    			posACS(3) = theta4_1;
    		else
    			posACS(3) = theta4_2;
        }
    }
    double delta_z = d2 * cos(posACS(3)) - d2 * cos(0);
    double a_3 = d9 + d10;
    double d_4 = -d1;
    double k_4 = z - z_0 + delta_z + d9 + d10;
    double temp_value = a_3 * a_3 +  d_4 * d_4 - k_4 * k_4;
    if(temp_value < 0)
    {
		printf("Theta2 can not be solved, so can not reach the point!\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
    }
    double delta = sqrt(temp_value);
	double delta_theta_1 = -(atan2(d_4, a_3) + atan2(delta, k_4));
	double delta_theta_2 = -(atan2(d_4, a_3) - atan2(delta, k_4));
    double delta_theta_zero = acos((pow(d4,2) + pow(d3,2) - pow(d5,2)) / (2 * d3 * d4));
	double delta_theta_temp_1 = delta_theta_1 + delta_theta_zero;
	double delta_theta_temp_2 = delta_theta_2 + delta_theta_zero;
    double delta_d5_1 = sqrt(pow(d3,2) + pow(d4,2) - 2 * d3 * d4 * cos(delta_theta_temp_1));
    double delta_d5_2 = sqrt(pow(d3,2) + pow(d4,2) - 2 * d3 * d4 * cos(delta_theta_temp_2));
    double theta1_1 = (delta_d5_1 - d5) * 2 * M_PI / d6;
    double theta1_2 = (delta_d5_2 - d5) * 2 * M_PI / d6;
    double delta_x_0;
    if(fabs(theta1_1 - posLast(0)) < fabs(theta1_2 - posLast(0)))
    {
    	posACS(1) = theta1_1;
    	delta_x_0 = x_0 + d10 * sin(delta_theta_1) - d1 * cos(delta_theta_1) + d9 * sin(delta_theta_1) - wine_chamfer_circle_center.joint_pos[0] / (2*M_PI) * d8;
    }
    else
    {
    	posACS(1) = theta1_2;
    	delta_x_0 = x_0 + d10 * sin(delta_theta_2) - d1 * cos(delta_theta_2) + d9 * sin(delta_theta_2) - wine_chamfer_circle_center.joint_pos[0] / (2*M_PI) * d8;
    }
    posACS(0) = delta_x_0 / d8 * (2*M_PI) + wine_chamfer_circle_center.joint_pos[0];
    if(!IS_VALID_DOUBLE(posACS[0]) || !IS_VALID_DOUBLE(posACS[1]) || !IS_VALID_DOUBLE(posACS[2]) || !IS_VALID_DOUBLE(posACS[3]))
    {
		printf("inverse calculate result is nan\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
    }
	if((posACS[0] < M_PI*smin/180.0L) || (posACS[0] > M_PI*smax/180.0L))
	{
		posACS[0] = pLast(0);
		printf("theta1 exceeds pos limit.\n");
		jointLimitNum = 1;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	if((posACS[1] < M_PI*lmin/180.0L) || (posACS[1] > M_PI*lmax/180.0L))
	{
		posACS[1] = pLast(1);
		printf("theta2 exceeds pos limit.\n");
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	if((posACS[2] < M_PI*umin/180.0L) || (posACS[2] > M_PI*umax/180.0L))
	{
		posACS[2] = pLast(2);
		printf("theta3 exceeds pos limit.\n");
		jointLimitNum = 3;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	if((posACS[3] < M_PI*rmin/180.0L) || (posACS[3] > M_PI*rmax/180.0L))
	{
		posACS[3] = pLast(3);
		printf("theta4 exceeds pos limit.\n");
		jointLimitNum = 4;
		return INVERSE_KIN_NOT_REACHABLE;
	}
    return RBTINTERF_NO_ERROR;
}

ErrorID RobotModel::calcSwivelAngle(const Position_ACS_rad& posACS,double* swivel_angle) const
{
	Position_ACS_rad q(posACS);
    for (unsigned int i = 0; i < axes.size(); i++)
    {
        q[i] = q[i] * axes[i]->axisDirection;
    }
    Matrix4 currentMatrix(Matrix4::Identity( 4, 4));
	if (rbt_type == R_GENERAL_7S_RBT)
	{
	    Matrix4 mat[5],mat0[5];
	    Matrix4 mat12,mat24,mat26,mat120,mat260;
		for(int i=0;i<5;i++)
		{
			if (axes[i])
			{
			    DH_Parameters dhParam;
				dhParam = axes[i]->DHJ;
				dhParam.Theta = 0 + dhParam.Theta;
				mat0[i] = AllCompute(dhParam,currentMatrix);
				dhParam.Theta = q(i) + dhParam.Theta;// theta = posACS + offset
				mat[i] = AllCompute(dhParam,currentMatrix);
			}
		}
		mat12 = mat[0];
		mat24 = mat[0]*mat[1]*mat[2];
		mat26 = mat[0]*mat[1]*mat[2]*mat[3]*mat[4];
		mat120 = mat0[0];
		mat260 = mat0[0]*mat0[1]*mat0[2]*mat0[3]*mat0[4];
		Vector3d X24;
		Vector3d X26;
		Vector3d X26_y0,X260;
		for(int i=0;i<3;i++)
		{
			X24(i) = mat24(i,3) - mat12(i,3);
			X26(i) = mat26(i,3) - mat12(i,3);
			X26_y0(i) = mat26(i,3) - mat12(i,3);
			X260(i) = mat260(i,3) - mat120(i,3);
		}
		X26_y0(1) = 0;
		X260(1) = 0;
		double cos26 = X26_y0.dot(X260)/X26_y0.norm()/X260.norm();
		double sin26;
		if(X26_y0(0)<X260(0))
		{
			if(X26_y0(2)<-X260(2))
				sin26 = sqrt(1-cos26*cos26);
			else
				sin26 = -sqrt(1-cos26*cos26);
		}
		else
			sin26 = sqrt(1-cos26*cos26);
		Vector3d X2N = X24.dot(X26) / X26.norm() * (X26 / X26.norm());
		Vector3d XN4 = X24-X2N; //求当前246平面内，关节4到向量26的垂直向量
		Vector3d V;
		if(X26(1) == 0)
			V<<sin26,0,cos26;
		else
			V<<0,0,1;
		Vector3d X2N1 = V.dot(X26) / X26.norm() * (X26 / X26.norm());
		Vector3d XNV = V - X2N1; //求臂角为0时，与 关节4到向量26的垂直向量 平行的向量

	    double temp_value = XN4.dot(XNV)/(XN4.norm()*XNV.norm()); //两向量夹角即为当前臂角

	    if(temp_value < -1 && temp_value >= -1-1e-10)
	        temp_value = -1;
	    else if(temp_value < -1-1e-10)
	    {
	        printf("swivel_angle can not be solved\n");
	        return FORWARD_KIN_ARG_ERROR;
	    }

	    if(temp_value > 1 && temp_value <= 1+1e-10)
	        temp_value = 1;
	    else if(temp_value > 1+1e-10)
	    {
	        printf("swivel_angle can not be solved\n");
	        return FORWARD_KIN_ARG_ERROR;
	    }

		double result1 = acos(temp_value);
		double result2 = -acos(temp_value);
		if(q(3) < (theta4_when_zero_pos - M_PI))
		{
			result1 = result1 + M_PI;
			result2 = result2 + M_PI;
		}
		{ //验证result1和result2哪个是正确的臂角
		    Matrix4 mat_single_axis[7];
		    Matrix4 mat_all;
			for(int i=0;i<7;i++)
			{
				if (axes[i])
				{
				    DH_Parameters dhParam;
					dhParam = axes[i]->DHJ;
					dhParam.Theta = q(i) + dhParam.Theta;// theta = posACS + offset
					mat_single_axis[i] = AllCompute(dhParam,currentMatrix);
				}
			}
			mat_all = mat_single_axis[0]*mat_single_axis[1]*mat_single_axis[2]*mat_single_axis[3]*mat_single_axis[4]*mat_single_axis[5]*mat_single_axis[6];

			Position_MCS_rad pos(6);
			for (int i=0;i<6;i++)
			{
			    pos[i]=0;
			}
			if (upsideDown == true)
			{
		        pos[4]=M_PI;
		        pos[5]=M_PI;
			}

			mat_all = rpy2tr(pos) * mat_all * toolMatrix;

			Position_ACS_rad acs_result1(7);
			Position_ACS_rad acs_result2(7);
			ErrorID lRet1, lRet2;
			CalculateParameter calculateParameter = CalculateParameter();
			bool forward_check = true;
			bool optimize = false;
			lRet1 = calcInverseKin_Trans_7s(mat_all, posACS, acs_result1 ,result1, calculateParameter, forward_check, optimize);
			lRet2 = calcInverseKin_Trans_7s(mat_all, posACS, acs_result2 ,result2, calculateParameter, forward_check, optimize);
			if(lRet1 == RBTINTERF_NO_ERROR && lRet2 == RBTINTERF_NO_ERROR)
        	{
            	double total_error1 = 0;
            	double total_error2 = 0;
    			for(int i = 0;i < 7;i++)
    			{
    				total_error1 += fabs(acs_result1[i] - posACS[i]);
    				total_error2 += fabs(acs_result2[i] - posACS[i]);
    			}

    			{ //此处是为了避免关节结果超过+-180带来的异常结果
        			while(total_error1 > (2 * M_PI - 1e-10))
        				total_error1 -= 2 * M_PI;
        			while(total_error2 > (2 * M_PI - 1e-10))
        				total_error2 -= 2 * M_PI;
    			}

    			if(total_error1 < total_error2)
    				*swivel_angle = result1;
    			else
    				*swivel_angle = result2;
        	}
        	else
        	{
        		if(lRet1 == RBTINTERF_NO_ERROR)
        		{
    				*swivel_angle = result1;
        		}
        		else
        		{
    				*swivel_angle = result2;
        		}
        	}
		}
		return RBTINTERF_NO_ERROR;
	}
	else
		return INVERSE_KIN_ARG_ERROR;
}
double old_swivel_angle = 0;
double old_Y = 0;
double old_X = 0;
ErrorID RobotModel::calcInverseKin_Trans_7s(const Matrix4& transMatrix,
        const Position_ACS_rad& posLast, Position_ACS_rad& posACS ,double swivel_angle,
		const CalculateParameter& calculateParameter, bool forward_check, bool optimize) const
{
	if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
			!=7)||(posACS.rows() !=7) ||(axes.size()!=7))
		return INVERSE_KIN_TRANS_ARG_ERROR;
	Matrix4 T(transMatrix);
	T *= toolMatrix.inverse();

	for(int i=0;i<3;i++)
		T(i,3)=T(i,3)/1000;
	Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值
	double d1=axes[0]->DHJ.D/1000;
	double d3=axes[2]->DHJ.D/1000;
	double d5=axes[4]->DHJ.D/1000;
	double d7=axes[6]->DHJ.D/1000;
	double a3=axes[2]->DHJ.A/1000;
	double a4=axes[3]->DHJ.A/1000;
	double offset[7];
	for(int i=0;i<7;i++)
		offset[i]=axes[i]->DHJ.Theta;

	double pos_limit_min[7];
	double pos_limit_max[7];
	for(int i = 0;i < 7;i++)
	{
		pos_limit_min[i] = (axes[i]->axisDirection == 1)?(axes[i]->paramLmt.negSWLimit):(-1 * axes[i]->paramLmt.posSWLimit);
		pos_limit_max[i] = (axes[i]->axisDirection == 1)?(axes[i]->paramLmt.posSWLimit):(-1 * axes[i]->paramLmt.negSWLimit);
	}

	Vector3d X18;
	Vector3d X28;
	for(int i=0;i<3;i++)
		X18[i]=T(i,3);
	X28 = X18;
	X28[2] = X28[2] - d1;
	Matrix3 R18;
	R18(0,0)=T(0,0);
	R18(0,1)=T(0,1);
	R18(0,2)=T(0,2);
	R18(1,0)=T(1,0);
	R18(1,1)=T(1,1);
	R18(1,2)=T(1,2);
	R18(2,0)=T(2,0);
	R18(2,1)=T(2,1);
	R18(2,2)=T(2,2);

	Vector3d X24,X46,X12,X23, X78,X26;
	X24<<a3,0,d3;
	X46<<d5,0,a4;
	X12<<0,0,d1;
	X78<<0,0,d7;

	double d24=X24.norm();
	double d46=X46.norm();
	X26=X18-R18*X78-X12;
	double d26=X26.norm();

	double temp_value = (d24*d24+d46*d46-d26*d26)/(2*d24*d46);

	if(temp_value < -1 && temp_value >= -1-1e-10)
		temp_value = -1;
	else if(temp_value < -1-1e-10)
	{
		if(!forward_check)
		{
			printf("theta4 can not be solved\n");
		}
        jointLimitNum = 4;
        return INVERSE_KIN_NOT_REACHABLE;
	}

	if(temp_value > 1 && temp_value <= 1+1e-10)
		temp_value = 1;
	else if(temp_value > 1+1e-10)
	{
		if(!forward_check)
		{
			printf("theta4 can not be solved\n");
		}
        jointLimitNum = 4;
        return INVERSE_KIN_NOT_REACHABLE;
	}

	double tempTheta4=acos(temp_value);
	double theta4_1 = theta4_when_zero_pos - tempTheta4;
	double theta4_2 = theta4_when_zero_pos - (2 * M_PI - tempTheta4);

	double theta4;

	if(calculateParameter.configuration == 1)
	{
		if(theta4_1 > (theta4_when_zero_pos - M_PI))
			theta4 = theta4_1;
		else
			theta4 = theta4_2;
	}
	else if(calculateParameter.configuration == 2)
	{
		if(theta4_1 < (theta4_when_zero_pos - M_PI))
			theta4 = theta4_1;
		else
			theta4 = theta4_2;
	}
	else
		theta4 = calcRealAngle(pLast(3), theta4_1, theta4_2);

	posACS[3] = theta4;

	if((posACS[3] < M_PI*pos_limit_min[3]/180.0L) || (posACS[3] > M_PI*pos_limit_max[3]/180.0L))
	{
		posACS[3] = pLast(3);
		if(!forward_check)
		{
			printf("theta4 exceeds pos limit.\n");
		}
		jointLimitNum = 4;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	/// m2 * cos(theta2) - m1 * sin(theta2) = -X26(2)
	double a_3, d_4, k_4, delta;
	a_3 = -d3 + d5*cos(theta4 - M_PI / 2) + a4 * sin(theta4 - M_PI / 2);
	d_4 = -(a3 + a4*cos(theta4 - M_PI/2) - d5*sin(theta4 - M_PI/2));
	k_4 = -X26(2);
	delta = sqrt(a_3*a_3 + d_4*d_4 - k_4*k_4);

	if(delta<0)
	{
		jointLimitNum = 2;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	double theta1_30 = atan2(X26(1),X26(0));
	double theta2_30[2];
	theta2_30[0] = -(atan2(d_4,a_3)+atan2(delta, k_4));
	theta2_30[1] = -(atan2(d_4,a_3)-atan2(delta, k_4));

	Matrix3 R120,R230[2],R340,R140[2];
	R120<<cos(theta1_30 + offset[0]),0,-sin(theta1_30 + offset[0]),
			sin(theta1_30 + offset[0]),0,cos(theta1_30 + offset[0]),
			0,-1,0;
	R340<<cos(0 + offset[2]),0,-sin(0 + offset[2]),
			sin(0 + offset[2]),0,cos(0 + offset[2]),
			0,-1,0;
	for(int i = 0;i < 2;i++)
	{
		R230[i]<<cos(theta2_30[i] + offset[1]),0,sin(theta2_30[i] + offset[1]),
				sin(theta2_30[i] + offset[1]),0,-cos(theta2_30[i] + offset[1]),
				0,1,0;
		R140[i] = R120 * R230[i] * R340;
	}

	Vector3d X26_basis = X26/X26.norm();
	Matrix3 X26_basis_M,I;
	X26_basis_M<<0,-X26_basis(2),X26_basis(1),
				X26_basis(2),0,-X26_basis(0),
				-X26_basis(1),X26_basis(0),0;
	I<<1,0,0,
	   0,1,0,
	   0,0,1;
	Matrix3 Rz;
	Rz = I + sin(swivel_angle) * X26_basis_M + (1-cos(swivel_angle)) * X26_basis_M * X26_basis_M;

	Matrix3 tempM[2];
	double theta2_temp[2 * 2];

	for(int i = 0;i < 2;i++)
	{
		tempM[i] = Rz * R140[i];
	    double temp_value_1 = -tempM[i](2,1);

	    if(temp_value_1 < -1 && temp_value_1 >= -1-1e-10)
	        temp_value_1 = -1;
	    else if(temp_value_1 < -1-1e-10)
	    {
	    	if(!forward_check)
			{
	    		printf("theta2 can not be solved\n");
			}
	        jointLimitNum = 2;
	        return INVERSE_KIN_NOT_REACHABLE;
	    }

	    if(temp_value_1 > 1 && temp_value_1 <= 1+1e-10)
	        temp_value_1 = 1;
	    else if(temp_value_1 > 1+1e-10)
	    {
	    	if(!forward_check)
			{
	    		printf("theta2 can not be solved\n");
			}
	        jointLimitNum = 2;
	        return INVERSE_KIN_NOT_REACHABLE;
	    }

		theta2_temp[i * 2] = acos(temp_value_1);
		theta2_temp[i * 2 + 1] = -acos(temp_value_1);
	}
	double theta1_temp[2], theta3_temp[2];
	double theta1[2 * 2], theta3[2 * 2], theta2[2 * 2];
	double theta5[2 * 2], theta6[2 * 2], theta7[2 * 2];
	double temp_result[7];
	double theta_final_standby[2][7];
	int correct_num = 0;
	double theta_final[7];
	for(int i = 0;i < 4;i++)
	{
		if(fabs(theta2_temp[i])<1e-10)
		{
			theta1_temp[0] = pLast(0);
		}
		else
		{
			theta1_temp[0] = atan2(-tempM[i / 2](1,1)/sin(theta2_temp[i]),-tempM[i / 2](0,1)/sin(theta2_temp[i]));
			theta1_temp[1] = theta1_temp[0];
		}
		theta1[i] = calcRealAngle(pLast(0), theta1_temp[0], theta1_temp[1]);

		if(fabs(theta2_temp[i])<1e-10)
		{
			theta3_temp[0] = pLast(2);
			theta3_temp[1] = pLast(2);
		}
		else
		{
			theta3_temp[0] = atan2(tempM[i / 2](2,2)/sin(theta2_temp[i]),-tempM[i / 2](2,0)/sin(theta2_temp[i]));
			theta3_temp[1] = theta3_temp[0];
		}
		theta3[i] = calcRealAngle(pLast(2), theta3_temp[0], theta3_temp[1]);
		temp_result[0] = theta1[i];
		temp_result[1] = theta2_temp[i];
		temp_result[2] = theta3[i];
		temp_result[3] = theta4;
		Matrix4 temp_matrix(Matrix4::Identity( 4, 4));
		for(int j = 0;j < 4;j++)
		{
		    DH_Parameters dhParam;
			dhParam = axes[j]->DHJ;

			dhParam.Theta = temp_result[j] + dhParam.Theta;// theta = posACS + offset
			temp_matrix = AllCompute(dhParam,temp_matrix);
		}
		Matrix3 R15 = temp_matrix.block(0,0,3,3);
		Matrix3 R58 = R15.inverse() * R18;
		if(fabs(fabs(R58(2,2)) - 1) < 1e-7)
		{
			theta5[i] = pLast(4);
			theta7[i] = pLast(6);
		}
		else
		{
			double theta51 = atan2(-R58(1,2),-R58(0,2));
			double theta52 = atan2(R58(1,2),R58(0,2));
			double theta61;
			double theta62;
			if(fabs(R58(1,2))<1e-7)
			{
				theta61 = atan2(-R58(2,2),-R58(0,2)/cos(theta51));
				theta62 = atan2(-R58(2,2),-R58(0,2)/cos(theta52));
			}
			else
			{
				theta61 = atan2(-R58(2,2),-R58(1,2)/sin(theta51));
				theta62 = atan2(-R58(2,2),-R58(1,2)/sin(theta52));
			}
			if((theta61 < M_PI*pos_limit_min[5]/180.0L) || (theta61 > M_PI*pos_limit_max[5]/180.0L))
			{
				theta5[i] = calcRealAngle(pLast(4), theta52, theta52);
				theta6[i] = theta62;
			}
			else if((theta62 < M_PI*pos_limit_min[5]/180.0L) || (theta62 > M_PI*pos_limit_max[5]/180.0L))
			{
				theta5[i] = calcRealAngle(pLast(4), theta51, theta51);
				theta6[i] = theta61;
			}
			else
			{
				theta5[i] = calcRealAngle(pLast(4), theta51, theta52);
				if(fabs(theta5[i] - theta51) < 1e-10 || fabs(fabs(theta5[i] - theta51) - 2 * M_PI) < 1e-10)
					theta6[i] = theta61;
				else
					theta6[i] = theta62;
			}

			double theta71 = atan2(-R58(2,1),R58(2,0));
			double theta72 = atan2(R58(2,1),-R58(2,0));
			if(fabs(theta5[i] - theta51) < 1e-10 || fabs(fabs(theta5[i] - theta51) - 2 * M_PI) < 1e-10)
				theta7[i] = theta71;
			else
				theta7[i] = theta72;
		}

		Position_ACS_rad temp_result_pos(7);
		Matrix4 check_trans;
		temp_result_pos(0) = theta1[i];
		temp_result_pos(1) = theta2_temp[i];
		temp_result_pos(2) = theta3[i];
		temp_result_pos(3) = theta4;
		temp_result_pos(4) = theta5[i];
		temp_result_pos(5) = theta6[i];
		temp_result_pos(6) = theta7[i];

		double temp_swivel_angle = 0;
		double* pAngle = &temp_swivel_angle;
		calcTransMatrix(temp_result_pos, check_trans, pAngle, 0, true);
		bool result_correct = true;
		for(int m = 0;m < 4;m++)
		{
			for (int n = 0;n < 4;n++)
			{
				if(fabs(check_trans(m,n) - transMatrix(m,n)) > 1e-2)
				{
					result_correct = false;
					break;
				}
			}
			if(!result_correct)
				break;
		}
		if(result_correct)
		{
			for(int j = 0;j < 7;j++)
			{
				while(temp_result_pos[j] < -2 * M_PI)
					temp_result_pos[j] += 2 * M_PI;
				while(temp_result_pos[j] > 2 * M_PI)
					temp_result_pos[j] -= 2 * M_PI;
				theta_final_standby[correct_num][j] = temp_result_pos[j];
			}
			correct_num++;
			if(correct_num == 2)
				break;
		}
	}
	if(correct_num == 0) //所有组解验证都不对，根据就近原则选取一组。如果没有这部分，会导致在零点位置无法向Y方向点动。原因是二轴0度是奇异点
	{
    	if(!forward_check)
		{
    		printf("correct_num is 0\n");
		}
		jointLimitNum = 0;
		return INVERSE_KIN_NOT_REACHABLE;
	}
	else if(correct_num == 1)
	{
		for(int i = 0;i < 7;i++)
		{
			theta_final[i] = theta_final_standby[0][i];
		}
	}
	else
	{
		bool result_1_over_limit = false;
		for(int i = 0;i < 7;i++)
		{
			if((theta_final_standby[0][i] < M_PI*pos_limit_min[i]/180.0L) ||
					(theta_final_standby[0][i] > M_PI*pos_limit_max[i]/180.0L))
			{
				correct_num--;
				result_1_over_limit = true;
				break;
			}
		}
		for(int i = 0;i < 7;i++)
		{
			if((theta_final_standby[1][i] < M_PI*pos_limit_min[i]/180.0L) ||
					(theta_final_standby[1][i] > M_PI*pos_limit_max[i]/180.0L))
			{
				correct_num--;
				break;
			}
		}
		if(correct_num == 1)
		{
			if(result_1_over_limit)
			{
				for (int i = 0;i < 7;i++)
					theta_final[i] = theta_final_standby[1][i];
			}
			else
			{
				for (int i = 0;i < 7;i++)
					theta_final[i] = theta_final_standby[0][i];
			}
		}
		else
		{
			theta_final[1] = calcRealAngle(pLast(1), theta_final_standby[0][1], theta_final_standby[1][1]);
			if(fabs(theta_final[1] - theta_final_standby[0][1]) < 1e-10 || fabs(fabs(theta_final[1] - theta_final_standby[0][1]) - 2 * M_PI) < 1e-10)
			{
				theta_final[0] = calcRealAngle(pLast(0), theta_final_standby[0][0], theta_final_standby[1][0]);
				if(fabs(theta_final[0] - theta_final_standby[0][0]) < 1e-10)
				{
					for(int i = 0;i < 7;i++)
					{
						theta_final[i] = calcRealAngle(pLast(i), theta_final_standby[0][i], theta_final_standby[0][i]);
					}
				}
				else
				{
					for(int i = 0;i < 7;i++)
					{
						theta_final[i] = calcRealAngle(pLast(i), theta_final_standby[1][i], theta_final_standby[1][i]);
					}
				}
			}
			else
			{
				theta_final[0] = calcRealAngle(pLast(0), theta_final_standby[0][0], theta_final_standby[1][0]);
				if(fabs(theta_final[0] - theta_final_standby[1][0]) < 1e-10 || fabs(fabs(theta_final[0] - theta_final_standby[1][0]) - 2 * M_PI) < 1e-10)
				{
					for(int i = 0;i < 7;i++)
					{
						theta_final[i] = calcRealAngle(pLast(i), theta_final_standby[1][i], theta_final_standby[1][i]);
					}
				}
				else
				{
					for(int i = 0;i < 7;i++)
					{
						theta_final[i] = calcRealAngle(pLast(i), theta_final_standby[0][i], theta_final_standby[0][i]);
					}
				}
			}
		}
	}

	bool already_modify = false;
	//这部分是否保留待商榷。
	//1.如果保留，会出现满足条件时（点动）计算出的直角坐标与传入的直角坐标有偏差的现象
	//2.如果不保留，会出现靠近零点附近时点动X或Y报超速的现象
	if(!forward_check && fabs(swivel_angle) < 1e-3 && old_swivel_angle == swivel_angle
			&& ((old_Y != T(1,3) && (fabs(old_X - T(0,3)) < 1e-10))
					|| (old_X != T(0,3) && (fabs(old_Y - T(1,3)) < 1e-10))))
	{
		if(!(((theta_final[0] + theta_final[2]) < M_PI*pos_limit_min[0]/180.0L) || ((theta_final[0] + theta_final[2]) > M_PI*pos_limit_max[0]/180.0L)))
		{
			already_modify = true;
			theta_final[0] = theta_final[0] + theta_final[2];
			theta_final[2] = pLast(2);
		}
	}
	if(!forward_check)
	{
		if(!already_modify)
		{
			if(fabs(theta_final[1]) < 1e-6)
			{
				if(!(((theta_final[0] + theta_final[2]) < M_PI*pos_limit_min[0]/180.0L) || ((theta_final[0] + theta_final[2]) > M_PI*pos_limit_max[0]/180.0L)))
				{
					theta_final[0] = theta_final[0] + theta_final[2];
					theta_final[2] = pLast(2);
				}
			}
		}
	}

    if(!IS_VALID_DOUBLE(theta_final[0]) || !IS_VALID_DOUBLE(theta_final[1]) ||
    	!IS_VALID_DOUBLE(theta_final[2]) || !IS_VALID_DOUBLE(theta_final[3]) ||
    	!IS_VALID_DOUBLE(theta_final[4]) || !IS_VALID_DOUBLE(theta_final[5]) ||
    	!IS_VALID_DOUBLE(theta_final[6]))
    {
    	if(!forward_check)
		{
    		printf("inverse calculate result is nan\n");
		}
		jointLimitNum = 0;
		return INVERSE_KIN_NOT_REACHABLE;
    }


    if(optimize)
    {
    	for(int i = 0;i < 7;i++)
    	{
    		if((theta_final[i] < M_PI*pos_limit_min[i]/180.0L))
    		{
        		theta_final[i] += 2 * M_PI;
        		if((theta_final[i] < M_PI*pos_limit_min[i]/180.0L)
        				|| (theta_final[i] > M_PI*pos_limit_max[i]/180.0L))
        		{
        	    	if(!forward_check)
        			{
        	    		printf("theta%i exceeds pos limit.\n", i + 1);
        			}
        	        jointLimitNum = i + 1;
        	        return INVERSE_KIN_NOT_REACHABLE;
        		}
    		}
    		else if((theta_final[i] > M_PI*pos_limit_max[i]/180.0L))
    		{
        		theta_final[i] -= 2 * M_PI;
        		if((theta_final[i] < M_PI*pos_limit_min[i]/180.0L)
        				|| (theta_final[i] > M_PI*pos_limit_max[i]/180.0L))
        		{
        	    	if(!forward_check)
        			{
        	    		printf("theta%i exceeds pos limit.\n", i + 1);
        			}
        	        jointLimitNum = i + 1;
        	        return INVERSE_KIN_NOT_REACHABLE;
        		}
    		}
    	}
    }
    else
    {
     	for(int i = 0;i < 7;i++)
	{
		if((theta_final[i] < M_PI*pos_limit_min[i]/180.0L) ||
				(theta_final[i] > M_PI*pos_limit_max[i]/180.0L))
		{
	    	if(!forward_check)
			{
	    		printf("theta%i exceeds pos limit.\n", i + 1);
			}
	        jointLimitNum = i + 1;
	        return INVERSE_KIN_NOT_REACHABLE;
		}
	} 
    }

	for(int i=0;i<7;i++)
	{
		posACS(i) = theta_final[i];
	}
	if(!forward_check)
	{
	    old_swivel_angle = swivel_angle;
		old_Y = T(1,3);
		old_X = T(0,3);
	}
	return RBTINTERF_NO_ERROR;
}

ErrorID RobotModel::calcInverseKin_Trans_SIXAXIS_SPRAY_BBR(const Matrix4& transMatrix,
        const Position_ACS_rad& posLast, Position_ACS_rad& posACS , bool optimize) const
{

    if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
                !=6)||(posACS.rows() !=6) ||(axes.size()!=6))
    {
        return INVERSE_KIN_TRANS_ARG_ERROR;
    }

    Matrix4 T(transMatrix);
    T *= toolMatrix.inverse();

    Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

    double nx(T(0,0)), ny(T(1,0)), nz(T(2,0));
    double ox(T(0,1)), oy(T(1,1)), oz(T(2,1));
    double ax(T(0,2)), ay(T(1,2)), az(T(2,2));
    double px(T(0,3)/1000.0), py(T(1,3)/1000.0), pz(T(2,3)/1000.0);

    //vars defined temporarily
    double l1=axes[0]->DHJ.D/1000;
    double l2=axes[1]->DHJ.A/1000;
    double l3=axes[2]->DHJ.A/1000;
    double l4=axes[4]->DHJ.D/1000;
    double l5=axes[5]->DHJ.D/1000;
    double l6=axes[0]->DHJ.A/1000;
    double l7=axes[1]->DHJ.D/1000;
    double thetaOffset3 = axes[2]->DHJ.Theta;
    double thetaOffset4 = axes[3]->DHJ.Theta;
    double smax,smin,lmax,lmin,umax,umin,rmax,rmin,bmax,bmin,tmax,tmin;
    //第一轴角度限制范围
    smax=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.posSWLimit):(-1 * axes[0]->paramLmt.negSWLimit);
    smin=(axes[0]->axisDirection == 1)?(axes[0]->paramLmt.negSWLimit):(-1 * axes[0]->paramLmt.posSWLimit);
    //第二轴角度限制范围
    lmax=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.posSWLimit):(-1 * axes[1]->paramLmt.negSWLimit);
    lmin=(axes[1]->axisDirection == 1)?(axes[1]->paramLmt.negSWLimit):(-1 * axes[1]->paramLmt.posSWLimit);
    //第三轴角度限制范围
    umax=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.posSWLimit):(-1 * axes[2]->paramLmt.negSWLimit);
    umin=(axes[2]->axisDirection == 1)?(axes[2]->paramLmt.negSWLimit):(-1 * axes[2]->paramLmt.posSWLimit);
    //第四轴角度限制范围
    rmax=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.posSWLimit):(-1 * axes[3]->paramLmt.negSWLimit);
    rmin=(axes[3]->axisDirection == 1)?(axes[3]->paramLmt.negSWLimit):(-1 * axes[3]->paramLmt.posSWLimit);
    //第五轴角度限制范围
    bmax=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.posSWLimit):(-1 * axes[4]->paramLmt.negSWLimit);
    bmin=(axes[4]->axisDirection == 1)?(axes[4]->paramLmt.negSWLimit):(-1 * axes[4]->paramLmt.posSWLimit);
    //第六轴角度限制范围
    tmax=(axes[5]->axisDirection == 1)?(axes[5]->paramLmt.posSWLimit):(-1 * axes[5]->paramLmt.negSWLimit);
    tmin=(axes[5]->axisDirection == 1)?(axes[5]->paramLmt.negSWLimit):(-1 * axes[5]->paramLmt.posSWLimit);

    auto outofAxisRange = [=](double val, double min, double max) -> bool {
        return (val < min || val > max);
    };

    //solve for theta1
    double theta1_1,theta1_2;
    double k1,k2;
    double theta1;
    k1 = px - l5 * ax;
    k2 = l5 * ay - py;

    double temp = pow(k1,2) + pow(k2,2) - pow(l7,2);
    if(temp < 0.0L)
    {
        printf("Theta1 can not be solved, so can not reach the point!\n");
        jointLimitNum = 1;
        return INVERSE_KIN_NOT_REACHABLE;
    }

    theta1_1 = atan2(l7,sqrt(temp)) - atan2(k2,k1);
    theta1_2 = atan2(l7,-sqrt(temp)) - atan2(k2,k1);
    theta1 = calcRealAngle(pLast(0), theta1_1, theta1_2);

//    printf("1=%f,%f\n",theta1_1/M_PI * 180,theta1_2/M_PI * 180);
//    printf("1=%f\n",theta1/M_PI * 180);
    _Optimal:
    // the limit of theta1 according to the reference
    if((theta1 < M_PI*smin/180.0L) || (theta1 > M_PI*smax/180.0L))
    {
        theta1 = pLast(0);
        printf("theta1 exceeds pos limit.\n");
        if (optimize == true)
        {
            optimize = false;
            theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
            goto _Optimal;
        }
        jointLimitNum = 1;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(0) = theta1;

    //solve for theta3
    k1 =  pz - l1 - l5*az;
    k2 = cos(theta1)*px+sin(theta1)*py-l6-l5*(cos(theta1)*ax+sin(theta1)*ay);
    double k3 = pow(az,2) + pow((ax*cos(theta1)+ay*sin(theta1)),2);
    if (fabs(k3)<10e-5)
    {
        printf("theta3 exceeds pos limit.5axis==90\n");
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
    }

     double k3_1 = -sqrt(k3);
     double k3_2 = sqrt(k3);

    double theta2;
    double theta4;
    double theta3,theta3_1,theta3_2,theta3_3,theta3_4,theta3_5,theta3_6;
    double theta3Cache[4];

    k3 = (pow((k1+l4*(cos(theta1)*ax+sin(theta1)*ay)/(-k3_1)), 2) + pow((k2-l4*az/(-k3_1)), 2) - pow(l2, 2) - pow(l3, 2))/(2*l2*l3);
    int  theta3CacheNum = 0;
    if (fabs(k3)<=1)
    {
        theta3_1 = atan2(sqrt(1-pow(k3,2)), k3) - thetaOffset3;
        theta3_2 = -atan2(sqrt(1-pow(k3,2)), k3) - thetaOffset3;
//        printf("3=%f,%f\n",theta3_1/M_PI * 180,theta3_2/M_PI * 180);
        theta3_5 = calcRealAngle(pLast(2), theta3_1, theta3_2);
        theta3Cache[0] = theta3_5;
        theta3Cache[1] = theta3_5;
        theta3CacheNum = 1;

        k3 = (pow((k1+l4*(cos(theta1)*ax+sin(theta1)*ay)/(-k3_2)), 2) + pow((k2-l4*az/(-k3_2)), 2) - pow(l2, 2) - pow(l3, 2))/(2*l2*l3);
        if (fabs(k3)<=1)
        {
            theta3_3 = atan2(sqrt(1-pow(k3,2)), k3) - thetaOffset3;
            theta3_4 = -atan2(sqrt(1-pow(k3,2)), k3) - thetaOffset3;
//            printf("3=%f,%f\n",theta3_3/M_PI * 180,theta3_4/M_PI * 180);
            theta3_6 = calcRealAngle(pLast(2), theta3_3, theta3_4);
            theta3Cache[2] = theta3_6;
            theta3Cache[3] = theta3_6;
            theta3CacheNum = 2;
//            theta3 = calcRealAngle(pLast(2), theta3_5, theta3_6);
        }
//        else
//        {
//            theta3 = theta3_5;
//        }
    }
    else
    {
        k3 = (pow((k1+l4*(cos(theta1)*ax+sin(theta1)*ay)/(-k3_2)), 2) + pow((k2-l4*az/(-k3_2)), 2) - pow(l2, 2) - pow(l3, 2))/(2*l2*l3);
        if (fabs(k3)<=1)
        {
            theta3_3 = atan2(sqrt(1-pow(k3,2)), k3) - thetaOffset3;
            theta3_4 = -atan2(sqrt(1-pow(k3,2)), k3) - thetaOffset3;
            theta3 = calcRealAngle(pLast(2), theta3_3, theta3_4);
            theta3Cache[0] = theta3;
            theta3Cache[1] = theta3;
            theta3CacheNum = 1;
        }
        else
        {
            printf("theta3 exceeds pos limit.5axis==90\n");
            if (optimize == true)
            {
                optimize = false;
                theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
                goto _Optimal;
            }
            jointLimitNum = 3;
            return INVERSE_KIN_NOT_REACHABLE;
        }
    }
    double theta2Cache[4];
    double theta4Cache[4];

    for(int i = 0; i < theta3CacheNum; i++)
    {
         k3 = l3 + l2 * cos(theta3Cache[2*i] + thetaOffset3);
         double k4 = l2 * sin(theta3Cache[2*i] + thetaOffset3);
         double theta2_1, theta2_2;
         double k5 = 2*k1*k4-2*k2*k3;
         double k6 = 2*k1*k3+2*k2*k4;
         double k7 = pow(l4,2) - pow(k1,2) - pow(k2,2) - pow(k3,2) - pow(k4,2);

         temp = pow(k5,2) + pow(k6,2) - pow(k7,2);
         if(temp < 0.0L)
           {
             theta2Cache[2*i] = 999;
             theta2Cache[2*i+1] = 999;

             theta4Cache[2*i] = 999;
             theta4Cache[2*i+1] = 999;
           }
         else
         {
           theta2_1 = -atan2(k7,sqrt(temp))+atan2(k5,k6) - (theta3Cache[2*i] + thetaOffset3) -  M_PI/2.0;
           theta2_2 = -atan2(k7,-sqrt(temp))+atan2(k5,k6) - (theta3Cache[2*i] + thetaOffset3) -  M_PI/2.0;
           theta2_1 = calcRealAngle(pLast(1), theta2_1, theta2_1);
           theta2_2 = calcRealAngle(pLast(1), theta2_2, theta2_2);

           theta2Cache[2*i] = theta2_1;
           {
               //solve for theta4
             double k8 = k1*sin(theta2_1+theta3Cache[2*i] + thetaOffset3 + M_PI/2.0)+k2*cos(theta2_1+theta3Cache[2*i] + thetaOffset3 + M_PI/2.0)-k3;
             double k9 = -k1*cos(theta2_1+theta3Cache[2*i] + thetaOffset3 + M_PI/2.0)+k2*sin(theta2_1+theta3Cache[2*i] + thetaOffset3 + M_PI/2.0)-k4;
             double theta4_1;
               if((fabs(k8) < 10e-13)  && (fabs(k9) < 10e-13))
                   theta4_1 = pLast(3);
               else
                   theta4_1 = atan2(k8,k9) - thetaOffset4;
               theta4_1 = calcRealAngle(pLast(3), theta4_1, theta4_1);
               theta4Cache[2*i] = outofAxisRange(theta4_1, rmin/180.0L*M_PI, rmax/180.0L*M_PI) ? 999 : theta4_1;
           }
           theta2Cache[2*i+1] = theta2_2;
           {
               //solve for theta4
             double k8 = k1*sin(theta2_2+theta3Cache[2*i] + thetaOffset3 + M_PI/2.0)+k2*cos(theta2_2+theta3Cache[2*i] + thetaOffset3 + M_PI/2.0)-k3;
             double k9 = -k1*cos(theta2_2+theta3Cache[2*i] + thetaOffset3 + M_PI/2.0)+k2*sin(theta2_2+theta3Cache[2*i] + thetaOffset3 + M_PI/2.0)-k4;
             double theta4_1;
               if((fabs(k8) < 10e-13)  && (fabs(k9) < 10e-13))
                   theta4_1 = pLast(3);
               else
                   theta4_1 = atan2(k8,k9) - thetaOffset4;
               theta4_1 = calcRealAngle(pLast(3), theta4_1, theta4_1);
               theta4Cache[2*i+1] = outofAxisRange(theta4_1, rmin/180.0L*M_PI, rmax/180.0L*M_PI) ? 999 : theta4_1;
           }
         }
//          printf("2=%f,%f\n",theta2_1/M_PI * 180,theta2_2/M_PI * 180);
    }
    double length = 0;
    int selectNum = -1;
    for(int i = 0; i < 2*theta3CacheNum; i++)
    {
        double temp1 =  (cos(theta1)*ax+sin(theta1)*ay)*sin(theta2Cache[i] + theta3Cache[i] + theta4Cache[i])-az*cos(theta2Cache[i] + theta3Cache[i] + theta4Cache[i]);
        if (fabs(temp1) < 10e-7)
        {
            if (selectNum == -1)
            {
                length = fabs(theta2Cache[i] - pLast(1)) + fabs(theta3Cache[i] - pLast(2)) + fabs(theta4Cache[i] - pLast(3));
                selectNum = i;
            }
            else
            {
                if (length > fabs(theta2Cache[i] - pLast(1)) + fabs(theta3Cache[i] - pLast(2)) + fabs(theta4Cache[i] - pLast(3)))
                {
                    length = fabs(theta2Cache[i] - pLast(1)) + fabs(theta3Cache[i] - pLast(2)) + fabs(theta4Cache[i] - pLast(3));
                    selectNum = i;
                }
            }
        }
    }
    if (selectNum == -1)
    {
        if (optimize == true)
        {
            optimize = false;
            theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
            goto _Optimal;
        }
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    theta2 = theta2Cache[selectNum];
    theta3 = theta3Cache[selectNum];
    theta4 = theta4Cache[selectNum];

//    printf("33333333333333=%f,%f,%f\n",theta2/M_PI * 180,theta3/M_PI * 180,theta4/M_PI * 180);

    // the limit of theta3 according to the reference
    if((theta3 < umin/180.0L*M_PI) || (theta3 > umax/180.0L*M_PI))
    {
        theta3 = pLast(2);
        printf("theta3 exceeds pos limit.\n");
        if (optimize == true)
        {
            optimize = false;
            theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
            goto _Optimal;
        }
        jointLimitNum = 3;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(2) = theta3;

        // the limit of theta2 according to the reference
        if((theta2 < lmin/180.0L*M_PI) || (theta2 > lmax/180.0L*M_PI))
        {
            theta2 = pLast(1);
            printf("theta2 exceeds pos limit.\n");
            if (optimize == true)
            {
                optimize = false;
                theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
                goto _Optimal;
            }
            jointLimitNum = 2;
            return INVERSE_KIN_NOT_REACHABLE;
        }

        posACS(1) = theta2;
        // the limit of theta4 according to the reference
        if((theta4 < rmin*M_PI/180.0L) || (theta4 > rmax*M_PI/180.0L))
        {
            theta4 = pLast(3);
            printf("theta4 exceeds pos limit.\n");
            if (optimize == true)
            {
                optimize = false;
                theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
                goto _Optimal;
            }
            jointLimitNum = 4;
            return INVERSE_KIN_NOT_REACHABLE;
        }
        posACS(3) = theta4;

//    printf("theta2 = %lf,theta2_1 =%lf,theta2_1=%lf,ax=%lf,ay=%lf,az=%lf,temp=%lf,temp1=%.10lf,temp2=%.10lf\n",theta2,theta2_1,theta2_2,ax,ay,az,atan2(k7,sqrt(temp)),temp1,temp2);
    //solve for theta5
    double k1_1 = az*sin(theta2 + theta3 + theta4) + ax*cos(theta1)*cos(theta2 + theta3 + theta4 )
                  + ay*sin(theta1)*cos(theta2 + theta3 + theta4);
    double k1_2 = -ax*sin(theta1)+ay*cos(theta1);

    double theta5_1;
    if ((fabs(k1_1) < 10e-13) && (fabs(k1_2) < 10e-13))
    {
        theta5_1 = pLast(4);
    }
    else
    {
        theta5_1 = atan2(k1_1, k1_2) - M_PI/2.0;
    }
    double theta5;

    theta5 = calcRealAngle(pLast(4), theta5_1, theta5_1);
    // the limit of theta5 according to the reference
    if((theta5 < bmin/180.0L*M_PI) || (theta5 > bmax/180.0L*M_PI))
    {
        theta5 = pLast(4);
        printf("theta5 exceeds pos limit.\n");
        if (optimize == true)
        {
            optimize = false;
            theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
            goto _Optimal;
        }
        jointLimitNum = 5;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(4) = theta5;

    //solve for theta6
    double k1_3=-nz*cos(theta2 + theta3 + theta4) + nx*cos(theta1)*sin(theta2 + theta3 + theta4)
                +ny*sin(theta1)*sin(theta2 + theta3 + theta4);
    double k1_4=-oz*cos(theta2 + theta3 + theta4 ) + ox*cos(theta1)*sin(theta2 + theta3 + theta4)
                + oy*sin(theta1)*sin(theta2 + theta3 + theta4);

    double theta6_1;
    if((fabs(k1_3) < 10e-13) && (fabs(k1_4) < 10e-13))
        theta6_1 = pLast(5);
    else
        theta6_1 = atan2(k1_3, k1_4);
    double theta6;

    theta6 = calcRealAngle(pLast(5), theta6_1, theta6_1);
    // the limit of theta6 according to the reference
    if((theta6 < tmin/180.0L*M_PI) || (theta6 > tmax/180.0L*M_PI))
    {
        theta6 = pLast(5);
        printf("theta6 exceeds pos limit.\n");
        if (optimize == true)
        {
            optimize = false;
            theta1 = calcLargeAngle(pLast(0), theta1_1, theta1_2);
            goto _Optimal;
        }
        jointLimitNum = 6;
        return INVERSE_KIN_NOT_REACHABLE;
    }
    posACS(5) = theta6;

    return RBTINTERF_NO_ERROR;
}
/*****************************************************************************/
/**
 * @brief 20点标定算法
 * @param[in] pointArray 输入点列矩阵
 * @param[out] dTheta  零点offset补偿角度
 * @param[out] distance 补偿后Tcp末端误差球半径
 * @param[out] toolOffset 工具手坐标系xyz偏移
 * @return 0表示成功，-1表示失败
 * @attention 
 * @note   过程中调用了求伪逆矩阵函数 pseudoInverse(),求拟合球心坐标和半径的函数  sphereFit()
 * @todo
 */
/*****************************************************************************/

int RobotModel::calculateOffsetCompensation(dmatrix& pointArray, Position_ACS_rad& dTheta, double& distance, Position& toolOffset)
{
	int n = 6;
	dvector alphaV(n), aV(n), dV(n), offsetV(n);
	for (int i=0; i<n; i++)
	{
		alphaV(i) = axes[i]->DHJ.Alpha;  //单位：弧度rad
		offsetV(i) = axes[i]->DHJ.Theta;               //单位：弧度rad
		aV(i) = axes[i]->DHJ.A;
		dV(i) = axes[i]->DHJ.D;
	}
	Position_ACS_rad  posACS(n);
	Matrix4 transMatrix;
	dmatrix xyzArray(20,3);
	dmatrix xyzArrayTool(20,3);
	dmatrix coeArray(n+3,3);
	dmatrix XT(30,n+3);
	dmatrix XTtool(30,3);
	dmatrix expPos(30,1);
	Matrix4 dA1, dA2, dA3, dA4, dA5, dA6;
	Matrix4 A1, A2, A3, A4, A5, A6;
	Matrix4 C1, C2, C3, C4, C5, C6;
	std::vector<DH_Parameters> dhParam(n);
	Vector3 xyz1,xyz2,xyz3,xyz4,xyz5,xyz6,xyz7,xyz8,xyz9;
	std::vector<Matrix4> Bvec(n);
	std::vector<dmatrix> coeVec(20);

	dmatrix X(20,3);
	dvector Rslt(4);
	Position_ACS_rad  posUpgradeACS(n);
	Matrix4 transMatrixUpgrade;
	dmatrix xyzArrayUpgrade(20,3);

	Vector4 vecSphereCentre(4),vecSphereCentreZ(4), vecTool2Flange(4);
	toolMatrix = Matrix4::Identity(4, 4); //确保此时工具坐标系为单位矩阵
	//计算矩阵A的微分
	for (int i=0; i<n; i++)
	{
		Bvec[i] <<         0,       -cos(alphaV(i)),     sin(alphaV(i)),                       0,
		              cos(alphaV(i)),                  0,                0,      aV(i)*cos(alphaV(i)),
		             -sin(alphaV(i)),                  0,                0,     -aV(i)*sin(alphaV(i)),
		                         0,                   0,                0,                         0;
	}
	//***************************************先计算toolMatrix*******************************************************
	for (int i=0; i<20; i++)
	{
		posACS = pointArray.row(i);
		calcTransMatrix(posACS, transMatrix);
		for (int j=0; j<3; j++)
		{
			xyzArrayTool(i,j) = transMatrix(j,3);  //20个理论位置
		}

		if (n == 6)
		{
			xyz1 = transMatrix.block(0,0,3,1);    //取transMatrix的第一列前3个元素作为dTx的3个系数
			xyz2 = transMatrix.block(0,1,3,1);    //取transMatrix的第二列前3个元素作为dTy的3个系数
			xyz3 = transMatrix.block(0,2,3,1);    //取transMatrix的第三列前3个元素作为dTz的3个系数
			coeArray.row(0) = xyz1;
			coeArray.row(1) = xyz2;
			coeArray.row(2) = xyz3;
		}
		else
		{
#if (defined VXWORKS)
			logMsg((char*)"Error: The number of axes is not 6 !!\n", 0,0,0,0,0,0);
#endif /* VXWORKS */

#if (defined LINUX) || (defined SYLIXOS)
			printf((char*)"Error: The number of axes is not 6 !!\n");
#endif /* LINUX SYLIXOS */
			return -1;
		}
		coeVec[i] = coeArray;
	}

	for (int k=0; k<10; k++)
	{
		for (int i=0; i<3; i++)
		{
			XTtool(3*k,  i)=coeVec[k+10](i,0) - coeVec[k](i,0); //		xx=dT{1,10+k}(1,4)-dT{1,k}(1,4);
			XTtool(3*k+1,i)=coeVec[k+10](i,1) - coeVec[k](i,1);	//		yy=dT{1,10+k}(2,4)-dT{1,k}(2,4);
			XTtool(3*k+2,i)=coeVec[k+10](i,2) - coeVec[k](i,2);	//		zz=dT{1,10+k}(3,4)-dT{1,k}(3,4);
		}
	}

	for (int k=0; k<10; k++)
	{
		expPos(3*k,  0) = xyzArrayTool(k,0) - xyzArrayTool(k+10,0);
		expPos(3*k+1,0) = xyzArrayTool(k,1) - xyzArrayTool(k+10,1);
		expPos(3*k+2,0) = xyzArrayTool(k,2) - xyzArrayTool(k+10,2);
	}

	dmatrix pinvXTtool(3,30);
	pseudoInverse(XTtool, pinvXTtool);

	dvector dThetaTool(3);
	dThetaTool = pinvXTtool * expPos;
//	logMsg((char*)"dThetaTool=%d,%d,%d\n",dThetaTool(0),dThetaTool(1),dThetaTool(2),0,0,0);
	if (!IS_VALID_DOUBLE(dThetaTool(0)) || !IS_VALID_DOUBLE(dThetaTool(1)) || !IS_VALID_DOUBLE(dThetaTool(2)))
	{
#if (defined VXWORKS)
			logMsg((char*)"Error: dThetaTool=NaN!!\n", 0,0,0,0,0,0);
#endif /* VXWORKS */

#if (defined LINUX) || (defined SYLIXOS)
			printf(((char*)"Error: dThetaTool=NaN!!\n"));
#endif /* LINUX SYLIXOS */
		return -1;
	}
	//更新工具手
	for (int i=0; i<3; i++)
	{
	   toolMatrix(i,3) = dThetaTool(i);
	}

	//***************************************计算dThetaAll*********************************************************
	for (int i=0; i<20; i++)
	{
		posACS = pointArray.row(i);
		calcTransMatrix(posACS, transMatrix);
		for (int j=0; j<3; j++)
		{
			xyzArray(i,j) = transMatrix(j,3);  //20个理论位置
		}

		for (int di=0; di<n; di++)
		{
			dhParam[di].A = aV(di);
			dhParam[di].D = dV(di);
			dhParam[di].Alpha = alphaV(di);
			dhParam[di].Theta = posACS(di) + offsetV(di);      //注意pointArray.row(i) + offsetV.transpose();
		}
		if (n == 6)
		{
			A1 = Compute(dhParam[0]);
			C1 = Compute(dhParam[1])*Compute(dhParam[2])*Compute(dhParam[3])*Compute(dhParam[4])*Compute(dhParam[5])*toolMatrix;

			A2 = Compute(dhParam[0])*Compute(dhParam[1]);
			C2 = Compute(dhParam[2])*Compute(dhParam[3])*Compute(dhParam[4])*Compute(dhParam[5])*toolMatrix;

			A3 = Compute(dhParam[0])*Compute(dhParam[1])*Compute(dhParam[2]);
			C3 = Compute(dhParam[3])*Compute(dhParam[4])*Compute(dhParam[5])*toolMatrix;

			A4 = Compute(dhParam[0])*Compute(dhParam[1])*Compute(dhParam[2])*Compute(dhParam[3]);
			C4 = Compute(dhParam[4])*Compute(dhParam[5])*toolMatrix;

			A5 = Compute(dhParam[0])*Compute(dhParam[1])*Compute(dhParam[2])*Compute(dhParam[3])*Compute(dhParam[4]);
			C5 = Compute(dhParam[5])*toolMatrix;

			A6 = Compute(dhParam[0])*Compute(dhParam[1])*Compute(dhParam[2])*Compute(dhParam[3])*Compute(dhParam[4])*Compute(dhParam[5]);
			C6 = Matrix4::Identity( 4, 4)*toolMatrix;
			dT(A1, Bvec[0], C1, xyz1);  //求dT矩阵中位置矢量x/y/z各分量有关dTheta1的3个系数
			dT(A2, Bvec[1], C2, xyz2);  //求dT矩阵中位置矢量x/y/z各分量有关dTheta2的3个系数
			dT(A3, Bvec[2], C3, xyz3);
			dT(A4, Bvec[3], C4, xyz4);
			dT(A5, Bvec[4], C5, xyz5);
			dT(A6, Bvec[5], C6, xyz6);
			xyz7 = A6.block(0,0,3,1);    //取A6的第一列前3个元素作为dTx的3个系数
			xyz8 = A6.block(0,1,3,1);    //取A6的第二列前3个元素作为dTy的3个系数
			xyz9 = A6.block(0,2,3,1);    //取A6的第三列前3个元素作为dTz的3个系数
			coeArray.row(0) = xyz1;
			coeArray.row(1) = xyz2;
			coeArray.row(2) = xyz3;
			coeArray.row(3) = xyz4;
			coeArray.row(4) = xyz5;
			coeArray.row(5) = xyz6;
			coeArray.row(6) = xyz7;
			coeArray.row(7) = xyz8;
			coeArray.row(8) = xyz9;
		}
		else
		{
#if (defined VXWORKS)
			logMsg((char*)"Error: The number of axes is not 6!!\n", 0,0,0,0,0,0);
#endif /* VXWORKS */

#if (defined LINUX) || (defined SYLIXOS)
			printf((char*)"Error: The number of axes is not 6!!\n");
#endif /* LINUX SYLIXOS */
			return -1;
		}
		coeVec[i] = coeArray;
	}

	for (int k=0; k<10; k++)
	{
		for (int i=0; i<n+3; i++)
		{
			XT(3*k,  i)=coeVec[k+10](i,0) - coeVec[k](i,0); //		xx=dT{1,10+k}(1,4)-dT{1,k}(1,4);
			XT(3*k+1,i)=coeVec[k+10](i,1) - coeVec[k](i,1);	//		yy=dT{1,10+k}(2,4)-dT{1,k}(2,4);
			XT(3*k+2,i)=coeVec[k+10](i,2) - coeVec[k](i,2);	//		zz=dT{1,10+k}(3,4)-dT{1,k}(3,4);
		}
	}
	for (int k=0; k<10; k++)
	{
		expPos(3*k,  0) = xyzArray(k,0) - xyzArray(k+10,0);
		expPos(3*k+1,0) = xyzArray(k,1) - xyzArray(k+10,1);
		expPos(3*k+2,0) = xyzArray(k,2) - xyzArray(k+10,2);
	}

	dmatrix pinvXT(n+3,30);
    pseudoInverse(XT, pinvXT);
    dvector dThetaAll(n+3);
	dThetaAll = pinvXT * expPos;
	if (!IS_VALID_DOUBLE(dThetaAll(0)) || !IS_VALID_DOUBLE(dThetaAll(1)) || !IS_VALID_DOUBLE(dThetaAll(2))
		|| !IS_VALID_DOUBLE(dThetaAll(3)) || !IS_VALID_DOUBLE(dThetaAll(4)) || !IS_VALID_DOUBLE(dThetaAll(5))
		|| !IS_VALID_DOUBLE(dThetaAll(6)) || !IS_VALID_DOUBLE(dThetaAll(7)) || !IS_VALID_DOUBLE(dThetaAll(8)))
	{
#if (defined VXWORKS)
			logMsg((char*)"Error: dTheta=NaN!!\n", 0,0,0,0,0,0);
#endif /* VXWORKS */

#if (defined LINUX) || (defined SYLIXOS)
			printf((char*)"Error: dTheta=NaN!!\n");
#endif /* LINUX SYLIXOS */
		return -1;
	}
	dTheta = dThetaAll.head(6);
	//更新工具手
	for (int i=0; i<3; i++)
	{
	    toolMatrix(i,3) += dThetaAll(i+6);
		toolOffset(i) = toolMatrix(i,3);
		if (fabs(toolOffset(i)) < 10e-4)
		{
			toolOffset(i) = 0;
		}
	}
	toolOffset(3) = 0;
	toolOffset(4) = 0;
	toolOffset(5) = 0;

	//评价计算结果
	for (int i=0; i<20; i++)  //拟合球心得工具手坐标系原点
	{
		posUpgradeACS = pointArray.row(i) + dTheta.transpose();     //误差补偿后
		calcTransMatrix(posUpgradeACS, transMatrixUpgrade);
		for (int j=0; j<3; j++)
		{
			X(i,j) = transMatrixUpgrade(j,3);  //20个补偿后的位置
		}
	}
    sphereFit(X, Rslt);
	vecSphereCentreZ(0) = Rslt(0)/2.0;
	vecSphereCentreZ(1) = Rslt(1)/2.0;
	vecSphereCentreZ(2) = Rslt(2)/2.0;
	distance = sqrt(pow(vecSphereCentreZ(0), 2) + pow(vecSphereCentreZ(1), 2) + pow(vecSphereCentreZ(2), 2) - Rslt(3));  //Tcp末端点误差球面半径

	return 0;
}

int RobotModel::toolCalibration20Position(dmatrix& pointArray, Position_ACS_rad& dTheta, double& distance, Position& toolOffset)
{
	Position_ACS_rad  posACS(6);
	Matrix4 transMatrix;
	std::vector<Point3D> points;
	double center_x;
	double center_y;
	double center_z;
	double radius;

	for (int i=0; i<18; i++)
	{
		posACS = pointArray.row(i);
		calcTransMatrix(posACS, transMatrix);

		Point3D point;
		point.x = transMatrix(0,3);
		point.y = transMatrix(1,3);
		point.z = transMatrix(2,3);
		points.push_back(point);
	}
	sphereLeastFit(points, center_x, center_y, center_z, radius);

	//更新工具手
    Vector4d vecSphereCentre(4),vecTool2Flange(4);
    Matrix4 matPos1;
    Position_MCS_rad tcp1(6);
    Position_MCS_rad tcp2(6);
    Position_MCS_rad tcp3(6);
	vecSphereCentre(0) = center_x;
	vecSphereCentre(1) = center_y;
	vecSphereCentre(2) = center_z;
	vecSphereCentre(3) = 1.0;

	posACS = pointArray.row(0);
	calcTransMatrix(posACS, matPos1);
	calcForwardKin(posACS, tcp1);
	posACS = pointArray.row(18);
	calcForwardKin(posACS, tcp2);
	posACS = pointArray.row(19);
	calcForwardKin(posACS, tcp3);

	vecTool2Flange = matPos1.inverse()*vecSphereCentre;

	toolOffset(0) = vecTool2Flange[0];
	toolOffset(1) = vecTool2Flange[1];
	toolOffset(2) = vecTool2Flange[2];

    Position_MCS_rad tcpFlange(6);
    Vector3 vecTemp1(3),vecTemp2(3),vecTemp3(3);

    vecTemp1(0) = tcp2[0] - tcp1[0];
    vecTemp1(1) = tcp2[1] - tcp1[1];
    vecTemp1(2) = tcp2[2] - tcp1[2];
    vecTemp2(0) = tcp3[0] - tcp1[0];
    vecTemp2(1) = tcp3[1] - tcp1[1];
    vecTemp2(2) = tcp3[2] - tcp1[2];

    vecTemp3 = vecTemp1.cross(vecTemp2);    //vector3=vector1叉乘vector2
    vecTemp2 = vecTemp3.cross(vecTemp1);    //vector2=vector3叉乘vector1

    vecTemp1.normalize();
    vecTemp2.normalize();
    vecTemp3.normalize();

    Matrix4 matPos5, matTool2Base, matTool2Flange;

    matTool2Base(0,0) = vecTemp1(0);
    matTool2Base(1,0) = vecTemp1(1);
    matTool2Base(2,0) = vecTemp1(2);
    matTool2Base(3,0) = 0.0;
    matTool2Base(0,1) = vecTemp2(0);
    matTool2Base(1,1) = vecTemp2(1);
    matTool2Base(2,1) = vecTemp2(2);
    matTool2Base(3,1) = 0.0;
    matTool2Base(0,2) = vecTemp3(0);
    matTool2Base(1,2) = vecTemp3(1);
    matTool2Base(2,2) = vecTemp3(2);
    matTool2Base(3,2) = 0.0;
    matTool2Base(0,3) = 0.0;
    matTool2Base(1,3) = 0.0;
    matTool2Base(2,3) = 0.0;
    matTool2Base(3,3) = 1.0;

    matPos5 = rpy2tr(tcp1);

    matTool2Flange = matPos5.inverse()*matTool2Base;
    tcpFlange = tr2MCS(matTool2Flange);
    for (AXIS_REFS_INDEX ai=0; ai<6; ++ai)
    {
        if (fabs(tcpFlange[ai]) < 10e-4)
        {
            tcpFlange[ai] = 0;
        }
    }
    for (AXIS_REFS_INDEX ai=0; ai<3; ++ai)
    {
        toolOffset[3+ai] = tcpFlange[3+ai];
    }

	for (int i=0; i<6; i++)
	{
		dTheta[i] = 0;
	}
	distance = 0;
	return 0;
}
/*****************************************************************************/
int RobotModel::calculateOffsetCompensation12Point(dmatrix& pointArray, Position_ACS_rad& dTheta, double& distance, Position& toolOffset)
{
	Position_ACS_rad dThetaTemp(6);
	for (int i=0; i<6; i++)
	{
		dThetaTemp[i] = 0.0;
		dTheta[i] = 0.0;
	}
	dmatrix pointArrayTemp(12,6);
	int cal_count = 0;

	cycle_calculation:
	pointArrayTemp = pointArray;

	for (int i=0; i<6; i++)
	{
		dTheta[i] += dThetaTemp[i];
	}

	for (int i=0; i < 12; i++)
	{
		pointArrayTemp(i,0) += dTheta[0];
		pointArrayTemp(i,1) += dTheta[1];
		pointArrayTemp(i,2) += dTheta[2];
		pointArrayTemp(i,3) += dTheta[3];
		pointArrayTemp(i,4) += dTheta[4];
		pointArrayTemp(i,5) += dTheta[5];
	}

	printf("dTheta[2]=%f,%f,%f,%f\n",rad2deg(dTheta[1]),rad2deg(dTheta[2]),rad2deg(dTheta[3]),rad2deg(dTheta[4]));
    //1.求工具手的xy
    dvector dThetaTool(3);
    Matrix4 transMatrix1;
    Matrix4 transMatrix2;
    Position pos1(6),pos2(6);

    pos1 = pointArrayTemp.row(0);
    pos2 = pointArrayTemp.row(1);

    toolMatrix = Matrix4::Identity(4, 4); //确保此时工具坐标系为单位矩阵

    calcTransMatrix(pos1, transMatrix1);
    calcTransMatrix(pos2, transMatrix2);
    double a,b,c,d,e,f;
    a = transMatrix1(0,0)-transMatrix2(0,0);
    b = transMatrix1(0,1)-transMatrix2(0,1);
    c = transMatrix1(1,0)-transMatrix2(1,0);
    d = transMatrix1(1,1)-transMatrix2(1,1);
    e = transMatrix2(0,3)-transMatrix1(0,3);
    f = transMatrix2(1,3)-transMatrix1(1,3);
    dThetaTool[0] = (e*d-b*f)/(a*d-b*c);
    dThetaTool[1] = (e*c-a*f)/(b*c-a*d);
    dThetaTool[2] = 0;
    printf("dThetaTool[0]=%f,dThetaTool[1]=%f\n",dThetaTool[0],dThetaTool[1]);

    //2.求工具手的z
    double zTool[3];
    pos1 = pointArrayTemp.row(2);
    pos2 = pointArrayTemp.row(3);

    calcTransMatrix(pos1, transMatrix1);
    calcTransMatrix(pos2, transMatrix2);

    for(int i = 0; i < 3; i++)
    {
        a = transMatrix1(0,0)-transMatrix2(0,0);
        b = transMatrix1(0,1)-transMatrix2(0,1);
        c = transMatrix1(0,2)-transMatrix2(0,2);
        d = transMatrix2(0,3)-transMatrix1(0,3);
        zTool[i] = (d - dThetaTool[0] * a - dThetaTool[1] * b) / c;
        printf("zTool[%d]=%f\n",i,zTool[i]);
    }

    dThetaTool[2] = zTool[0];
    printf("dThetaTool[2]=%f\n",dThetaTool[2]);
    //3.求2-5轴
    int n = 6;
    dvector alphaV(n), aV(n), dV(n), offsetV(n);
    for (int i=0; i<n; i++)
    {
        alphaV(i) = axes[i]->DHJ.Alpha;  //单位：弧度rad
        offsetV(i) = axes[i]->DHJ.Theta;               //单位：弧度rad
        aV(i) = axes[i]->DHJ.A;
        dV(i) = axes[i]->DHJ.D;
    }


    Position_ACS_rad  posACS(n);
    Matrix4 transMatrix;
    dmatrix xyzArray(8,3);
    dmatrix coeArray(4+3,3);
    dmatrix XT(12,4+3);
    dmatrix expPos(12,1);
    Matrix4 dA1, dA2, dA3, dA4, dA5, dA6;
    Matrix4 A1, A2, A3, A4, A5, A6;
    Matrix4 C1, C2, C3, C4, C5, C6;
    std::vector<DH_Parameters> dhParam(n);
    Vector3 xyz1,xyz2,xyz3,xyz4,xyz5,xyz6,xyz7,xyz8,xyz9;
    std::vector<Matrix4> Bvec(n);
    std::vector<dmatrix> coeVec(8);

    dmatrix X(8,3);
    dvector Rslt(4);
    Position_ACS_rad  posUpgradeACS(n);
    Matrix4 transMatrixUpgrade;

    Vector4 vecSphereCentre(4),vecSphereCentreZ(4), vecTool2Flange(4);

    //计算矩阵A的微分
    for (int i=0; i<n; i++)
    {
        Bvec[i] <<         0,       -cos(alphaV(i)),     sin(alphaV(i)),                       0,
                      cos(alphaV(i)),                  0,                0,      aV(i)*cos(alphaV(i)),
                     -sin(alphaV(i)),                  0,                0,     -aV(i)*sin(alphaV(i)),
                                 0,                   0,                0,                         0;
    }

    //更新工具手
    for (int i=0; i<3; i++)
    {
       toolMatrix(i,3) = dThetaTool(i);
    }

    //***************************************计算dThetaAll*********************************************************
    for (int i=0; i < 8; i++)
    {
        posACS = pointArrayTemp.row(i + 4);
        calcTransMatrix(posACS, transMatrix);
        for (int j=0; j<3; j++)
        {
            xyzArray(i,j) = transMatrix(j,3);  //8个理论位置
        }

        for (int di=0; di<n; di++)
        {
            dhParam[di].A = aV(di);
            dhParam[di].D = dV(di);
            dhParam[di].Alpha = alphaV(di);
            dhParam[di].Theta = posACS(di) + offsetV(di);      //注意pointArray.row(i) + offsetV.transpose();
        }
        if (n == 6)
        {
            A1 = Compute(dhParam[0]);
            C1 = Compute(dhParam[1])*Compute(dhParam[2])*Compute(dhParam[3])*Compute(dhParam[4])*Compute(dhParam[5])*toolMatrix;

            A2 = Compute(dhParam[0])*Compute(dhParam[1]);
            C2 = Compute(dhParam[2])*Compute(dhParam[3])*Compute(dhParam[4])*Compute(dhParam[5])*toolMatrix;

            A3 = Compute(dhParam[0])*Compute(dhParam[1])*Compute(dhParam[2]);
            C3 = Compute(dhParam[3])*Compute(dhParam[4])*Compute(dhParam[5])*toolMatrix;

            A4 = Compute(dhParam[0])*Compute(dhParam[1])*Compute(dhParam[2])*Compute(dhParam[3]);
            C4 = Compute(dhParam[4])*Compute(dhParam[5])*toolMatrix;

            A5 = Compute(dhParam[0])*Compute(dhParam[1])*Compute(dhParam[2])*Compute(dhParam[3])*Compute(dhParam[4]);
            C5 = Compute(dhParam[5])*toolMatrix;

            A6 = Compute(dhParam[0])*Compute(dhParam[1])*Compute(dhParam[2])*Compute(dhParam[3])*Compute(dhParam[4])*Compute(dhParam[5]);
            C6 = Matrix4::Identity( 4, 4)*toolMatrix;

            dT(A2, Bvec[1], C2, xyz2);  //求dT矩阵中位置矢量x/y/z各分量有关dTheta2的3个系数
            dT(A3, Bvec[2], C3, xyz3);
            dT(A4, Bvec[3], C4, xyz4);
            dT(A5, Bvec[4], C5, xyz5);

            xyz7 = A6.block(0,0,3,1);    //取A6的第一列前3个元素作为dTx的3个系数
            xyz8 = A6.block(0,1,3,1);    //取A6的第二列前3个元素作为dTy的3个系数
            xyz9 = A6.block(0,2,3,1);    //取A6的第三列前3个元素作为dTz的3个系数

            coeArray.row(0) = xyz2;
            coeArray.row(1) = xyz3;
            coeArray.row(2) = xyz4;
            coeArray.row(3) = xyz5;
            coeArray.row(4) = xyz7;
            coeArray.row(5) = xyz8;
            coeArray.row(6) = xyz9;
        }
        else
        {
#if (defined VXWORKS)
            logMsg((char*)"Error: The number of axes is not 6!!\n", 0,0,0,0,0,0);
#endif /* VXWORKS */

#if (defined LINUX) || (defined SYLIXOS)
            printf((char*)"Error: The number of axes is not 6!!\n");
#endif /* LINUX SYLIXOS */
            return -1;
        }
        coeVec[i] = coeArray;
    }

    for (int k=0; k< 4; k++)
    {
        for (int i=0; i<4+3; i++)
        {
            XT(3*k,  i)=coeVec[k+4](i,0) - coeVec[k](i,0); //      xx=dT{1,10+k}(1,4)-dT{1,k}(1,4);
            XT(3*k+1,i)=coeVec[k+4](i,1) - coeVec[k](i,1); //      yy=dT{1,10+k}(2,4)-dT{1,k}(2,4);
            XT(3*k+2,i)=coeVec[k+4](i,2) - coeVec[k](i,2); //      zz=dT{1,10+k}(3,4)-dT{1,k}(3,4);
        }
    }
    for (int k=0; k<4; k++)
    {
        expPos(3*k,  0) = xyzArray(k,0) - xyzArray(k+4,0);
        expPos(3*k+1,0) = xyzArray(k,1) - xyzArray(k+4,1);
        expPos(3*k+2,0) = xyzArray(k,2) - xyzArray(k+4,2);
    }

    dmatrix pinvXT(4+3,12);
    pseudoInverse(XT, pinvXT);
    dvector dThetaAll(4+3);
    dThetaAll = pinvXT * expPos;
    if (!IS_VALID_DOUBLE(dThetaAll(0)) || !IS_VALID_DOUBLE(dThetaAll(1)) || !IS_VALID_DOUBLE(dThetaAll(2))
        || !IS_VALID_DOUBLE(dThetaAll(3)) || !IS_VALID_DOUBLE(dThetaAll(4)) || !IS_VALID_DOUBLE(dThetaAll(5))
        || !IS_VALID_DOUBLE(dThetaAll(6)))
    {
#if (defined VXWORKS)
            logMsg((char*)"Error: dTheta=NaN!!\n", 0,0,0,0,0,0);
#endif /* VXWORKS */

#if (defined LINUX) || (defined SYLIXOS)
            printf((char*)"Error: dTheta=NaN!!\n");
#endif /* LINUX SYLIXOS */
        return -1;
    }
    dThetaTemp[0] = 0;
    dThetaTemp[5] = 0;
    for (int i=0; i<4; i++)
    {
        dThetaTemp[i+1] = dThetaAll[i];
    }
    printf("dTheta[2]=%f,%f,%f,%f\n",rad2deg(dThetaTemp[1]),rad2deg(dThetaTemp[2]),rad2deg(dThetaTemp[3]),rad2deg(dThetaTemp[4]));
    if (pointArray.rows() == 15)
    {
        Position posTool(6);
        Position_MCS_rad tcp1(6);
        Position_MCS_rad tcp2(6);
        Position_MCS_rad tcp3(6);
        Position_MCS_rad tcpFlange(6);
        Vector3 vecTemp1(3),vecTemp2(3),vecTemp3(3);

        toolMatrix = Matrix4::Identity(4, 4); //确保此时工具坐标系为单位矩阵

        posTool = pointArray.row(12) + dThetaTemp.transpose();
        calcForwardKin(posTool, tcp1);
        posTool = pointArray.row(13) + dThetaTemp.transpose();
        calcForwardKin(posTool, tcp2);
        posTool = pointArray.row(14) + dThetaTemp.transpose();
        calcForwardKin(posTool, tcp3);

        vecTemp1(0) = tcp2[0] - tcp1[0];
        vecTemp1(1) = tcp2[1] - tcp1[1];
        vecTemp1(2) = tcp2[2] - tcp1[2];
        vecTemp2(0) = tcp3[0] - tcp1[0];
        vecTemp2(1) = tcp3[1] - tcp1[1];
        vecTemp2(2) = tcp3[2] - tcp1[2];

        vecTemp3 = vecTemp1.cross(vecTemp2);    //vector3=vector1叉乘vector2
        vecTemp2 = vecTemp3.cross(vecTemp1);    //vector2=vector3叉乘vector1

        vecTemp1.normalize();
        vecTemp2.normalize();
        vecTemp3.normalize();

        Matrix4 matPos5, matTool2Base, matTool2Flange;

        matTool2Base(0,0) = vecTemp1(0);
        matTool2Base(1,0) = vecTemp1(1);
        matTool2Base(2,0) = vecTemp1(2);
        matTool2Base(3,0) = 0.0;
        matTool2Base(0,1) = vecTemp2(0);
        matTool2Base(1,1) = vecTemp2(1);
        matTool2Base(2,1) = vecTemp2(2);
        matTool2Base(3,1) = 0.0;
        matTool2Base(0,2) = vecTemp3(0);
        matTool2Base(1,2) = vecTemp3(1);
        matTool2Base(2,2) = vecTemp3(2);
        matTool2Base(3,2) = 0.0;
        matTool2Base(0,3) = 0.0;
        matTool2Base(1,3) = 0.0;
        matTool2Base(2,3) = 0.0;
        matTool2Base(3,3) = 1.0;

        matPos5 = rpy2tr(tcp1);

        matTool2Flange = matPos5.inverse()*matTool2Base;
        tcpFlange = tr2MCS(matTool2Flange);
        for (AXIS_REFS_INDEX ai=0; ai<6; ++ai)
        {
            if (fabs(tcpFlange[ai]) < 10e-4)
            {
                tcpFlange[ai] = 0;
            }
        }
        for (AXIS_REFS_INDEX ai=0; ai<3; ++ai)
        {
            toolOffset[3+ai] = tcpFlange[3+ai];
        }
    }
    else
    {
        for (AXIS_REFS_INDEX ai=0; ai<3; ++ai)
        {
            toolOffset[3+ai] = 0;
        }
    }

    //更新工具手
    for (int i=0; i<3; i++)
    {
        toolOffset(i) = dThetaTool(i) + dThetaAll(i+4);
        if (fabs(toolOffset(i)) < 10e-4)
        {
            toolOffset(i) = 0;
        }
    }
    printf("toolOffset=%f,%f,%f\n",toolOffset[0],toolOffset[1],toolOffset[2]);

    toolMatrix = rpy2tr(toolOffset);
    //评价计算结果
    for (int i=0; i<8; i++)  //拟合球心得工具手坐标系原点
    {
        posUpgradeACS = pointArrayTemp.row(i+4) + dThetaTemp.transpose();     //误差补偿后
        calcTransMatrix(posUpgradeACS, transMatrixUpgrade);
        for (int j=0; j<3; j++)
        {
            X(i,j) = transMatrixUpgrade(j,3);  //8个补偿后的位置
        }
    }
    sphereFit(X, Rslt);
    vecSphereCentreZ(0) = Rslt(0)/2.0;
    vecSphereCentreZ(1) = Rslt(1)/2.0;
    vecSphereCentreZ(2) = Rslt(2)/2.0;
    distance = sqrt(pow(vecSphereCentreZ(0), 2) + pow(vecSphereCentreZ(1), 2) + pow(vecSphereCentreZ(2), 2) - Rslt(3));  //Tcp末端点误差球面半径
    for (int i=0; i<8; i++)
    {
        if (distance < sqrt(pow(vecSphereCentreZ(0)-X(i,0), 2) + pow(vecSphereCentreZ(1)-X(i,1), 2) + pow(vecSphereCentreZ(2)-X(i,2), 2)))
        {
            distance = sqrt(pow(vecSphereCentreZ(0)-X(i,0), 2) + pow(vecSphereCentreZ(1)-X(i,1), 2) + pow(vecSphereCentreZ(2)-X(i,2), 2));
        }
    }
    printf("distance=%f\n",distance);

    if ((dThetaTemp[1] < 0.0001 && dThetaTemp[2] < 0.0001 && dThetaTemp[3] < 0.0001 && dThetaTemp[4] < 0.0001 ) || cal_count > 30)
    {
    	for (int i=0; i<6; i++)
    	{
    		dTheta[i] += dThetaTemp[i];
    	}
    	return 0;
    }
    cal_count++;
    goto cycle_calculation;

    return 0;
}

/*****************************************************************************/
/**
 * @brief 20点标定算法中，用来求中间量的系数
 * @param[in] A,B,C 输入矩阵
 * @param[out] coe 系数
 * @return 无
 */
/*****************************************************************************/
void RobotModel::dT(Matrix4& A, Matrix4& B, Matrix4& C, Vector3& coe)
{
	coe(0) = A(0,1)*B(1,3) + A(0,2)*B(2,3) + C(0,3)*(A(0,1)*B(1,0) + A(0,2)*B(2,0)) + A(0,0)*B(0,1)*C(1,3) + A(0,0)*B(0,2)*C(2,3);
	coe(1) = A(1,1)*B(1,3) + A(1,2)*B(2,3) + C(0,3)*(A(1,1)*B(1,0) + A(1,2)*B(2,0)) + A(1,0)*B(0,1)*C(1,3) + A(1,0)*B(0,2)*C(2,3);
	coe(2) = A(2,1)*B(1,3) + A(2,2)*B(2,3) + C(0,3)*(A(2,1)*B(1,0) + A(2,2)*B(2,0)) + A(2,0)*B(0,1)*C(1,3) + A(2,0)*B(0,2)*C(2,3);
}

/*****************************************************************************/
/**
 * @brief 采用SVD分解法求矩阵的伪逆矩阵
 * @param[in] XT 输入矩阵
 * @param[out] pinvXT 输出矩阵
 * @return 无
 * @attention 用len定义singularValuesInv的维数,有待确认，如果矩阵的秩<矩阵的行列维数呢？
 * @note  optional parameter allowing to specify if you want full or thin U or V unitaries to be computed. 
 * @note  By default, none is computed. This is a bit-field, the possible bits are ComputeFullU, ComputeThinU, ComputeFullV, ComputeThinV.
 * @todo
 */
/*****************************************************************************/
void RobotModel::pseudoInverse(dmatrix& XT, dmatrix& pinvXT)
{
	unsigned int option = 0;
	if(XT.rows() == XT.cols())
	{
		option =  Eigen::ComputeFullU | Eigen::ComputeFullV;
	}
	else
	{
		option =  Eigen::ComputeThinU | Eigen::ComputeThinV;
	}	
	Eigen::JacobiSVD<dmatrix> svd(XT, option);
    int len = XT.rows() < XT.cols() ? XT.rows() : XT.cols();
    dmatrix singularValuesInv(len,len);
    singularValuesInv.setZero();
    double  pinvtoler = 1.e-6; //  tolerance 
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > pinvtoler)
            singularValuesInv(i, i) = 1.0 / svd.singularValues()(i);
        else
            singularValuesInv(i, i) = 0;
    }
    pinvXT = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
}
/*****************************************************************************/
/**
 * @brief 拟合球心
 * @param[in] XYZ 直角坐标系下n个点列
 * @param[out] Result 球心坐标和半径的中间数据
 * @return 无
 * @attention XYZ矩阵每行代表一个点坐标
 * @note 过程中调用了求伪逆矩阵函数pseudoInverse()
 * @todo
 */
/*****************************************************************************/
void RobotModel::sphereFit(dmatrix& XYZ, dvector& Result)
{
	int n = XYZ.rows();
	dmatrix X(n,4), Q(4,4);
	dvector Y(n), P(4);
	dmatrix pinvQ(4,n);
	for (int i=0; i<n; i++)
	{
		for (int j=0; j<3; j++)
		{
			X(i,j) = XYZ(i,j);
		}
		X(i,3) = -1;
		Y(i) = X(i,0)*X(i,0) +  X(i,1)*X(i,1) + X(i,2)*X(i,2);
	}
	pseudoInverse(X, pinvQ);
    Result = pinvQ * Y;
}

Matrix4 RobotModel::Compute(DH_Parameters DH)
{
    Matrix4 X; /**< @brief Gets or sets the matrix regarding X axis transformations. */
    Matrix4 Z; /**< @brief Gets or sets the matrix regarding Z axis transformations. */
    // Calculate Z with Z = TranslationZ(d).RotationZ(theta)
    Z << Eigen::AngleAxisd(DH.Theta, Vector3::UnitZ()).toRotationMatrix(), (Vector3()<<0.0,0.0,DH.D).finished(),
            RowVector3::Zero(1, 3), 1;
    // Calculate X with X = TranslationX(radius).RotationZ(alpha)
    X << Eigen::AngleAxisd(DH.Alpha, Vector3::UnitX()).toRotationMatrix(), (Vector3()<<DH.A,0.0,0.0).finished(),
            RowVector3::Zero(1, 3), 1;
    // Calculate the transform with T=Z.X
    return Z*X;
}

Matrix4 RobotModel::AllCompute(DH_Parameters DH, Matrix4 transformMatrix) const
{
    Matrix4 X; /**< @brief Gets or sets the matrix regarding X axis transformations. */
    Matrix4 Z; /**< @brief Gets or sets the matrix regarding Z axis transformations. */
    // Calculate Z with Z = TranslationZ(d).RotationZ(theta)
    Z << Eigen::AngleAxisd(DH.Theta, Vector3::UnitZ()).toRotationMatrix(), (Vector3()<<0.0,0.0,DH.D).finished(),
            RowVector3::Zero(1, 3), 1;
    // Calculate X with X = TranslationX(radius).RotationZ(alpha)
    X << Eigen::AngleAxisd(DH.Alpha, Vector3::UnitX()).toRotationMatrix(), (Vector3()<<DH.A,0.0,0.0).finished(),
            RowVector3::Zero(1, 3), 1;
    // Calculate the transform with T=Z.X
    return transformMatrix*Z*X;
}

int RobotModel::userToBaseCalibration(UCSPOS_ORIENTATION &ucsPos1,UCSPOS_ORIENTATION &ucsPos2,UCSPOS_ORIENTATION &ucsPos3, UCSPOS_ORIENTATION *ucsToBase)
{
    using namespace Eigen;

    Position_MCS_rad ucsChange(6);
    Vector3 xAxisVector, yAxisVector,zAxisVector, origin;

    origin<< ucsPos1.x, ucsPos1.y, ucsPos1.z;
    xAxisVector << (ucsPos2.x - ucsPos1.x), (ucsPos2.y - ucsPos1.y), (ucsPos2.z - ucsPos1.z);
    yAxisVector << (ucsPos3.x - ucsPos1.x), (ucsPos3.y - ucsPos1.y), (ucsPos3.z - ucsPos1.z);
    zAxisVector = xAxisVector.cross(yAxisVector);
    yAxisVector = zAxisVector.cross(xAxisVector);

    xAxisVector.normalize();
    yAxisVector.normalize();
    zAxisVector.normalize();

    Matrix3 rotationMatrix,basicMatrix,ucsMatrix;
    Matrix4 changeMatrix;

    basicMatrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    ucsMatrix << xAxisVector, yAxisVector, zAxisVector;
    rotationMatrix = ucsMatrix * basicMatrix.reverse();
    changeMatrix << rotationMatrix, origin, dmatrix::Zero(1, 3), 1;

    ucsChange = tr2MCS(changeMatrix);

    for (AXIS_REFS_INDEX ai=0; ai<6; ++ai)
    {
        if (fabs(ucsChange[ai]) < 10e-4)
        {
            ucsChange[ai] = 0;
        }
    }

    (*ucsToBase).x=ucsChange(0);
    (*ucsToBase).y=ucsChange(1);
    (*ucsToBase).z=ucsChange(2);
    (*ucsToBase).A=ucsChange(3);
    (*ucsToBase).B=ucsChange(4);
    (*ucsToBase).C=ucsChange(5);

    if (IS_VALID_DOUBLE((*ucsToBase).x) && IS_VALID_DOUBLE((*ucsToBase).y) && IS_VALID_DOUBLE((*ucsToBase).z)
        && IS_VALID_DOUBLE((*ucsToBase).A) && IS_VALID_DOUBLE((*ucsToBase).B) && IS_VALID_DOUBLE((*ucsToBase).C))
        return 0;
    else
        return ERROR;
}

/****************************************************************************************
 * Function：    给定欧拉角和位置，转换成相应的齐次变换矩阵T(ZYZ)
 * Paraments：
 * posOrientation  --> current PosOrientation
 * pArrMatrix      --> 指向一个4*4的数组，用来存储变换矩阵T
 * Call：None
 * Called: xcTeachAc
 ****************************************************************************************/
void RobotModel::eulerToTrans(POS_ORIENTATION &posOrientation, double (*pArrMatrix)[4])
{
    double cA = cos(posOrientation.A*ANGLE_TO_RADIAN), sA = sin(posOrientation.A*ANGLE_TO_RADIAN),
           cB = cos(posOrientation.B*ANGLE_TO_RADIAN), sB = sin(posOrientation.B*ANGLE_TO_RADIAN),
           cC = cos(posOrientation.C*ANGLE_TO_RADIAN), sC = sin(posOrientation.C*ANGLE_TO_RADIAN);
    *(*(pArrMatrix + 0) + 0) =  cA*cB*cC - sA*sC;
    *(*(pArrMatrix + 0) + 1) = -cA*cB*sC - sA*cC;
    *(*(pArrMatrix + 0) + 2) =  cA*sB;
    *(*(pArrMatrix + 0) + 3) =  posOrientation.x;
    *(*(pArrMatrix + 1) + 0) =  sA*cB*cC + cA*sC;
    *(*(pArrMatrix + 1) + 1) = -sA*cB*sC + cA*cC;
    *(*(pArrMatrix + 1) + 2) =  sA*sB;
    *(*(pArrMatrix + 1) + 3) =  posOrientation.y;
    *(*(pArrMatrix + 2) + 0) = -sB*cC;
    *(*(pArrMatrix + 2) + 1) =  sB*sC;
    *(*(pArrMatrix + 2) + 2) =  cB;
    *(*(pArrMatrix + 2) + 3) =  posOrientation.z;
    *(*(pArrMatrix + 3) + 0) =  0;
    *(*(pArrMatrix + 3) + 1) =  0;
    *(*(pArrMatrix + 3) + 2) =  0;
    *(*(pArrMatrix + 3) + 3) =  1;
}
/*******************************************************************
 * Function：    给定齐次变换矩阵T，转换成相应的欧拉角和位置(ZYZ)
 * Paraments：
 * posOrientation  --> current PosOrientation
 * nextPosOrientation --> next PosOrientation
 * pArrMatrix      --> 指向一个4*4的数组 ，用来存储变换矩阵T
 * Call：None
 * Called：xcTeachAc
 ********************************************************************/
void RobotModel::transToEuler(double (*pArrMatrix)[4],POS_ORIENTATION &posOrientation, POS_ORIENTATION *nextPosOrientation)
{
    double cosBeta;
    cosBeta = *(*(pArrMatrix + 2) + 2);
    double r11,r12,r13;
    double r23;
    double r31,r32,r33;
    r11 = *(*(pArrMatrix + 0) + 0);
    r12 = *(*(pArrMatrix + 0) + 1);
    r13 = *(*(pArrMatrix + 0) + 2);
    r23 = *(*(pArrMatrix + 1) + 2);
    r31 = *(*(pArrMatrix + 2) + 0);
    r32 = *(*(pArrMatrix + 2) + 1);
    r33 = *(*(pArrMatrix + 2) + 2);
    /*利用齐次变换矩阵求解欧拉角的时候，beta = 0或90度时解就退化了，只能求解alpha与gamma*/
    /*beta = 0*/
    if(fabs(cosBeta - 1.0) < 10e-10)
    {
        (*nextPosOrientation).A = posOrientation.A;
        (*nextPosOrientation).B = acos(r33)*RADIAN_TO_ANGLE;
        (*nextPosOrientation).C = (atan2(-r12, r11) - posOrientation.A*ANGLE_TO_RADIAN)*RADIAN_TO_ANGLE;
        //printf("\ncondition_A\n");
    }
    else if(fabs(cosBeta + 1) < 10e-10)
    {
        (*nextPosOrientation).A = posOrientation.A;
        (*nextPosOrientation).B = acos(r33)*RADIAN_TO_ANGLE*posOrientation.A/fabs(posOrientation.A);
        (*nextPosOrientation).C = (atan2(r12, -r11) + posOrientation.A*ANGLE_TO_RADIAN)*RADIAN_TO_ANGLE;
        //printf("\ncondition_B\n");
    }
    else
    {
        /*solve alpha*/
        (*nextPosOrientation).A = (atan2(r23, r13))*RADIAN_TO_ANGLE;
        /*两解中选取合理的解*/
        double sign = (*nextPosOrientation).A/fabs((*nextPosOrientation).A);
        if(fabs((*nextPosOrientation).A - posOrientation.A) > 90)
            (*nextPosOrientation).A -= 180.0*sign;
        /*solve gamma*/
        (*nextPosOrientation).C = (atan2(r32, -r31))*RADIAN_TO_ANGLE;
        sign = (*nextPosOrientation).C/fabs((*nextPosOrientation).C);
        if(fabs((*nextPosOrientation).C - posOrientation.C) > 90)
            (*nextPosOrientation).C -= 180.0*sign;
        /*solve beta*/
        double sAlpha = sin((*nextPosOrientation).A*ANGLE_TO_RADIAN);
        double cAlpha = cos((*nextPosOrientation).A*ANGLE_TO_RADIAN);
        if(fabs(sAlpha) > 10e-4)
        {
            if((r23/sAlpha) > 0.0)
                sign =  1.0;
            else
                sign = -1.0;
        }
        else
        {
            if((r13/cAlpha) > 0.0)
                sign =  1.0;
            else
                sign = -1.0;
        }
        (*nextPosOrientation).B = sign*fabs((atan2(sqrt(r31*r31 + r32*r32), r33))*RADIAN_TO_ANGLE);
    }
    (*nextPosOrientation).x = *(*(pArrMatrix + 0) + 3);
    (*nextPosOrientation).y = *(*(pArrMatrix + 1) + 3);
    (*nextPosOrientation).z = *(*(pArrMatrix + 2) + 3);
}

void RobotModel::xcTeachAc(POS_ORIENTATION &endPosOrientation, POS_ORIENTATION &tcpPosOrientation, POS_ORIENTATION &dPosOrientation, POS_ORIENTATION *nextEndPosOrientation)
{
    using namespace Eigen;

    int i = 0,j = 0;
    /*分别用来存放Tcp、End-effector、delta、nextEnd-effector变换矩阵*/
    double arrEnd[4][4];
    double arrTcp[4][4];
    double arrD[4][4];
    double arrNextEnd[4][4];
    Matrix4 matEnd;
    Matrix4 matTcp;
    Matrix4 matD;
    Matrix4 matNextEnd;
    /*从欧拉、位置转换为标准的变换矩阵*/
    eulerToTrans(endPosOrientation, arrEnd);
    eulerToTrans(tcpPosOrientation, arrTcp);
    eulerToTrans(dPosOrientation, arrD);
    for(i = 0; i <= 3; i++)
        for(j = 0; j <= 3; j++)
        {
            matEnd(i,j) = *(*(arrEnd + i) + j);
            matTcp(i,j) = *(*(arrTcp + i) + j);
            matD(i,j) = *(*(arrD + i) + j);
        }
    matNextEnd = matEnd*matTcp*matD*matTcp.inverse();
    for(i = 0; i <= 3; i++)
        for(j = 0; j <= 3; j++)
        {
            arrNextEnd[i][j] = matNextEnd(i,j);
        }
    transToEuler(arrNextEnd, endPosOrientation, nextEndPosOrientation);
}
/***********************************************************************
 * 此版本的欧拉角解算函数作如下规定
 * alpha、gamma限制在[-pi,pi]，beta限制在[-pi/2,pi/2]
 * 这样就确定了解的唯一性，也能对应于任何一个方向的姿态
 ************************************************************************/
void RobotModel::transToEulerZYXv2(double (*pArrMatrix)[4], POS_ORIENTATION *nextPosOrientation)
{
    double sinBeta;
    sinBeta = -*(*(pArrMatrix + 2) + 0);
    double r11, r12, r13;
    double r21, r22, r23;
    double r31, r32, r33;
    r11 = *(*(pArrMatrix + 0) + 0) ;
    r12 = *(*(pArrMatrix + 0) + 1);
    r13 = *(*(pArrMatrix + 0) + 2);
    r21 = *(*(pArrMatrix + 1) + 0);
    r22 = *(*(pArrMatrix + 1) + 1);
    r23 = *(*(pArrMatrix + 1) + 2);
    r31 = *(*(pArrMatrix + 2) + 0);
    r32 = *(*(pArrMatrix + 2) + 1);
    r33 = *(*(pArrMatrix + 2) + 2);

    UNUSED_VARIABLE(r12);
    UNUSED_VARIABLE(r13);
    UNUSED_VARIABLE(r31);

    //防止asin()域的错误，beta(pitch)锁定在[-pi/2,pi/2]之间，
    if(sinBeta <= -1.0L)
    {
        (*nextPosOrientation).B = asin(-1.0L)*RADIAN_TO_ANGLE;
    }
    else if(sinBeta >= 1.0L)
    {
        (*nextPosOrientation).B = asin(1.0L)*RADIAN_TO_ANGLE;
    }
    else
    {
        (*nextPosOrientation).B = asin(sinBeta)*RADIAN_TO_ANGLE;
    }
    //检查万向锁的情况，允许一些误差
    if(fabs(sinBeta) > 0.9999999999L)
    {
        //向正上或正下看，将alpha置零，赋值给gamma,自由度将为2DOF
        (*nextPosOrientation).A = 0.0L;
        (*nextPosOrientation).C = atan2(-r23, r22)*RADIAN_TO_ANGLE;
    }
    else
    {
        (*nextPosOrientation).A = (atan2(r21, r11))*RADIAN_TO_ANGLE;
        (*nextPosOrientation).C = (atan2(r32, r33))*RADIAN_TO_ANGLE;
    }
    (*nextPosOrientation).x = *(*(pArrMatrix + 0) + 3);
    (*nextPosOrientation).y = *(*(pArrMatrix + 1) + 3);
    (*nextPosOrientation).z = *(*(pArrMatrix + 2) + 3);
}
/*************************************************************
 * Transform --> EulerZYX
 * 相关接口同ZYZ模式下的
 ***************************************************************/
void RobotModel::transToEulerZYX(double (*pArrMatrix)[4],POS_ORIENTATION &posOrientation, POS_ORIENTATION *nextPosOrientation)
{
    double sinBeta,sAlpha,cAlpha;
    sinBeta = -*(*(pArrMatrix + 2) + 0);
    double r11, r12, r13;
    double r21, r22, r23;
    double r31, r32, r33;
    r11 = *(*(pArrMatrix + 0) + 0);
    r12 = *(*(pArrMatrix + 0) + 1);
    r13 = *(*(pArrMatrix + 0) + 2);
    r21 = *(*(pArrMatrix + 1) + 0);
    r22 = *(*(pArrMatrix + 1) + 1);
    r23 = *(*(pArrMatrix + 1) + 2);
    r31 = *(*(pArrMatrix + 2) + 0);
    r32 = *(*(pArrMatrix + 2) + 1);
    r33 = *(*(pArrMatrix + 2) + 2);

    UNUSED_VARIABLE(r13);
    UNUSED_VARIABLE(r23);

    /*beta = 90,sinBeta = 1*/
    if(fabs(sinBeta - 1.0) < 10e-10)
    {
        (*nextPosOrientation).A = posOrientation.A;
        sAlpha = sin((*nextPosOrientation).A*ANGLE_TO_RADIAN);
        cAlpha = cos((*nextPosOrientation).A*ANGLE_TO_RADIAN);
        /*在beta=90.0附近运用acos()函数求解beta值*/
        if(fabs(sAlpha) > 10e-4)
        {
            (*nextPosOrientation).B = acos(r21/sAlpha)*RADIAN_TO_ANGLE;
        }
        else
        {
            (*nextPosOrientation).B = acos(r11/cAlpha)*RADIAN_TO_ANGLE;
        }
        (*nextPosOrientation).C = (atan2(r12, r22) + posOrientation.A*ANGLE_TO_RADIAN)*RADIAN_TO_ANGLE;
        /*修正某些特殊情况*/
        if((*nextPosOrientation).C > 180.0)
            (*nextPosOrientation).C = 360.0 - (*nextPosOrientation).C;
        else if((*nextPosOrientation).C < -180.0)
            (*nextPosOrientation).C += 360.0;
    }
    else if(fabs(sinBeta + 1) < 10e-10) /*beta = -90,sinBeta = -1*/
    {
        (*nextPosOrientation).A = posOrientation.A;
        sAlpha = sin((*nextPosOrientation).A*ANGLE_TO_RADIAN);
        cAlpha = cos((*nextPosOrientation).A*ANGLE_TO_RADIAN);
        /*在beta = -90.0附近运用acos()函数求解beta值*/
        if(fabs(sAlpha) > 10e-4)
        {
            (*nextPosOrientation).B = -acos(r21/sAlpha)*RADIAN_TO_ANGLE;
        }
        else
        {
            (*nextPosOrientation).B = -acos(r11/cAlpha)*RADIAN_TO_ANGLE;
        }

        (*nextPosOrientation).C = (atan2(-r12, r22) - posOrientation.A*ANGLE_TO_RADIAN)*RADIAN_TO_ANGLE;
        /*修正某些特殊情况*/
        if((*nextPosOrientation).C > 180.0)
            (*nextPosOrientation).C -= 360.0;
        else if((*nextPosOrientation).C < -180.0)
            (*nextPosOrientation).C += 360.0;
    }
    else
    {
        /*solve alpha*/
        (*nextPosOrientation).A = (atan2(r21, r11))*RADIAN_TO_ANGLE;
        /*两解中选取合理的解*/
        double sign = (*nextPosOrientation).A/fabs((*nextPosOrientation).A);
        if(fabs((*nextPosOrientation).A - posOrientation.A) > 90.0)
            (*nextPosOrientation).A -= 180.0*sign;
        /*solve gamma*/
        (*nextPosOrientation).C = (atan2(r32, r33))*RADIAN_TO_ANGLE;
        sign = (*nextPosOrientation).C/fabs((*nextPosOrientation).C);
        if(fabs((*nextPosOrientation).C - posOrientation.C) > 90.0)
            (*nextPosOrientation).C -= 180.0*sign;
        /*solve beta*/
        sAlpha = sin((*nextPosOrientation).A*ANGLE_TO_RADIAN);
        cAlpha = cos((*nextPosOrientation).A*ANGLE_TO_RADIAN);
        if(fabs(sAlpha) > 10e-4)
        {
            if((r21/sAlpha) > 0.0)
                sign =  1.0;  //fabs(Beta) < 90.0
            else
                sign = -1.0;
        }
        else
        {
            if((r11/cAlpha) > 0.0)
                sign =  1.0;
            else
                sign = -1.0;
        }
        (*nextPosOrientation).B = (atan2(-r31, sqrt(r32*r32 + r33*r33))*RADIAN_TO_ANGLE);
        /*根据sign的判断来修正beta值*/
        if((sign > 0.0) && (fabs((*nextPosOrientation).B) > 90.0 ))
        {
            if((*nextPosOrientation).B < 0.0)
                (*nextPosOrientation).B = -(*nextPosOrientation).B - 180.0;
            else
                (*nextPosOrientation).B = 180 - (*nextPosOrientation).B;
        }

        if((sign < 0.0) && (fabs((*nextPosOrientation).B) < 90.0 ))
        {
            if((*nextPosOrientation).B < 0.0)
                (*nextPosOrientation).B = -(*nextPosOrientation).B - 180.0;
            else
                (*nextPosOrientation).B = 180 - (*nextPosOrientation).B;
        }
    }
    (*nextPosOrientation).x = *(*(pArrMatrix + 0) + 3);
    (*nextPosOrientation).y = *(*(pArrMatrix + 1) + 3);
    (*nextPosOrientation).z = *(*(pArrMatrix + 2) + 3);
}

void RobotModel::eulerZYXToTrans(POS_ORIENTATION &posOrientation, double (*pArrMatrix)[4])
{
    double cA = cos(posOrientation.A*ANGLE_TO_RADIAN), sA = sin(posOrientation.A*ANGLE_TO_RADIAN),
           cB = cos(posOrientation.B*ANGLE_TO_RADIAN), sB = sin(posOrientation.B*ANGLE_TO_RADIAN),
           cC = cos(posOrientation.C*ANGLE_TO_RADIAN), sC = sin(posOrientation.C*ANGLE_TO_RADIAN);
//  *(*(pArrMatrix + 0) + 0) =  cA*cB;
//  *(*(pArrMatrix + 0) + 1) =  cA*sB*sC - sA*cC;
//  *(*(pArrMatrix + 0) + 2) =  cA*sB*cC + sA*sC;
//  *(*(pArrMatrix + 0) + 3) =  posOrientation.x;
//  *(*(pArrMatrix + 1) + 0) =  sA*cB;
//  *(*(pArrMatrix + 1) + 1) =  sA*sB*sC + cA*cC;
//  *(*(pArrMatrix + 1) + 2) =  sA*sB*cC - cA*sC;
//  *(*(pArrMatrix + 1) + 3) =  posOrientation.y;
//  *(*(pArrMatrix + 2) + 0) = -sB;
//  *(*(pArrMatrix + 2) + 1) =  cB*sC;
//  *(*(pArrMatrix + 2) + 2) =  cB*cC;
//  *(*(pArrMatrix + 2) + 3) =  posOrientation.z;
//  *(*(pArrMatrix + 3) + 0) =  0;
//  *(*(pArrMatrix + 3) + 1) =  0;
//  *(*(pArrMatrix + 3) + 2) =  0;
//  *(*(pArrMatrix + 3) + 3) =  1;
    *(*(pArrMatrix + 0) + 0) =  cA*cB;
    *(*(pArrMatrix + 0) + 1) =  -sA*cB;
    *(*(pArrMatrix + 0) + 2) =  sB;
    *(*(pArrMatrix + 0) + 3) =  posOrientation.x;
    *(*(pArrMatrix + 1) + 0) =  cA*sB*sC + sA*cC;
    *(*(pArrMatrix + 1) + 1) =  -sA*sB*sC + cA*cC;
    *(*(pArrMatrix + 1) + 2) =  -cB*sC;
    *(*(pArrMatrix + 1) + 3) =  posOrientation.y;
    *(*(pArrMatrix + 2) + 0) = -cA*sB*cC + sA*sC;
    *(*(pArrMatrix + 2) + 1) =  sA*sB*cC + cA*sC;
    *(*(pArrMatrix + 2) + 2) =  cB*cC;
    *(*(pArrMatrix + 2) + 3) =  posOrientation.z;
    *(*(pArrMatrix + 3) + 0) =  0;
    *(*(pArrMatrix + 3) + 1) =  0;
    *(*(pArrMatrix + 3) + 2) =  0;
    *(*(pArrMatrix + 3) + 3) =  1;
}

void RobotModel::xcTeachAcZYX(POS_ORIENTATION &endPosOrientation, POS_ORIENTATION &tcpPosOrientation, POS_ORIENTATION &dPosOrientation, POS_ORIENTATION *nextEndPosOrientation)
{
    using namespace Eigen;

    int i = 0,j = 0;
    /*分别用来存放Tcp、End-effector、delta、nextEnd-effector变换矩阵*/
    double arrEnd[4][4];
    double arrTcp[4][4] ;
    double arrD[4][4];
    double arrNextEnd[4][4];
    Matrix4 matEnd;
    Matrix4 matTcp;
    Matrix4 matD;
    Matrix4 matNextEnd;
    /*从欧拉、位置转换为标准的变换矩阵*/
    eulerZYXToTrans(endPosOrientation, arrEnd);
    eulerZYXToTrans(tcpPosOrientation, arrTcp);
    eulerZYXToTrans(dPosOrientation, arrD);
    for(i = 0; i <= 3; i++)
        for(j = 0; j <= 3; j++)
        {
            matEnd(i,j) = *(*(arrEnd + i) + j);
            matTcp(i,j) = *(*(arrTcp + i) + j);
            matD(i,j) = *(*(arrD + i) + j);
        }
    matNextEnd = matEnd*matTcp*matD*matTcp.inverse();
    for(i = 0; i <= 3; i++)
        for(j = 0; j <= 3; j++)
        {
            arrNextEnd[i][j] = matNextEnd(i,j);
        }
    transToEulerZYX(arrNextEnd, endPosOrientation, nextEndPosOrientation);
}
/**********************************************************************************
 * Function :convert from Quaternion to Homogeneous transformation matrix
 * Paraments:
 * In  :quat --> quaternion
 * Out :(*pArrMatrix)[4] --> store Homogeneous transformation matrix
 * Return:None
 * Reference:《3D数学基础：图形与游戏开发》
 *************************************************************************************/
void RobotModel::quaternionToTrans(QUATERNIONS quat, double (*pArrMatrix)[4])
{
    double w, x, y, z;
    w = quat.q0;
    x = quat.q1;
    y = quat.q2;
    z = quat.q3;
    *(*(pArrMatrix + 0) + 0) =  1 - 2*y*y - 2*z*z;
    *(*(pArrMatrix + 0) + 1) =  2*x*y + 2*w*z;
    *(*(pArrMatrix + 0) + 2) =  2*x*z - 2*w*y;
    *(*(pArrMatrix + 0) + 3) =  0;
    *(*(pArrMatrix + 1) + 0) =  2*x*y - 2*w*z;
    *(*(pArrMatrix + 1) + 1) =  1 - 2*x*x - 2*z*z;
    *(*(pArrMatrix + 1) + 2) =  2*y*z + 2*w*x;
    *(*(pArrMatrix + 1) + 3) =  0;
    *(*(pArrMatrix + 2) + 0) =  2*x*z + 2*w*y;
    *(*(pArrMatrix + 2) + 1) =  2*y*z - 2*w*x;
    *(*(pArrMatrix + 2) + 2) =  1 - 2*x*x -2*y*y;
    *(*(pArrMatrix + 2) + 3) =  0;
    *(*(pArrMatrix + 3) + 0) =  0;
    *(*(pArrMatrix + 3) + 1) =  0;
    *(*(pArrMatrix + 3) + 2) =  0;
    *(*(pArrMatrix + 3) + 3) =  1;
}
/**********************************************************************************
 * Function:convert from Quaternion to Homogeneous transformation matrix
 * Parament:
 * In  :(*pArrMatrix)[4] --> store Homogeneous transformation matrix
 * Out :quat --> quaternion
 * Return: None
 * Reference: 《3D数学基础：图形与游戏开发》
 ************************************************************************************/
void RobotModel::transToQuaternion(double (*pArrMatrix)[4], QUATERNIONS &quat)
{
    double fourWSquareMax, fourXSquareMax, fourYSquareMax, fourZSquareMax;
    double fourBiggestSquareMax, biggestVar, multTmp;
    double m11, m12, m13;
    double m21, m22, m23;
    double m31, m32, m33;
    int biggestIndex = 0;
    //Input rotation matrix
    m11 = *(*(pArrMatrix + 0) + 0);
    m12 = *(*(pArrMatrix + 0) + 1);
    m13 = *(*(pArrMatrix + 0) + 2);
    m21 = *(*(pArrMatrix + 1) + 0);
    m22 = *(*(pArrMatrix + 1) + 1);
    m23 = *(*(pArrMatrix + 1) + 2);
    m31 = *(*(pArrMatrix + 2) + 0);
    m32 = *(*(pArrMatrix + 2) + 1);
    m33 = *(*(pArrMatrix + 2) + 2);
    //check the biggest var among w,x,y,z
    fourWSquareMax = m11 + m22 + m33;
    fourXSquareMax = m11 - m22 - m33;
    fourYSquareMax = m22 - m11 - m33;
    fourZSquareMax = m33 - m11 - m22;
    fourBiggestSquareMax = fourWSquareMax;
    if(fourXSquareMax > fourBiggestSquareMax)
    {
        fourBiggestSquareMax = fourXSquareMax;
        biggestIndex = 1;
    }
    if(fourYSquareMax > fourBiggestSquareMax)
    {
        fourBiggestSquareMax = fourYSquareMax;
        biggestIndex = 2;
    }
    if(fourZSquareMax > fourBiggestSquareMax)
    {
        fourBiggestSquareMax = fourZSquareMax;
        biggestIndex = 3;
    }
    biggestVar = sqrt(fourBiggestSquareMax + 1.0L)*0.5L;
    multTmp = 0.25L/biggestVar;
    //calculate the quaternion
    switch(biggestIndex)
    {
        case 0:
            quat.q0 = biggestVar;
            quat.q1 = (m23 - m32)*multTmp;
            quat.q2 = (m31 - m13)*multTmp;
            quat.q3 = (m12 - m21)*multTmp;
            break;
        case 1:
            quat.q0 = biggestVar;
            quat.q1 = (m23 - m32)*multTmp;
            quat.q2 = (m12 + m21)*multTmp;
            quat.q3 = (m31 + m13)*multTmp;
            break;
        case 2:
            quat.q0 = biggestVar;
            quat.q1 = (m31 - m13)*multTmp;
            quat.q2 = (m12 + m21)*multTmp;
            quat.q3 = (m23 + m32)*multTmp;
            break;
        case 3:
            quat.q0 = biggestVar;
            quat.q1 = (m12 - m21)*multTmp;
            quat.q2 = (m31 + m13)*multTmp;
            quat.q3 = (m23 + m32)*multTmp;
            break;
        default:
            break;
    }
}
/***********************************************************************************************
 * Function:    get the posOrientation tcpToFlange by seven position
 * Parament:
 * In  :tcpPos1-7
 * Out :tcpToFlange --> Tcp2Flange(x y z A B C)
 ***********************************************************************************************/
int RobotModel::toolCalibration(POS_ORIENTATION &tcpPos1, POS_ORIENTATION &tcpPos2, POS_ORIENTATION &tcpPos3,
                        POS_ORIENTATION &tcpPos4, POS_ORIENTATION &tcpPos5, POS_ORIENTATION &tcpPos6,
                        POS_ORIENTATION &tcpPos7, int calibrationPointNum, POS_ORIENTATION *tcpToFlange)
{
    if (calibrationPointNum == 7)
    {
        //to store the centre of the sphere
        double ox = 0.0, oy = 0.0, oz = 0.0;
        //vars used in the step1
        Vector4d vecSphereCentre(4),vecTool2Flange(4);
        Matrix4 matPos1;
        Position_MCS_rad tcp1(6);
        Position_MCS_rad tcp5(6);
        Position_MCS_rad tcpFlange(6);
        //vars used in the step2
        Vector4d vecPos5ToBase(4);
        Vector3d vecTemp1(3),vecTemp2(3),vecTemp3(3);
        Matrix4 matPos5, matTool2Base, matTool2Flange;
        //Step1: 求解圆心,然后解出TCP点到法兰末端的位置P(Px Py Pz)，存放在tcpToFlange指针中
        Sphere(tcpPos1.x, tcpPos1.y, tcpPos1.z, tcpPos2.x, tcpPos2.y, tcpPos2.z, tcpPos3.x,tcpPos3.y,
                tcpPos3.z, tcpPos4.x, tcpPos4.y, tcpPos4.z, &ox, &oy, &oz);
        vecSphereCentre(0) = ox;
        vecSphereCentre(1) = oy;
        vecSphereCentre(2) = oz;
        vecSphereCentre(3) = 1.0;
        //eulerZYXToTrans(tcpPos1, arrPos1);
        tcp1(0) = tcpPos1.x;
        tcp1(1) = tcpPos1.y;
        tcp1(2) = tcpPos1.z;
        tcp1(3) = deg2rad(tcpPos1.A);
        tcp1(4) = deg2rad(tcpPos1.B);
        tcp1(5) = deg2rad(tcpPos1.C);
        matPos1 = rpy2tr(tcp1);
        vecTool2Flange = matPos1.inverse()*vecSphereCentre;     //P(Px Py Pz) in the tool coordinate system
        //Step2: 再求相对于法兰盘末端的旋转方向
        //1.求TCP点在基坐标系下的姿态（向量）
        vecTemp1(0) = tcpPos6.x - tcpPos5.x;
        vecTemp1(1) = tcpPos6.y - tcpPos5.y;
        vecTemp1(2) = tcpPos6.z - tcpPos5.z;
        vecTemp2(0) = tcpPos7.x - tcpPos5.x;
        vecTemp2(1) = tcpPos7.y - tcpPos5.y;
        vecTemp2(2) = tcpPos7.z - tcpPos5.z;
        vecTemp3 = vecTemp1.cross(vecTemp2);    //vector3=vector1叉乘vector2
        vecTemp2 = vecTemp3.cross(vecTemp1);    //vector2=vector3叉乘vector1
        //2.姿态（向量）单位化得到N、O、A矢量，即旋转矩阵R
        vecTemp1.normalize();
        vecTemp2.normalize();
        vecTemp3.normalize();
        //3.在Pos5处创建TCP点在基坐标系下的齐次变换矩阵bTt
        tcp5(0) = tcpPos5.x;
        tcp5(1) = tcpPos5.y;
        tcp5(2) = tcpPos5.z;
        tcp5(3) = deg2rad(tcpPos5.A);
        tcp5(4) = deg2rad(tcpPos5.B);
        tcp5(5) = deg2rad(tcpPos5.C);
        matPos5 = rpy2tr(tcp5);
        vecPos5ToBase = matPos5*vecTool2Flange; //position5 P(Px Py Pz)in the base coordinate system
        matTool2Base(0,0) = vecTemp1(0);
        matTool2Base(1,0) = vecTemp1(1);
        matTool2Base(2,0) = vecTemp1(2);
        matTool2Base(3,0) = 0.0;
        matTool2Base(0,1) = vecTemp2(0);
        matTool2Base(1,1) = vecTemp2(1);
        matTool2Base(2,1) = vecTemp2(2);
        matTool2Base(3,1) = 0.0;
        matTool2Base(0,2) = vecTemp3(0);
        matTool2Base(1,2) = vecTemp3(1);
        matTool2Base(2,2) = vecTemp3(2);
        matTool2Base(3,2) = 0.0;
        matTool2Base(0,3) = vecPos5ToBase(0);
        matTool2Base(1,3) = vecPos5ToBase(1);
        matTool2Base(2,3) = vecPos5ToBase(2);
        matTool2Base(3,3) = 1.0;
        matTool2Flange = matPos5.inverse()*matTool2Base;
        tcpFlange = tr2MCS(matTool2Flange);
        for (AXIS_REFS_INDEX ai=0; ai<6; ++ai)
        {
            if (fabs(tcpFlange[ai]) < 10e-4)
            {
                tcpFlange[ai] = 0;
            }
        }
        tcpToFlange->x = tcpFlange(0);
        tcpToFlange->y = tcpFlange(1);
        tcpToFlange->z = tcpFlange(2);
        tcpToFlange->A = tcpFlange(3);
        tcpToFlange->B = tcpFlange(4);
        tcpToFlange->C = tcpFlange(5);
    }
    else if (calibrationPointNum == 6)
    {
        //1.求工具手的xy
        dvector dThetaTool(3);
        Matrix4 transMatrix1;
        Matrix4 transMatrix2;
        Position pos1(6),pos2(6);

        pos1(0) = tcpPos1.x;
        pos1(1) = tcpPos1.y;
        pos1(2) = tcpPos1.z;
        pos1(3) = deg2rad(tcpPos1.A);
        pos1(4) = deg2rad(tcpPos1.B);
        pos1(5) = deg2rad(tcpPos1.C);
        transMatrix1 = rpy2tr(pos1);

        pos2(0) = tcpPos2.x;
        pos2(1) = tcpPos2.y;
        pos2(2) = tcpPos2.z;
        pos2(3) = deg2rad(tcpPos2.A);
        pos2(4) = deg2rad(tcpPos2.B);
        pos2(5) = deg2rad(tcpPos2.C);
        transMatrix2 = rpy2tr(pos2);


        double a,b,c,d,e,f;
        a = transMatrix1(0,0)-transMatrix2(0,0);
        b = transMatrix1(0,1)-transMatrix2(0,1);
        c = transMatrix1(1,0)-transMatrix2(1,0);
        d = transMatrix1(1,1)-transMatrix2(1,1);
        e = transMatrix2(0,3)-transMatrix1(0,3);
        f = transMatrix2(1,3)-transMatrix1(1,3);
        dThetaTool[0] = (e*d-b*f)/(a*d-b*c);
        dThetaTool[1] = (e*c-a*f)/(b*c-a*d);
        dThetaTool[2] = 0;

        //2.求工具手的z
        double zTool[3];
        pos2(0) = tcpPos3.x;
        pos2(1) = tcpPos3.y;
        pos2(2) = tcpPos3.z;
        pos2(3) = deg2rad(tcpPos3.A);
        pos2(4) = deg2rad(tcpPos3.B);
        pos2(5) = deg2rad(tcpPos3.C);
        transMatrix2 = rpy2tr(pos2);

        for(int i = 0; i < 3; i++)
        {
            a = transMatrix1(0,0)-transMatrix2(0,0);
            b = transMatrix1(0,1)-transMatrix2(0,1);
            c = transMatrix1(0,2)-transMatrix2(0,2);
            d = transMatrix2(0,3)-transMatrix1(0,3);
            zTool[i] = (d - dThetaTool[0] * a - dThetaTool[1] * b) / c;
        }

        dThetaTool[2] = zTool[0];

        Vector3d vecTemp1(3),vecTemp2(3),vecTemp3(3);
        vecTemp1(0) = tcpPos5.x - tcpPos4.x;
        vecTemp1(1) = tcpPos5.y - tcpPos4.y;
        vecTemp1(2) = tcpPos5.z - tcpPos4.z;
        vecTemp2(0) = tcpPos6.x - tcpPos4.x;
        vecTemp2(1) = tcpPos6.y - tcpPos4.y;
        vecTemp2(2) = tcpPos6.z - tcpPos4.z;
        vecTemp3 = vecTemp1.cross(vecTemp2);    //vector3=vector1叉乘vector2
        vecTemp2 = vecTemp3.cross(vecTemp1);    //vector2=vector3叉乘vector1
        //2.姿态（向量）单位化得到N、O、A矢量，即旋转矩阵R
        vecTemp1.normalize();
        vecTemp2.normalize();
        vecTemp3.normalize();
        //3.在Pos5处创建TCP点在基坐标系下的齐次变换矩阵bTt
        RPY rpyBase,tcpFlange;
        Matrix3 matBase,matTool2Base,matTool2Flange;
        rpyBase(0) = deg2rad(tcpPos4.A);
        rpyBase(1) = deg2rad(tcpPos4.B);
        rpyBase(2) = deg2rad(tcpPos4.C);
        matBase = rpy2r(rpyBase);

        matTool2Base(0,0) = vecTemp1(0);
        matTool2Base(1,0) = vecTemp1(1);
        matTool2Base(2,0) = vecTemp1(2);

        matTool2Base(0,1) = vecTemp2(0);
        matTool2Base(1,1) = vecTemp2(1);
        matTool2Base(2,1) = vecTemp2(2);

        matTool2Base(0,2) = vecTemp3(0);
        matTool2Base(1,2) = vecTemp3(1);
        matTool2Base(2,2) = vecTemp3(2);

        matTool2Flange = matBase.inverse()*matTool2Base;
        tcpFlange = tr2rpy(matTool2Flange);

        tcpToFlange->x = dThetaTool[0];
        tcpToFlange->y = dThetaTool[1];
        tcpToFlange->z = dThetaTool[2];
        tcpToFlange->A = tcpFlange(0);
        tcpToFlange->B = tcpFlange(1);
        tcpToFlange->C = tcpFlange(2);
    }
    else
    {
        return ERROR;
    }
    if (IS_VALID_DOUBLE(tcpToFlange->x) && IS_VALID_DOUBLE(tcpToFlange->y) && IS_VALID_DOUBLE(tcpToFlange->z)
        && IS_VALID_DOUBLE(tcpToFlange->A) && IS_VALID_DOUBLE(tcpToFlange->B) && IS_VALID_DOUBLE(tcpToFlange->C))
        return 0;
    else
        return ERROR;
}
/*******************************************************************************
 *Function: get the centre of the sphere  (OK)
 *******************************************************************************/
int RobotModel::Sphere(double x1, double y1, double z1, double x2, double y2, double z2,
        double x3, double y3, double z3, double x4, double y4, double z4,
        double *px, double *py, double *pz)
{
    double a11, a12, a13, a21, a22, a23, a31, a32, a33, b1, b2, b3, d, d1, d2,
            d3;
    a11 = 2 * (x2 - x1);
    a12 = 2 * (y2 - y1);
    a13 = 2 * (z2 - z1);
    a21 = 2 * (x3 - x2);
    a22 = 2 * (y3 - y2);
    a23 = 2 * (z3 - z2);
    a31 = 2 * (x4 - x3);
    a32 = 2 * (y4 - y3);
    a33 = 2 * (z4 - z3);
    b1 = x2 * x2 - x1 * x1 + y2 * y2 - y1 * y1 + z2 * z2 - z1 * z1;
    b2 = x3 * x3 - x2 * x2 + y3 * y3 - y2 * y2 + z3 * z3 - z2 * z2;
    b3 = x4 * x4 - x3 * x3 + y4 * y4 - y3 * y3 + z4 * z4 - z3 * z3;
    d = a11 * a22 * a33 + a12 * a23 * a31 + a13 * a21 * a32 - a11 * a23 * a32
            - a12 * a21 * a33 - a13 * a22 * a31;
    d1 = b1 * a22 * a33 + a12 * a23 * b3 + a13 * b2 * a32 - b1 * a23 * a32
            - a12 * b2 * a33 - a13 * a22 * b3;
    d2 = a11 * b2 * a33 + b1 * a23 * a31 + a13 * a21 * b3 - a11 * a23 * b3
            - b1 * a21 * a33 - a13 * b2 * a31;
    d3 = a11 * a22 * b3 + a12 * b2 * a31 + b1 * a21 * a32 - a11 * b2 * a32
            - a12 * a21 * b3 - b1 * a22 * a31;
    (*px) = d1 / d;
    (*py) = d2 / d;
    (*pz) = d3 / d;
    return 0;
}
//int Sphere(double x1,double y1,double z1,double x2,double y2,double z2,
//          double x3,double y3,double z3,double x4,double y4,double z4,
//          double *px,double *py,double *pz)
//{
//  (*px)= 0.5*(((-z4+z3)*y2+(y4-y3)* z2+y3*z4-y4*z3)*x1*x1+
//      ((-z4+z3)*y2+(y4-y3)*z2+y3*z4-y4*z3)*y1*y1+
//      ((-z3+z4)*x2*x2+(-z3+z4)*y2*y2+(-z3+z4)*z2*z2+
//      (x3*x3-z4*z4+z3*z3-y4*y4-x4*x4+y3*y3)*z2-x3*x3*z4-y3*y3*z4-z3*z3*z4+
//      (y4*y4+x4*x4+z4*z4)*z3)*y1+((-z4+z3)*y2+(y4-y3)*z2+y3*z4-y4*z3)*z1*z1+
//      ((y3-y4)*x2*x2+(y3-y4)*y2*y2+(x4*x4+y4*y4-x3*x3+z4*z4-z3*z3-y3*y3)*y2+(y3-y4)*z2*z2+
//      x3*x3*y4+y4*y3*y3+(-x4*x4-z4*z4-y4*y4)*y3+y4*z3*z3)*z1+(y4*z3-y3*z4)*x2*x2+
//      (y4*z3-y3*z4)*y2*y2+(x3*x3*z4+y3*y3*z4+z3*z3*z4+(-x4*x4-z4*z4-y4*y4)*z3)*y2+
//      (y4*z3-y3*z4)*z2*z2+(-x3*x3*y4-y4*y3*y3+(y4*y4+x4*x4+z4*z4)*y3-y4*z3*z3)*z2)
//      /(((-z4+z3)*y2+(y4-y3)*z2+y3*z4-y4*z3)*x1+((-z3+z4)*x2+(x3-x4)*z2+z3*x4-x3*z4)*y1+
//      ((y3-y4)*x2+(x4-x3)*y2-x4*y3+x3*y4)*z1+(y4*z3-y3*z4)*x2+(x3*z4-z3*x4)*y2+(x4*y3-x3*y4)*z2);
//
//  (*py)= -0.5*(((-z4+z3)*x2+(x4-x3)*z2+x3*z4-z3*x4)*x1*x1+((-z3+z4)*x2*x2+(-z3+z4)*y2*y2+
//      (-z3+z4)*z2*z2+(x3*x3-z4*z4+z3*z3-y4*y4-x4*x4+y3*y3)*z2-x3*x3*z4-y3*y3*z4-z3*z3*z4+
//      (y4*y4+x4*x4+z4*z4)*z3)*x1+((-z4+z3)*x2+(x4-x3)*z2+x3*z4-z3*x4)*y1*y1+
//      ((-z4+z3)*x2+(x4-x3)*z2+x3*z4-z3*x4)*z1*z1+((x3-x4)*x2*x2+(x4*x4+y4*y4-x3*x3+z4*z4
//      -z3*z3-y3*y3)*x2+(x3-x4)*y2*y2+(x3-x4)*z2*z2+x4*x3*x3+(-x4*x4-z4*z4-y4*y4)*x3+y3*y3*x4+
//      x4*z3*z3)*z1+(z3*x4-x3*z4)*x2*x2+(x3*x3*z4+y3*y3*z4+z3*z3*z4+(-x4*x4-z4*z4-y4*y4)*z3)*x2
//      +(z3*x4-x3*z4)*y2*y2+(z3*x4-x3*z4)*z2*z2+(-x4*x3*x3+(y4*y4+x4*x4+z4*z4)*x3-x4*z3*z3-
//      y3*y3*x4)*z2)/
//      (((-z4+z3)*y2+(y4-y3)*z2+y3*z4-y4*z3)*x1+((-z3+z4)*x2+(x3-x4)*z2+z3*x4-x3*z4)*y1+
//      ((y3-y4)*x2+(x4-x3)*y2-x4*y3+x3*y4)*z1+(y4*z3-y3*z4)*x2+(x3*z4-z3*x4)*y2+(x4*y3-x3*y4)*z2);
//
//  (*pz) = 0.5*(((y3-y4)*x2+(x4-x3)*y2-x4*y3+x3*y4)*x1*x1+((y4-y3)*x2*x2+(y4-y3)*y2*y2+
//      (x3*x3-z4*z4+z3*z3-y4*y4-x4*x4+y3*y3)*y2+(y4-y3)*z2*z2-x3*x3*y4-y4*y3*y3+
//      (y4*y4+x4*x4+z4*z4)*y3-y4*z3*z3)*x1+((y3-y4)*x2+(x4-x3)*y2-x4*y3+x3*y4)*y1*y1+
//      ((x3-x4)*x2*x2+(x4*x4+y4*y4-x3*x3+z4*z4-z3*z3-y3*y3)*x2+
//      (x3-x4)*y2*y2+(x3-x4)*z2*z2+x4*x3*x3+(-x4*x4-z4*z4-y4*y4)*x3+y3*y3*x4+x4*z3*z3)*y1+
//      ((y3-y4)*x2+(x4-x3)*y2-x4*y3+x3*y4)*z1*z1+(x4*y3-x3*y4)*x2*x2+
//      (x3*x3*y4+y4*y3*y3+(-x4*x4-z4*z4-y4*y4)*y3+y4*z3*z3)*x2+
//      (x4*y3-x3*y4)*y2*y2+(-x4*x3*x3+(y4*y4+x4*x4+z4*z4)*x3-x4*z3*z3-y3*y3*x4)*y2+
//      (x4*y3-x3*y4)*z2*z2)/
//      (((-z4+z3)*y2+(y4-y3)*z2+y3*z4-y4*z3)*x1+((-z3+z4)*x2+(x3-x4)*z2+z3*x4-x3*z4)*y1+
//      ((y3-y4)*x2+(x4-x3)*y2-x4*y3+x3*y4)*z1+(y4*z3-y3*z4)*x2+(x3*z4-z3*x4)*y2+
//      (x4*y3-x3*y4)*z2);
//      //r = sqrt((x1-ox)*(x1-ox)+(y1-oy)*(y1-oy)+(z1-oz)*(z1-oz));
//      //double temp= sqrt((x2-ox)*(x2-ox)+(y2-oy)*(y2-oy)+(z2-oz)*(z2-oz));
//      //if( (fabs(r-temp)>5))
//      //{
//      //  AfxMessageBox("计算结果有误！请给出空间非同一平面内的四个点！");
//      //  return FALSE;
//      //}
//  return 0;
//}
/**************************************************************************************
 * Function:get the user coordinate system defined in the base coordinate system
 * Parament:
 * Input: posTool --> posTool2Flange
 * posORG  --> TCP点运动到用户坐标系原点时，此时法兰处的位姿
 * posOx   --> TCP点运动到用户坐标系X轴上一点是，此时法兰处的位姿
 * posOy   --> TCP点运动到用户坐标系Y轴上一点是，此时法兰处的位姿
 * Output: posUsr  --> result(指针形式)
 ***************************************************************************************/
void RobotModel::usrCalibration(POS_ORIENTATION &posTool, POS_ORIENTATION &posORG, POS_ORIENTATION &posOx, POS_ORIENTATION &posOy, POS_ORIENTATION *posUsr)
{
    using namespace Eigen;

    int i, j;
    //vars used by the step 1
    double arrORG[4][4], arrTool[4][4];
    Matrix4 matORG, matTool, matTemp1;
    //vars used by the step2
    Vector3d vecTemp1(3), vecTemp2(3), vecTemp3(3);
    double arrOx[4][4], arrOy[4][4], arrUsr[4][4];
    Matrix4 matOx, matOy, matTemp2, matTemp3;
    //step1: 确定用户坐标系的原点位置（相对于基坐标系）
    eulerZYXToTrans(posORG, arrORG);
    eulerZYXToTrans(posTool, arrTool);
    for(i = 0; i <= 3; i++)
        for(j = 0; j <= 3; j++)
        {
            matORG(i,j) = *(*(arrORG + i) + j);
            matTool(i,j) = *(*(arrTool + i) + j);
        }
    matTemp1 = matORG*matTool;  //用户坐标系示教原点(结果在基坐标系下)
    //step2: 确定用户坐标系的姿态（相对于基坐标系）
    //1.计算ORG、Ox、Oy在基坐标系下的位置值
    eulerZYXToTrans(posOx, arrOx);
    eulerZYXToTrans(posOy, arrOy);
    for(i = 0; i <= 3; i++)
        for(j = 0; j <= 3; j++)
        {
            matOx(i,j) = *(*(arrOx + i) + j);
            matOy(i,j) = *(*(arrOy + i) + j);
        }
    matTemp2 = matOx*matTool;   //用户坐标系示教x轴
    matTemp3 = matOy*matTool;   //用户坐标系示教y轴
    //2.计算姿态
    //axis X = Ox - ORG
    vecTemp1(0) = matTemp2(0,3) - matTemp1(0,3);    //x
    vecTemp1(1) = matTemp2(1,3) - matTemp1(1,3);    //y
    vecTemp1(2) = matTemp2(2,3) - matTemp1(2,3);    //z
    //axis Y = Oy - ORG
    vecTemp2(0) = matTemp3(0,3) - matTemp1(0,3);
    vecTemp2(1) = matTemp3(1,3) - matTemp1(1,3);
    vecTemp2(2) = matTemp3(2,3) - matTemp1(2,3);
    vecTemp3 = vecTemp1.cross(vecTemp2);
    //3.normalize
    vecTemp1.normalize();
    vecTemp2.normalize();
    vecTemp3.normalize();
    //4.构造齐次变换矩阵
    arrUsr[0][0] = vecTemp1(0);
    arrUsr[1][0] = vecTemp1(1);
    arrUsr[2][0] = vecTemp1(2);
    arrUsr[3][0] = 0.0;
    arrUsr[0][1] = vecTemp2(0);
    arrUsr[1][1] = vecTemp2(1);
    arrUsr[2][1] = vecTemp2(2);
    arrUsr[3][1] = 0.0;
    arrUsr[0][2] = vecTemp3(0);
    arrUsr[1][2] = vecTemp3(1);
    arrUsr[2][2] = vecTemp3(2);
    arrUsr[3][2] = 0.0;
    arrUsr[0][3] = matTemp1(0,3);
    arrUsr[1][3] = matTemp1(1,3);
    arrUsr[2][3] = matTemp1(2,3);
    arrUsr[3][3] = 1.0;
    transToEulerZYXv2(arrUsr, posUsr);
}

int RobotModel::syncPositionerToBaseCalibration(XYZ &ucsPos1,XYZ &ucsPos2,XYZ &ucsPos3, dvector6 &ucsChange)
{
    using namespace Eigen;

    Vector3 xAxisVector, yAxisVector,zAxisVector, origin;

    origin = ucsPos1;
    xAxisVector = ucsPos2 - ucsPos1;
    yAxisVector = ucsPos3 - ucsPos1;
    zAxisVector = xAxisVector.cross(yAxisVector);
    yAxisVector = zAxisVector.cross(xAxisVector);

    xAxisVector.normalize();
    yAxisVector.normalize();
    zAxisVector.normalize();

    Matrix3 rotationMatrix,basicMatrix,ucsMatrix;
    Matrix4 changeMatrix;

    basicMatrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    ucsMatrix << xAxisVector, yAxisVector, zAxisVector;
    rotationMatrix = ucsMatrix * basicMatrix.reverse();
    changeMatrix << rotationMatrix, origin, dmatrix::Zero(1, 3), 1;
    ucsChange = tr2MCS(changeMatrix);
    for (AXIS_REFS_INDEX ai=0; ai<6; ++ai)
    {
        if (fabs(ucsChange[ai]) < 10e-4)
        {
            ucsChange[ai] = 0;
        }
    }
    return 0;
}
/*****************************************************************************/
/**
 * @brief 相贯线标定
 * @param[in] p1~p6,为标定时记录的6个tcp点位置坐标值
 * @param[out]
 * @return 若成功，返回 RBTINTERF_NO_ERROR
 * @note 方法用来测定待焊接工件几何尺寸（R,r,e,alpha）和工件坐标系在机器人基坐标系的变换矩阵T_b_wp
 * @todo 角速度限制
 */
/*****************************************************************************/
int RobotModel::cylinderIntersectCalibration(const Vector3 &p1, const Vector3 &p2, const Vector3 &p3,
                                 const Vector3 &p4, const Vector3 &p5, const Vector3 &p6,
                                 Position_MCS_rad &Result, CylinderIntersectLine_Para &cylindIntersecParam)
{
    XYZ pO1, pO2, pO3, pO4;//圆心
    Vector3 lineUp, lineDown, r1, r2, perpendicularLine;
    Vector3 xAxisVector, yAxisVector,zAxisVector, zAxisBase;
    Matrix3 rotationMatrix;
    Vector3 Zb,Yb;
    Zb << 0, 0, 1;
    Yb << 0, 1, 0;
    //判断共点、共线
    Vector3 v1 = p1 - p2;
    Vector3 v2 = p1 - p3;
    if (v1.norm() < 0.1 || v2.norm() < 0.1) //值很小
        return -1;
    double agl = acos(v1.dot(v2) / v1.norm() / v2.norm());
    if (fabs(agl) < 0.1 * M_PI / 180) //小于0.1度
        return -1;
    //给出空间三点求圆心
    pO1 = calculateCircleCenter(p1, p2, p3);

    CylinderIntersectLine_Para cylinderIntersectLine; //相贯线几何信息
    Matrix4 T_b_wp; //相贯线工件坐标系
    cylinderIntersectLine.r = (p1-pO1).norm();//上圆柱半径
    lineUp = v1.cross(v2);
    if (lineUp.dot(Zb) < 0)
    {
        lineUp = -lineUp;
    }
    //判断共点、共线
    Vector3 v3 = p4 - p5;
    Vector3 v4 = p4 - p6;
    if (v3.norm() < 0.1 || v4.norm() < 0.1) //值很小
        return -1;
    agl = acos(v3.dot(v4) / v3.norm() / v4.norm());
    if (fabs(agl) < 0.1 * M_PI / 180) //小于0.1度
        return -1;

    //给出空间三点求圆心
    pO2 = calculateCircleCenter(p4, p5, p6);
    cylinderIntersectLine.R = (p4-pO2).norm();//下圆柱半径
    lineDown =  v3.cross(v4);
    if (lineDown.dot(Yb) < 0)
    {
        lineDown = -lineDown;
    }
    intersectByTwoLine(lineUp, pO1, lineDown, pO2, r1, r2, cylinderIntersectLine.e, cylinderIntersectLine.alpha);
    perpendicularLine = lineUp.cross(lineDown); //求解两条直线的公垂线的向量，为上圆柱坐标系的Y轴

    zAxisBase << 0, 0, 1;
    if (lineUp.dot(zAxisBase) > 0)
    {
        zAxisVector = lineUp;
    }
    else
    {
        zAxisVector = -lineUp;
    }

    yAxisVector = perpendicularLine;
    xAxisVector = yAxisVector.cross(zAxisVector);
    xAxisVector.normalize();
    yAxisVector.normalize();
    zAxisVector.normalize();
    rotationMatrix << xAxisVector, yAxisVector, zAxisVector;
    T_b_wp << rotationMatrix, r1, dmatrix::Zero(1, 3), 1; //工件坐标系在机器人基坐标系的变换矩阵
    Result = tr2MCS(T_b_wp);
    cylindIntersecParam.R = cylinderIntersectLine.R;
    cylindIntersecParam.r = cylinderIntersectLine.r;
    cylindIntersecParam.e = cylinderIntersectLine.e;
    cylindIntersecParam.alpha = cylinderIntersectLine.alpha;

    return 0;
}

/**
 * @brief  将rpy转换为4*4矩阵中的姿态的3*3矩阵
 * @param[in] rpy TCP的姿态
 * @return 4*4矩阵中的姿态的3*3矩阵
*/
Matrix3 RobotModel::rpy2r(RPY rpy) const
{
    if (eulerAngle == "xyz" || eulerAngle == "XYZ")
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitZ()).toRotationMatrix();
    }
    else if (eulerAngle == "xzy" || eulerAngle == "XZY")
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitY()).toRotationMatrix();
    }
    else if (eulerAngle == "yxz" || eulerAngle == "YXZ")
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitZ()).toRotationMatrix();
    }
    else if (eulerAngle == "yzx" || eulerAngle == "YZX")
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitX()).toRotationMatrix();
    }
    else if (eulerAngle == "zxy" || eulerAngle == "ZXY")
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitY()).toRotationMatrix();
    }
    else if (eulerAngle == "zyx" || eulerAngle == "ZYX")
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitX()).toRotationMatrix();
    }
    else if (eulerAngle == "xyx" || eulerAngle == "XYX")
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitX()).toRotationMatrix();
    }
    else if (eulerAngle == "xzx" || eulerAngle == "XZX")
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitX()).toRotationMatrix();
    }
    else if (eulerAngle == "yxy" || eulerAngle == "YXY")
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitY()).toRotationMatrix();
    }
    else if (eulerAngle == "yzy" || eulerAngle == "YZY")
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitY()).toRotationMatrix();
    }
    else if (eulerAngle == "zxz" || eulerAngle == "ZXZ")
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitZ()).toRotationMatrix();
    }
    else if (eulerAngle == "zyz" || eulerAngle == "ZYZ")
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitZ()).toRotationMatrix();
    }
    else
    {
        return Eigen::AngleAxisd(rpy(0), Vector3::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3::UnitZ()).toRotationMatrix();
    }
}

/**
 * @brief  将TCP的位姿转换为4*4位姿矩阵
 * @param[in]  posMCS TCP的位姿
 * @return tr 4*4的位姿矩阵
*/
Matrix4 RobotModel::rpy2tr(Position_MCS_rad posMCS) const
{
    Matrix4 tr;
    tr << rpy2r(posMCS.segment(3,3)), posMCS.head(3), RowVector3::Zero(1, 3), 1;
    return tr;
}

/**
 * @brief  将矩阵转化为rpy
 * @param[in] m 位姿矩阵
 * @return rpy
*/
RPY RobotModel::tr2rpy(Matrix4 m) const
{
    RPY rpy;
    if (eulerAngle == "xyz" || eulerAngle == "XYZ")
    {
		if (fabs(m(1, 2)) < 10e-13 && fabs(m(2, 2)) < 10e-13)
		{
			if (m(0, 2) > 10e-13)
			{
				rpy(0) = atan2(m(2, 1), m(1, 1));
			}
			else
			{
				rpy(0) = -atan2(m(1, 0), m(2, 0));
			}
			rpy(2) = 0;
		}
		else
		{
			rpy(0) = atan2(-m(1, 2), m(2, 2));
			rpy(2) = atan2(-m(0, 1), m(0, 0));
		}

        double sr = sin(rpy(0, 0));
        double cr = cos(rpy(0, 0));
        rpy(1) = atan2(m(0, 2), cr*m(2, 2)-sr*m(1, 2));
    }
    else if (eulerAngle == "xzy" || eulerAngle == "XZY")
    {
        rpy(0) = atan2(m(2, 1), m(1, 1));
        rpy(2) = atan2(m(0, 2), m(0, 0));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(-m(0, 1), cr*m(0, 0)+sr*m(0, 2));
    }
    else if (eulerAngle == "yxz" || eulerAngle == "YXZ")
    {
        rpy(0) = atan2(m(0, 2), m(2, 2));
        rpy(2) = atan2(m(1, 0), m(1, 1));
        double sr = sin(rpy(0, 0));
        double cr = cos(rpy(0, 0));
        rpy(1) = atan2(-m(1, 2), cr*m(2, 2)+sr*m(0, 2));
    }
    else if (eulerAngle == "yzx" || eulerAngle == "YZX")
    {
        rpy(0) = atan2(-m(2, 0), m(0, 0));
        rpy(2) = atan2(-m(1, 2), m(1, 1));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(m(1, 0), cr*m(1, 1)-sr*m(1, 2));
    }
    else if (eulerAngle == "zxy" || eulerAngle == "ZXY")
    {
        rpy(0) = atan2(-m(0, 1), m(1, 1));
        rpy(2) = atan2(-m(2, 0), m(2, 2));
        double sr = sin(rpy(0, 0));
        double cr = cos(rpy(0, 0));
        rpy(1) = atan2(m(2, 1), cr*m(1, 1)-sr*m(0, 1));
    }
    else if (eulerAngle == "zyx" || eulerAngle == "ZYX")
    {
        rpy(0) = atan2(m(1, 0), m(0, 0));
        rpy(2) = atan2(m(2, 1), m(2, 2));
        double sr = sin(rpy(0, 0));
        double cr = cos(rpy(0, 0));
        rpy(1) = atan2(-m(2, 0), cr*m(0, 0)+sr*m(1, 0));
    }
    else if (eulerAngle == "xyx" || eulerAngle == "XYX")
    {
        rpy(0) = atan2(m(1, 0), -m(2, 0));
        rpy(2) = atan2(m(0, 1), m(0, 2));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(cr*m(0, 2)+sr*m(0, 1), m(0, 0));
    }
    else if (eulerAngle == "xzx" || eulerAngle == "XZX")
    {
        rpy(0) = atan2(m(2, 0), m(1, 0));
        rpy(2) = atan2(m(0, 2), -m(0, 1));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(sr*m(0, 2)-cr*m(0, 1), m(0, 0));
    }
    else if (eulerAngle == "yxy" || eulerAngle == "YXY")
    {
        rpy(0) = atan2(m(0, 1), m(2, 1));
        rpy(2) = atan2(m(1, 0), -m(1, 2));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(sr*m(1, 0)-cr*m(1, 2), m(1, 1));
    }
    else if (eulerAngle == "yzy" || eulerAngle == "YZY")
    {
        rpy(0) = atan2(m(2, 1), -m(0, 1));
        rpy(2) = atan2(m(1, 2), m(1, 0));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(sr*m(1, 2)+cr*m(1, 0), m(1, 1));
    }
    else if (eulerAngle == "zxz" || eulerAngle == "ZXZ")
    {
        rpy(0) = atan2(m(0, 2), -m(1, 2));
        rpy(2) = atan2(m(2, 0), m(2, 1));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(sr*m(2, 0)+cr*m(2, 1), m(2, 2));
    }
    else if (eulerAngle == "zyz" || eulerAngle == "ZYZ")
    {
        rpy(0) = atan2(m(1, 2), m(0, 2));
        rpy(2) = atan2(m(2, 1), -m(2, 0));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(sr*m(2, 1)-cr*m(2, 0), m(2, 2));
    }
    else
    {
        rpy(0) = atan2(-m(1, 2), m(2, 2));
        double sr = sin(rpy(0, 0));
        double cr = cos(rpy(0, 0));
        rpy(1) = atan2(m(0, 2), cr*m(2, 2)-sr*m(1, 2));
        rpy(2) = atan2(-m(0, 1), m(0, 0));
    }
    return rpy;
}

RPY RobotModel::tr2rpy(Matrix3 m) const
{
    RPY rpy;
    if (eulerAngle == "xyz" || eulerAngle == "XYZ")
    {
		if (fabs(m(1, 2)) < 10e-13 && fabs(m(2, 2)) < 10e-13)
		{
			if (m(0, 2) > 10e-13)
			{
				rpy(0) = atan2(m(2, 1), m(1, 1));
			}
			else
			{
				rpy(0) = -atan2(m(1, 0), m(2, 0));
			}
			rpy(2) = 0;
		}
		else
		{
			rpy(0) = atan2(-m(1, 2), m(2, 2));
			rpy(2) = atan2(-m(0, 1), m(0, 0));
		}
        double sr = sin(rpy(0, 0));
        double cr = cos(rpy(0, 0));
        rpy(1) = atan2(m(0, 2), cr*m(2, 2)-sr*m(1, 2));
    }
    else if (eulerAngle == "xzy" || eulerAngle == "XZY")
    {
        rpy(0) = atan2(m(2, 1), m(1, 1));
        rpy(2) = atan2(m(0, 2), m(0, 0));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(-m(0, 1), cr*m(0, 0)+sr*m(0, 2));
    }
    else if (eulerAngle == "yxz" || eulerAngle == "YXZ")
    {
        rpy(0) = atan2(m(0, 2), m(2, 2));
        rpy(2) = atan2(m(1, 0), m(1, 1));
        double sr = sin(rpy(0, 0));
        double cr = cos(rpy(0, 0));
        rpy(1) = atan2(-m(1, 2), cr*m(2, 2)+sr*m(0, 2));
    }
    else if (eulerAngle == "yzx" || eulerAngle == "YZX")
    {
        rpy(0) = atan2(-m(2, 0), m(0, 0));
        rpy(2) = atan2(-m(1, 2), m(1, 1));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(m(1, 0), cr*m(1, 1)-sr*m(1, 2));
    }
    else if (eulerAngle == "zxy" || eulerAngle == "ZXY")
    {
        rpy(0) = atan2(-m(0, 1), m(1, 1));
        rpy(2) = atan2(-m(2, 0), m(2, 2));
        double sr = sin(rpy(0, 0));
        double cr = cos(rpy(0, 0));
        rpy(1) = atan2(m(2, 1), cr*m(1, 1)-sr*m(0, 1));
    }
    else if (eulerAngle == "zyx" || eulerAngle == "ZYX")
    {
        rpy(0) = atan2(m(1, 0), m(0, 0));
        rpy(2) = atan2(m(2, 1), m(2, 2));
        double sr = sin(rpy(0, 0));
        double cr = cos(rpy(0, 0));
        rpy(1) = atan2(-m(2, 0), cr*m(0, 0)+sr*m(1, 0));
    }
    else if (eulerAngle == "xyx" || eulerAngle == "XYX")
    {
        rpy(0) = atan2(m(1, 0), -m(2, 0));
        rpy(2) = atan2(m(0, 1), m(0, 2));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(cr*m(0, 2)+sr*m(0, 1), m(0, 0));
    }
    else if (eulerAngle == "xzx" || eulerAngle == "XZX")
    {
        rpy(0) = atan2(m(2, 0), m(1, 0));
        rpy(2) = atan2(m(0, 2), -m(0, 1));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(sr*m(0, 2)-cr*m(0, 1), m(0, 0));
    }
    else if (eulerAngle == "yxy" || eulerAngle == "YXY")
    {
        rpy(0) = atan2(m(0, 1), m(2, 1));
        rpy(2) = atan2(m(1, 0), -m(1, 2));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(sr*m(1, 0)-cr*m(1, 2), m(1, 1));
    }
    else if (eulerAngle == "yzy" || eulerAngle == "YZY")
    {
        rpy(0) = atan2(m(2, 1), -m(0, 1));
        rpy(2) = atan2(m(1, 2), m(1, 0));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(sr*m(1, 2)+cr*m(1, 0), m(1, 1));
    }
    else if (eulerAngle == "zxz" || eulerAngle == "ZXZ")
    {
        rpy(0) = atan2(m(0, 2), -m(1, 2));
        rpy(2) = atan2(m(2, 0), m(2, 1));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(sr*m(2, 0)+cr*m(2, 1), m(2, 2));
    }
    else if (eulerAngle == "zyz" || eulerAngle == "ZYZ")
    {
        rpy(0) = atan2(m(1, 2), m(0, 2));
        rpy(2) = atan2(m(2, 1), -m(2, 0));
        double sr = sin(rpy(2, 0));
        double cr = cos(rpy(2, 0));
        rpy(1) = atan2(sr*m(2, 1)-cr*m(2, 0), m(2, 2));
    }
    else
    {
        rpy(0) = atan2(-m(1, 2), m(2, 2));
        double sr = sin(rpy(0, 0));
        double cr = cos(rpy(0, 0));
        rpy(1) = atan2(m(0, 2), cr*m(2, 2)-sr*m(1, 2));
        rpy(2) = atan2(-m(0, 1), m(0, 0));
    }
    return rpy;
}

/**
 * @brief  将矩阵转化为机器人空间中tcp点位姿
 * @param[in] m 位姿矩阵
 * @return posMCS 是tcp点在空间中的位置和姿态
*/
Position_MCS_rad RobotModel::tr2MCS(Matrix4 m) const
{
    Position_MCS_rad posMCS(6);
    posMCS << m.block(0,3,3,1),tr2rpy(m);

    return posMCS;
}
void RobotModel::set_wine_chamfer_circle_center(Position_ACS_rad pos_acs)
{
    double d1 = axes[0]->DHJ.A; //L1
    double d2 = axes[0]->DHJ.D; //L2
    double d3 = axes[1]->DHJ.A; //L3
    double d4 = axes[1]->DHJ.D; //L4
    double d5 = axes[2]->DHJ.A; //L5
    double d6 = axes[2]->DHJ.D; //顶升电动缸导程L13
    double d8 = axes[3]->DHJ.D; //滑动电动缸导程L12
    double d9 = rad2deg(axes[2]->DHJ.Theta); //L6
    double d10 = rad2deg(axes[0]->DHJ.Alpha); //L7
    double d11 = rad2deg(axes[0]->DHJ.Theta); //L8
    double d12 = rad2deg(axes[1]->DHJ.Alpha); //L9
    double d13 = rad2deg(axes[1]->DHJ.Theta); //L10
    double d14 = rad2deg(axes[2]->DHJ.Alpha); //L11

	for(int i = 0;i < 4;i++)
	{
		wine_chamfer_circle_center.joint_pos[i] = pos_acs(i);
	}
    double delta_theta = 0;
    double delta_d5 = d5 + pos_acs(1) / (2*M_PI) * d6;
    double theta2_zero = acos((pow(d4,2) + pow(d3,2) - pow(d5,2)) / (2 * d3 * d4));
    double theta2_delta = acos((pow(d4,2) + pow(d3,2) - pow(delta_d5,2)) / (2 * d3 * d4));
    delta_theta = theta2_delta - theta2_zero;

	wine_chamfer_circle_center.pos_x = -d10 * sin(delta_theta) + d1 * cos(delta_theta) - d9 * sin(delta_theta) + pos_acs(0) / (2*M_PI) * d8;
	wine_chamfer_circle_center.pos_y = 0;
	wine_chamfer_circle_center.pos_z = d11 + d14 + d10 * cos(delta_theta) + d1 * sin(delta_theta) + d9 * cos(delta_theta) - d9 - d12 - d2 * cos(pos_acs(3)) - d13;
	printf("sign circle center %f %f %f\n",wine_chamfer_circle_center.pos_x,wine_chamfer_circle_center.pos_y,wine_chamfer_circle_center.pos_z);
}


// 全选主元高斯消去法
//a-n*n 存放方程组的系数矩阵，返回时将被破坏
//b-常数向量
//x-返回方程组的解向量
//n-存放方程组的阶数
//返回0表示原方程组的系数矩阵奇异
int RobotModel::cagaus(double a[],double b[],int n,double x[])
{
	int *js, l, k, i, j, is, p, q;
	double d, t;
	js = new int[n];
	l = 1;
	is = 0;
	for (k = 0; k <= n - 2; k++)
	{
		d = 0.0;
		for (i = k; i <= n - 1; i++)
		{
			for (j = k; j <= n - 1; j++)
			{
				t = fabs(a[i * n + j]);
				if (t > d)
				{
					d = t;
					js[k] = j;
					is = i;
				}
			}
		}
		if (d + 1.0 == 1.0)
		{
			l = 0;
		}
		else
		{
			if (js[k] != k)
			{
				for (i = 0; i <= n - 1; i++)
				{
					p = i * n + k;
					q = i * n + js[k];
					t = a[p];
					a[p] = a[q];
					a[q] = t;
				}
			}
			if (is != k)
			{
				for (j = k; j <= n - 1; j++)
				{
					p = k * n + j;
					q = is * n + j;
					t = a[p];
					a[p] = a[q];
					a[q] = t;
				}
				t = b[k];
				b[k] = b[is];
				b[is] = t;
			}
		}
		if (l == 0)
		{
			delete js;
			return (0);
		}
		d = a[k * n + k];
		for (j = k + 1; j <= n - 1; j++)
		{
			p = k * n + j;
			a[p] = a[p] / d;
		}
		b[k] = b[k] / d;
		for (i = k + 1; i <= n - 1; i++)
		{
			for (j = k + 1; j <= n - 1; j++)
			{
				p = i * n + j;
				a[p] = a[p] - a[i * n + k] * a[k * n + j];
			}
			b[i] = b[i] - a[i * n + k] * b[k];
		}
	}
	d = a[(n - 1) * n + n - 1];
	if (fabs(d) + 1.0 == 1.0)
	{
		delete js;
		return (0);
	}
	x[n - 1] = b[n - 1] / d;
	for (i = n - 2; i >= 0; i--)
	{
		t = 0.0;
		for (j = i + 1; j <= n - 1; j++)
		{
			t = t + a[i * n + j] * x[j];
		}
		x[i] = b[i] - t;
	}
	js[n - 1] = n - 1;
	for (k = n - 1; k >= 0; k--)
	{
		if (js[k] != k)
		{
			t = x[k];
			x[k] = x[js[k]];
			x[js[k]] = t;
		}
	}
	delete js;
	return 1;
}
bool RobotModel::sphereLeastFit(const std::vector<Point3D> &points,
                    double &center_x, double &center_y, double &center_z, double &radius)
{
	center_x = 0.0f;
	center_y = 0.0f;
	radius = 0.0f;

	int N = points.size();
	if (N < 4) // 最少 4 个点才能拟合出一个球面。
	{
		return false;
	}

	double sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

	for (int i = 0; i < N; i++)
	{
		double x = points[i].x;
		double y = points[i].y;
		double z = points[i].z;
		sum_x += x;
		sum_y += y;
		sum_z += z;
	}
	double mean_x = sum_x / N;
	double mean_y = sum_y / N;
	double mean_z = sum_z / N;
	double sum_u2 = 0.0f, sum_v2 = 0.0f, sum_w2 = 0.0f;
	double sum_u3 = 0.0f, sum_v3 = 0.0f, sum_w3 = 0.0f;
	double sum_uv = 0.0f, sum_uw = 0.0f, sum_vw = 0.0f;
	double sum_u2v = 0.0f, sum_uv2 = 0.0f, sum_u2w = 0.0f, sum_v2w = 0.0f, sum_uw2 = 0.0f, sum_vw2 = 0.0f;
	for (int i = 0; i < N; i++)
	{
		double u = points[i].x - mean_x;
		double v = points[i].y - mean_y;
		double w = points[i].z - mean_z;
		double u2 = u * u;
		double v2 = v * v;
		double w2 = w * w;
		sum_u2 += u2;
		sum_v2 += v2;
		sum_w2 += w2;
		sum_u3 += u2 * u;
		sum_v3 += v2 * v;
		sum_w3 += w2 * w;
		sum_uv += u * v;
		sum_vw += v * w;
		sum_uw += u * w;
		sum_u2v += u2 * v;
		sum_u2w += u2 * w;
		sum_uv2 += u * v2;
		sum_v2w += v2 * w;
		sum_uw2 += u * w2;
		sum_vw2 += v * w2;
	}

	double A[3][3];
	double B[3] =
	{ (sum_u3 + sum_uv2 + sum_uw2) / 2.0, (sum_u2v + sum_v3 + sum_vw2) / 2.0, (sum_u2w + sum_v2w + sum_w3) / 2.0 };

	A[0][0] = sum_u2;
	A[0][1] = sum_uv;
	A[0][2] = sum_uw;
	A[1][0] = sum_uv;
	A[1][1] = sum_v2;
	A[1][2] = sum_vw;
	A[2][0] = sum_uw;
	A[2][1] = sum_vw;
	A[2][2] = sum_w2;

	double ans[3];

	if (cagaus((double *) A, B, 3, ans) == 0)
	{
		return false;
	}
	else
	{
		center_x = ans[0] + mean_x;
		center_y = ans[1] + mean_y;
		center_z = ans[2] + mean_z;

		double sum = 0;
		for (int i = 0; i < N; i++)
		{
			double dx = points[i].x - center_x;
			double dy = points[i].y - center_y;
			double dz = points[i].z - center_z;
			sum += dx * dx + dy * dy + dz * dz;
		}
		sum /= N;
		radius = sqrt(sum);
	}
	return true;
}
