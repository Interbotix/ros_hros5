/*
 *   JointData.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include "MX28.h"
#include "JointData.h"
#include "MotionManager.h"

using namespace Robot;

JointData::JointData()
{
    for(int i=0; i<NUMBER_OF_JOINTS; i++)
    {
        m_Enable[i] = true;
        m_Value[i] = MX28::CENTER_VALUE;
        m_Angle[i] = 0.0;
        m_CWSlope[i] = SLOPE_DEFAULT;
        m_CCWSlope[i] = SLOPE_DEFAULT;
        m_PGain[i] = P_GAIN_DEFAULT;
        m_IGain[i] = I_GAIN_DEFAULT;
        m_DGain[i] = D_GAIN_DEFAULT;
	m_Temp[i] = TEMP_DEFAULT;
    }
}

JointData::~JointData()
{
}

void JointData::SetEnable(int id, bool enable)
{
    m_Enable[id] = enable;
}

void JointData::SetEnable(int id, bool enable, bool exclusive)
{
    if(enable && exclusive) MotionManager::GetInstance()->SetJointDisable(id);
    m_Enable[id] = enable;
}

void JointData::SetEnableHeadOnly(bool enable)
{
    SetEnableHeadOnly(enable, false);
}

void JointData::SetEnableHeadOnly(bool enable, bool exclusive)
{
	SetEnable(ID_HEAD_PAN,          enable, exclusive);
	SetEnable(ID_HEAD_TILT,         enable, exclusive);
//	SetEnable(ID_TORSO_ROTATE,	enable, exclusive);
}

void JointData::SetEnableRightArmOnly(bool enable)
{
    SetEnableRightArmOnly(enable, false);
}

void JointData::SetEnableRightArmOnly(bool enable, bool exclusive)
{
    SetEnable(ID_R_SHOULDER_PITCH,  enable, exclusive);
    SetEnable(ID_R_SHOULDER_ROLL,   enable, exclusive);
    SetEnable(ID_R_ELBOW,           enable, exclusive);
//	SetEnable(ID_R_ELBOW_YAW, 	enable, exclusive);
//	SetEnable(ID_R_WRIST_YAW,	enable, exclusive);
//	SetEnable(ID_R_GRIPPER,		enable, exclusive);
}

void JointData::SetEnableLeftArmOnly(bool enable)
{
    SetEnableLeftArmOnly(enable, false);
}

void JointData::SetEnableLeftArmOnly(bool enable, bool exclusive)
{
    SetEnable(ID_L_SHOULDER_PITCH,  enable, exclusive);
    SetEnable(ID_L_SHOULDER_ROLL,   enable, exclusive);
    SetEnable(ID_L_ELBOW,           enable, exclusive);
//	SetEnable(ID_L_ELBOW_YAW, 	enable, exclusive);
//	SetEnable(ID_L_WRIST_YAW,	enable, exclusive);
//	SetEnable(ID_L_GRIPPER,		enable, exclusive);
}

void JointData::SetEnableRightLegOnly(bool enable)
{
    SetEnableRightLegOnly(enable, false);
}

void JointData::SetEnableRightLegOnly(bool enable, bool exclusive)
{
    SetEnable(ID_R_HIP_YAW,         enable, exclusive);
    SetEnable(ID_R_HIP_ROLL,        enable, exclusive);
    SetEnable(ID_R_HIP_PITCH,       enable, exclusive);
    SetEnable(ID_R_KNEE,            enable, exclusive);
    SetEnable(ID_R_ANKLE_PITCH,     enable, exclusive);
    SetEnable(ID_R_ANKLE_ROLL,      enable, exclusive);
}

void JointData::SetEnableLeftLegOnly(bool enable)
{
    SetEnableLeftLegOnly(enable, false);
}

void JointData::SetEnableLeftLegOnly(bool enable, bool exclusive)
{
    SetEnable(ID_L_HIP_YAW,         enable, exclusive);
    SetEnable(ID_L_HIP_ROLL,        enable, exclusive);
    SetEnable(ID_L_HIP_PITCH,       enable, exclusive);
    SetEnable(ID_L_KNEE,            enable, exclusive);
    SetEnable(ID_L_ANKLE_PITCH,     enable, exclusive);
    SetEnable(ID_L_ANKLE_ROLL,      enable, exclusive);
}

void JointData::SetEnableUpperBodyWithoutHead(bool enable)
{
    SetEnableUpperBodyWithoutHead(enable, false);
}

void JointData::SetEnableUpperBodyWithoutHead(bool enable, bool exclusive)
{
    SetEnableRightArmOnly(enable, exclusive);
    SetEnableLeftArmOnly(enable, exclusive);
}

void JointData::SetEnableLowerBody(bool enable)
{
    SetEnableLowerBody(enable, false);
}

void JointData::SetEnableLowerBody(bool enable, bool exclusive)
{
    SetEnableRightLegOnly(enable, exclusive);
    SetEnableLeftLegOnly(enable, exclusive);
}

void JointData::SetEnableBodyWithoutHead(bool enable)
{
    SetEnableBodyWithoutHead(enable, false);
}

void JointData::SetEnableBodyWithoutHead(bool enable, bool exclusive)
{
    SetEnableRightArmOnly(enable, exclusive);
    SetEnableLeftArmOnly(enable, exclusive);
    SetEnableRightLegOnly(enable, exclusive);
    SetEnableLeftLegOnly(enable, exclusive);
}

void JointData::SetEnableBody(bool enable)
{
    SetEnableBody(enable, false);
}

void JointData::SetEnableBody(bool enable, bool exclusive)
{
    for(int id = JointData::ID_MIN; id <= JointData::ID_MAX; id++)
        SetEnable(id, enable, exclusive);
}

bool JointData::GetEnable(int id)
{
    return m_Enable[id];
}

void JointData::SetValue(int id, int value)
{
    if(value < MX28::MIN_VALUE)
        value = MX28::MIN_VALUE;
    else if(value >= MX28::MAX_VALUE)
        value = MX28::MAX_VALUE;

    m_Value[id] = value;
    m_Angle[id] = MX28::Value2Angle(value);
}

int JointData::GetValue(int id)
{
    return m_Value[id];
}

void JointData::SetAngle(int id, double angle)
{
    if(angle < MX28::MIN_ANGLE)
        angle = MX28::MIN_ANGLE;
    else if(angle > MX28::MAX_ANGLE)
        angle = MX28::MAX_ANGLE;

    m_Angle[id] = angle;
    m_Value[id] = MX28::Angle2Value(angle);
}

double JointData::GetAngle(int id)
{
    return m_Angle[id];
}

void JointData::SetRadian(int id, double radian)
{
    SetAngle(id, radian * (180.0 / 3.141592));
}

double JointData::GetRadian(int id)
{
    return GetAngle(id) * (180.0 / 3.141592);
}

void JointData::SetSlope(int id, int cwSlope, int ccwSlope)
{
    SetCWSlope(id, cwSlope);
    SetCCWSlope(id, ccwSlope);
}

void JointData::SetCWSlope(int id, int cwSlope)
{
    m_CWSlope[id] = cwSlope;
}

int JointData::GetCWSlope(int id)
{
    return m_CWSlope[id];
}

void JointData::SetCCWSlope(int id, int ccwSlope)
{
    m_CCWSlope[id] = ccwSlope;
}

int JointData::GetCCWSlope(int id)
{
    return m_CCWSlope[id];
}
