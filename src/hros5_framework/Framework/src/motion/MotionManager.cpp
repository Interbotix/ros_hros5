/*
 *   MotionManager.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include <math.h>
#include "FSR.h"
#include "MX28.h"
#include "MotionManager.h"
#include <unistd.h>
#include <assert.h>

using namespace Robot;

// Torque adaption every second
const int TORQUE_ADAPTION_CYCLES = 1000 / MotionModule::TIME_UNIT;
const int DEST_TORQUE = 1023;

//#define LOG_VOLTAGES 1

MotionManager* MotionManager::m_UniqueInstance = new MotionManager();

MotionManager::MotionManager() :
        m_CM730(0),
        m_ProcessEnable(false),
        m_Enabled(false),
        m_IsRunning(false),
        m_IsThreadRunning(false),
        m_IsLogging(false),
		m_torqueAdaptionCounter(TORQUE_ADAPTION_CYCLES),
		m_voltageAdaptionFactor(1.0),
        DEBUG_PRINT(false)
{
    for(int i = 0; i < JointData::NUMBER_OF_JOINTS; i++)
        m_Offset[i] = 0;

#if LOG_VOLTAGES
    assert((m_voltageLog = fopen("voltage.log", "w")));
    fprintf(m_voltageLog, "Voltage   Torque\n");
#endif
}

MotionManager::~MotionManager()
{
}

bool MotionManager::Initialize(CM730 *cm730, bool fadeIn)
{
	int value, error;

	usleep(100);
	m_CM730 = cm730;
	m_Enabled = false;
	m_ProcessEnable = true;

	if(m_CM730->Connect() == false)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Fail to connect CM-730\n");
		return false;
	}

	for(int id=JointData::ID_MIN; id<=JointData::ID_MAX; id++)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "ID:%d initializing...", id);
		
		if(m_CM730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS)
		{
			MotionStatus::m_CurrentJoints.SetValue(id, value);
			MotionStatus::m_CurrentJoints.SetEnable(id, true);

			if(DEBUG_PRINT == true)
				fprintf(stderr, "[%d] Success\n", value);
		}
		else
		{
			MotionStatus::m_CurrentJoints.SetEnable(id, false);

			if(DEBUG_PRINT == true)
				fprintf(stderr, " Fail\n");
		}
	}

	if(fadeIn)
	{
		for(int i=JointData::ID_R_SHOULDER_PITCH; i<JointData::NUMBER_OF_JOINTS; i++)
			cm730->WriteWord(i, MX28::P_TORQUE_LIMIT_L, 0, 0);
	}

	m_fadeIn = fadeIn;
	m_torque_count = 0;

	m_CalibrationStatus = 0;
	m_FBGyroCenter = 512;
	m_RLGyroCenter = 512;

	return true;
}

bool MotionManager::Reinitialize()
{
	m_ProcessEnable = false;

	m_CM730->DXLPowerOn();

	int value, error;
	for(int id=JointData::ID_MIN; id<=JointData::ID_MAX; id++)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "ID:%d initializing...", id);
		
        if(m_CM730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS)
        {
            MotionStatus::m_CurrentJoints.SetValue(id, value);
            MotionStatus::m_CurrentJoints.SetEnable(id, true);

            if(DEBUG_PRINT == true)
                fprintf(stderr, "FirstRead [%d] Success\n", value);
        
    		if(m_CM730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS)
    		{
                if ( value != MotionStatus::m_CurrentJoints.GetValue(id) )
                {
                    fprintf( stderr, "Sequential Reads differed [%d] [%d]\r\n", value, MotionStatus::m_CurrentJoints.GetValue(id) );
                }
    			MotionStatus::m_CurrentJoints.SetValue(id, value);
    			MotionStatus::m_CurrentJoints.SetEnable(id, true);

    			if(DEBUG_PRINT == true)
    				fprintf(stderr, "[%d] Success\n", value);
    		}
        }
		else
		{
			MotionStatus::m_CurrentJoints.SetEnable(id, false);

			if(DEBUG_PRINT == true)
				fprintf(stderr, " Fail\n");
		}
	}

	m_ProcessEnable = true;
	return true;
}

void MotionManager::StartLogging()
{
    char szFile[32] = {0,};

    int count = 0;
    while(1)
    {
        sprintf(szFile, "Logs/Log%d.csv", count);
        if(0 != access(szFile, F_OK))
            break;
        count++;
		if(count > 256) return;
    }
		m_LogFileStream.open(szFile, std::ios::out);
    for(int id = JointData::ID_MIN; id <= JointData::ID_MAX; id++)
        m_LogFileStream << "nID_" << id << "_GP,nID_" << id << "_PP,";
    m_LogFileStream << "GyroFB,GyroRL,AccelFB,AccelRL,L_FSR_X,L_FSR_Y,R_FSR_X,R_FSR_Y," << "\x0d\x0a";

    m_IsLogging = true;
}

void MotionManager::StopLogging()
{
    m_IsLogging = false;
    m_LogFileStream.close();
}

void MotionManager::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, OFFSET_SECTION);
}
void MotionManager::LoadINISettings(minIni* ini, const std::string &section)
{
    int ivalue = INVALID_VALUE;

    for(int i = JointData::ID_MIN; i <= JointData::ID_MAX; i++)
    {
        char key[10];
        sprintf(key, "ID_%.2d", i);
        if((ivalue = ini->geti(section, key, INVALID_VALUE)) != INVALID_VALUE)  m_Offset[i] = ivalue;
    }
		m_angleEstimator.LoadINISettings(ini, section + "_angle");
}
void MotionManager::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, OFFSET_SECTION);
}
void MotionManager::SaveINISettings(minIni* ini, const std::string &section)
{
    for(int i = JointData::ID_MIN; i <= JointData::ID_MAX; i++)
    {
        char key[10];
        sprintf(key, "ID_%.2d", i);
        ini->put(section, key, m_Offset[i]);
    }
		m_angleEstimator.SaveINISettings(ini, section + "_angle");
}

#define GYRO_WINDOW_SIZE    100
#define ACCEL_WINDOW_SIZE   30
#define MARGIN_OF_SD        2.0
void MotionManager::Process()
{
    if(m_fadeIn && m_torque_count < DEST_TORQUE) {
        m_CM730->WriteWord(CM730::ID_BROADCAST, MX28::P_TORQUE_LIMIT_L, m_torque_count, 0);
        m_torque_count += 2;
    }

    if(m_ProcessEnable == false || m_IsRunning == true)
        return;
		
		m_IsRunning = true;

    // calibrate gyro sensor
    if(m_CalibrationStatus == 0 || m_CalibrationStatus == -1)
    {
        static int fb_gyro_array[GYRO_WINDOW_SIZE] = {512,};
        static int rl_gyro_array[GYRO_WINDOW_SIZE] = {512,};
        static int buf_idx = 0;

        if(buf_idx < GYRO_WINDOW_SIZE)
        {
            if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
            {
                fb_gyro_array[buf_idx] = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L);
                rl_gyro_array[buf_idx] = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L);
                buf_idx++;
            }
        }
        else
        {
            double fb_sum = 0.0, rl_sum = 0.0;
            double fb_sd = 0.0, rl_sd = 0.0;
            double fb_diff, rl_diff;
            double fb_mean = 0.0, rl_mean = 0.0;

            buf_idx = 0;

            for(int i = 0; i < GYRO_WINDOW_SIZE; i++)
            {
                fb_sum += fb_gyro_array[i];
                rl_sum += rl_gyro_array[i];
            }
            fb_mean = fb_sum / GYRO_WINDOW_SIZE;
            rl_mean = rl_sum / GYRO_WINDOW_SIZE;

            fb_sum = 0.0; rl_sum = 0.0;
            for(int i = 0; i < GYRO_WINDOW_SIZE; i++)
            {
                fb_diff = fb_gyro_array[i] - fb_mean;
                rl_diff = rl_gyro_array[i] - rl_mean;
                fb_sum += fb_diff * fb_diff;
                rl_sum += rl_diff * rl_diff;
            }
            fb_sd = sqrt(fb_sum / GYRO_WINDOW_SIZE);
            rl_sd = sqrt(rl_sum / GYRO_WINDOW_SIZE);

            if(fb_sd < MARGIN_OF_SD && rl_sd < MARGIN_OF_SD)
            {
                m_FBGyroCenter = (int)fb_mean;
                m_RLGyroCenter = (int)rl_mean;
                m_CalibrationStatus = 1;
                if(DEBUG_PRINT == true)
                    fprintf(stderr, "FBGyroCenter:%d , RLGyroCenter:%d \n", m_FBGyroCenter, m_RLGyroCenter);
            }
            else
            {
                m_FBGyroCenter = 512;
                m_RLGyroCenter = 512;
                m_CalibrationStatus = -1;
            }
        }
    }

    if(m_CalibrationStatus == 1 && m_Enabled == true)
    {
        static int fb_array[ACCEL_WINDOW_SIZE] = {512,};
        static int buf_idx = 0;
        if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
		{
            const double GYRO_ALPHA = 0.08;
            int gyroValFB = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L) - m_FBGyroCenter;
            int gyroValRL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L) - m_RLGyroCenter;

            MotionStatus::FB_GYRO = (1.0 - GYRO_ALPHA) * MotionStatus::FB_GYRO + GYRO_ALPHA * gyroValFB;
            MotionStatus::RL_GYRO = (1.0 - GYRO_ALPHA) * MotionStatus::RL_GYRO + GYRO_ALPHA * gyroValRL;;
            MotionStatus::RL_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L);
            MotionStatus::FB_ACCEL = 1024 - m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L);

            fb_array[buf_idx] = MotionStatus::FB_ACCEL;
            if(++buf_idx >= ACCEL_WINDOW_SIZE) buf_idx = 0;

            const double TICKS_TO_RADIANS_PER_STEP = (M_PI/180.0) * 250.0/512.0 * (0.001 * MotionModule::TIME_UNIT);
            m_angleEstimator.predict(
                TICKS_TO_RADIANS_PER_STEP * gyroValFB,
                TICKS_TO_RADIANS_PER_STEP * gyroValRL,
                0
            );

            m_angleEstimator.update(
                (m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L) - 512),
                (m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L) - 512),
                (m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Z_L) - 512)
            );

            MotionStatus::ANGLE_PITCH = m_angleEstimator.pitch();
            MotionStatus::ANGLE_ROLL  = m_angleEstimator.roll();
		}

        int sum = 0, avr = 512;
        for(int idx = 0; idx < ACCEL_WINDOW_SIZE; idx++)
            sum += fb_array[idx];
        avr = sum / ACCEL_WINDOW_SIZE;

        if(avr < MotionStatus::FALLEN_F_LIMIT)
            MotionStatus::FALLEN = FORWARD;
        else if(avr > MotionStatus::FALLEN_B_LIMIT)
            MotionStatus::FALLEN = BACKWARD;
        else
            MotionStatus::FALLEN = STANDUP;

        if(m_Modules.size() != 0)
        {
            for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
            {
                (*i)->Process();
                for(int id=JointData::ID_MIN; id<=JointData::ID_MAX; id++)
                {
                    if((*i)->m_Joint.GetEnable(id) == true)
                    {
                        MotionStatus::m_CurrentJoints.SetSlope(id, (*i)->m_Joint.GetCWSlope(id), (*i)->m_Joint.GetCCWSlope(id));
                        MotionStatus::m_CurrentJoints.SetValue(id, (*i)->m_Joint.GetValue(id));

                        MotionStatus::m_CurrentJoints.SetPGain(id, (*i)->m_Joint.GetPGain(id));
                        MotionStatus::m_CurrentJoints.SetIGain(id, (*i)->m_Joint.GetIGain(id));
                        MotionStatus::m_CurrentJoints.SetDGain(id, (*i)->m_Joint.GetDGain(id));
                    }
                }
            }
        }

        int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
        int n = 0;
        int joint_num = 0;
        for(int id=JointData::ID_MIN; id<=JointData::ID_MAX; id++)
        {
            if(MotionStatus::m_CurrentJoints.GetEnable(id) == true)
            {
                param[n++] = id;

                param[n++] = MotionStatus::m_CurrentJoints.GetDGain(id);
                param[n++] = MotionStatus::m_CurrentJoints.GetIGain(id);
                int p_gain = m_voltageAdaptionFactor * MotionStatus::m_CurrentJoints.GetPGain(id);
                if(p_gain <= 0)
                    p_gain = 1;
                param[n++] = p_gain;
                param[n++] = 0;
                param[n++] = CM730::GetLowByte(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]);
                param[n++] = CM730::GetHighByte(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]);
								joint_num++;
            }

            if(DEBUG_PRINT == true)
                fprintf(stderr, "ID[%d] : %d \n", id, MotionStatus::m_CurrentJoints.GetValue(id));
        }

        if(joint_num > 0)
            m_CM730->SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);
		unsigned int ic=0;
    while(ic < m_CM730->m_DelayedWords)
			{
			m_CM730->WriteWord(m_CM730->m_DelayedAddress[ic],m_CM730->m_DelayedWord[ic],0);
			ic++;
			}
		m_CM730->m_DelayedWords = 0;
		}
		m_CM730->BulkRead();
		// update joint temps
    for(int id=JointData::ID_MIN; id<=JointData::ID_MAX; id++)
       {
       if(MotionStatus::m_CurrentJoints.GetEnable(id) == true)
					{
					int value = m_CM730->m_BulkReadData[id].ReadByte(MX28::P_PRESENT_TEMPERATURE);
					MotionStatus::m_CurrentJoints.SetTemp(id,value);
					}
			}
    if(m_IsLogging)
    {
        for(int id = JointData::ID_MIN; id <= JointData::ID_MAX; id++)
            m_LogFileStream << MotionStatus::m_CurrentJoints.GetValue(id) << "," << m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L) << ",";

        m_LogFileStream << m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadByte(FSR::P_FSR_X) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadByte(FSR::P_FSR_Y) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadByte(FSR::P_FSR_X) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadByte(FSR::P_FSR_Y) << ",";
        m_LogFileStream << "\x0d\x0a";
    }

    if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
        MotionStatus::BUTTON = m_CM730->m_BulkReadData[CM730::ID_CM].ReadByte(CM730::P_BUTTON);
    m_IsRunning = false;

    if(m_torque_count != DEST_TORQUE && --m_torqueAdaptionCounter == 0)
    {
        m_torqueAdaptionCounter = TORQUE_ADAPTION_CYCLES;
        adaptTorqueToVoltage();
    }
}

void MotionManager::SetEnable(bool enable)
{
	m_Enabled = enable;
	if(m_Enabled == true)
		m_CM730->WriteWord(CM730::ID_BROADCAST, MX28::P_MOVING_SPEED_L, 0, 0);
}

void MotionManager::AddModule(MotionModule *module)
{
	module->Initialize();
	m_Modules.push_back(module);

}

void MotionManager::RemoveModule(MotionModule *module)
{
	m_Modules.remove(module);
}

void MotionManager::SetJointDisable(int index)
{
    if(m_Modules.size() != 0)
    {
        for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
            (*i)->m_Joint.SetEnable(index, false);
    }
}

void MotionManager::adaptTorqueToVoltage()
{
    const int DEST_TORQUE = 1023;
    const int FULL_TORQUE_VOLTAGE = 130; // 13V - at 13V darwin will make no adaptation as the standard 3 cell battery is always below this voltage, this implies Nimbro-OP runs on 4 cells

    int voltage;
		// torque is only reduced if it is greater then FULL_TORQUE_VOLTAGE
    if(m_CM730->ReadByte(CM730::ID_CM, CM730::P_VOLTAGE, &voltage, 0) != CM730::SUCCESS)
        return;

    voltage = (voltage > FULL_TORQUE_VOLTAGE) ? voltage : FULL_TORQUE_VOLTAGE;
    m_voltageAdaptionFactor = ((double)FULL_TORQUE_VOLTAGE) / voltage;
    int torque = m_voltageAdaptionFactor * DEST_TORQUE;

#if LOG_VOLTAGES
    fprintf(m_voltageLog, "%3d       %4d\n", voltage, torque);
#endif

    m_CM730->WriteWord(CM730::ID_BROADCAST, MX28::P_TORQUE_LIMIT_L, torque, 0);
}
