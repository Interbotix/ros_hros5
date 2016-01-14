 /*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>

#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "StatusCheck.h"


#define MOTION_FILE_PATH    ((char *)"../../../Data/motion_4096.bin")

#define INI_FILE_PATH       ((char *)"../../../Data/config.ini")

//#define M_INI	((char *)"../../../Data/slow-walk.ini")


#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);
int GetCurrentPosition(CM730 &cm730);
////////////////////////////////////////////
Action::PAGE Page;
Action::STEP Step;
////////////////////////////////////////////
int change_current_dir()
{
    char exepath[1024] = {0};
    int status = 0;
		if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        status = chdir(dirname(exepath));
		return status;
}

int main(int argc, char *argv[])
{
		int trackerSel;    
		change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH); 
		//minIni* ini1 = new minIni(M_INI); 
		StatusCheck::m_ini = ini;
		//StatusCheck::m_ini1 = ini1;

    //////////////////// Framework Initialize ////////////////////////////
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if(MotionManager::GetInstance()->Initialize(&cm730) == false)
        {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }

    Walking::GetInstance()->LoadINISettings(ini);
	usleep(100);
    MotionManager::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    //MotionManager::GetInstance()->StartThread();
    //LinuxMotionTimer::Initialize(MotionManager::GetInstance());
    LinuxMotionTimer linuxMotionTimer;
		linuxMotionTimer.Initialize(MotionManager::GetInstance());
		linuxMotionTimer.Start();
   /////////////////////////////////////////////////////////////////////
//	MotionManager::GetInstance()->LoadINISettings(ini);

    int firm_ver = 0,retry=0;
    //important but allow a few retries
		while(cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)
    {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
        retry++;
				if(retry >=3) exit(1);// if we can't do it after 3 attempts its not going to work.
    }

    if(0 < firm_ver && firm_ver < 40)
    {
        Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
    }
    else
	{
	fprintf(stderr, "Wrong firmware version %d!! \n\n", JointData::ID_HEAD_PAN);	
        exit(0);
	}
		//conversion! ////////////////
		/*
		Action::GetInstance()->LoadFile("../../../Data/motion.bin");
		int j,k,p,a;
		double f;		
		for(k=0;k<Action::MAXNUM_PAGE;k++)
			{
			Action::GetInstance()->LoadPage(k, &Page);
			for(j=0;j<Action::MAXNUM_STEP;j++)
				{
				for(p=0;p<31;p++)
					{
					a = Page.step[j].position[p];
					if(a < 1024)
						{
						f = ((a-512)*10)/3+2048;						
						a = (int)f;						
						if(a<0) a =0;
						if(a>4095) a = 4095;						
						Page.step[j].position[p] = a;						
						}						
					}				
				}
			Action::GetInstance()->SavePage(k, &Page);
			}
		exit(0);
		*/
		//copy page ////////////////
		if(argc>1 && strcmp(argv[1],"-copy")==0)
			{
			printf("Page copy -- uses files motion_src.bin and motion_dest.bin\n");
			if(Action::GetInstance()->LoadFile((char *)"../../../Data/motion_src.bin") == false)
				{
				printf("Unable to open source file\n");
				exit(1);
				}
			int k;
			void *page1;

			page1 = malloc(sizeof(Robot::Action::PAGE));
			printf("Page to load:");
			if(scanf("%d",&k) != EOF)
				{
				if(Action::GetInstance()->LoadPage(k, (Robot::Action::PAGE *)page1) == false)
					{
					printf("Unable to load page %d\n",k);
					exit(1);
					}
				if(Action::GetInstance()->LoadFile((char *)"../../../Data/motion_dest.bin") == false)
					{
					printf("Unable to open destination file\n");
					exit(1);
					}
				if(Action::GetInstance()->SavePage(k, (Robot::Action::PAGE *)page1) == false)
					{
					printf("Unable to save page %d\n",k);
					exit(1);
					}
				printf("Completed successfully.\n");
				exit(0);
				}
			}
		/////////////////////////////
/*
    Walking::GetInstance()->m_Joint.SetEnableBody(true,true);
    MotionManager::GetInstance()->SetEnable(true);

		Walking::GetInstance()->LoadINISettings(m_ini);                  

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01|0x02|0x04, NULL);

    PS3Controller_Start();
		LinuxActionScript::PlayMP3("../../../Data/mp3/ready.mp3");
    Action::GetInstance()->Start(15);
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
*/
		Walking::GetInstance()->LoadINISettings(ini);   
MotionManager::GetInstance()->LoadINISettings(ini); 

    Walking::GetInstance()->m_Joint.SetEnableBody(false);
    Head::GetInstance()->m_Joint.SetEnableBody(false);
    Action::GetInstance()->m_Joint.SetEnableBody(true);
    MotionManager::GetInstance()->SetEnable(true);
             
MotionManager::GetInstance()->ResetGyroCalibration();

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x02, NULL);

	if(PS3Controller_Start() == 0)
		printf("PS3 controller not installed.\n");
	
	cm730.WriteWord(CM730::P_LED_HEAD_L, cm730.MakeColor(1,1,1),0);
	//determine current position
	StatusCheck::m_cur_mode = GetCurrentPosition(cm730);
	//LinuxActionScript::PlayMP3("../../../Data/mp3/ready.mp3");
	if((argc>1 && strcmp(argv[1],"-off")==0) || (StatusCheck::m_cur_mode == SITTING))
	{
		cm730.DXLPowerOn(false);
		//for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
		//	cm730.WriteByte(id, MX28::P_TORQUE_ENABLE, 0, 0);
	}
	else
	{
		Action::GetInstance()->Start(15);
		while(Action::GetInstance()->IsRunning()) usleep(8*1000);
	}

	const double pitch_error_max = 10.0; //degrees
	const double nominal_pitch = -8.0; //degrees
	double hip_pitch_default = Walking::GetInstance()->HIP_PITCH_OFFSET;
	double x_offset_default = Walking::GetInstance()->X_OFFSET;
	double pitch_offset_default = Walking::GetInstance()->P_OFFSET;
	double pitch_smoothed = MotionStatus::ANGLE_PITCH * 57.2958;
	double x_offset_smoothed = x_offset_default;
	int printSkipCount = 0;
    while(1)
	{
     	StatusCheck::Check(cm730);

		if(StatusCheck::m_is_started == 0)
        continue;
/*
    	pitch_smoothed = (0.95 * pitch_smoothed) + ( 0.05 * (MotionStatus::ANGLE_PITCH * 57.2958) );
    	double pitch_degrees = pitch_smoothed;
    	
    	double pitch_error = nominal_pitch - pitch_degrees; //Neg=Back, Pos=Fwd
    	if ( pitch_error > pitch_error_max )
    	{
    		pitch_error = pitch_error_max;
    	}
    	else if ( pitch_error < -pitch_error_max )
    	{
    		pitch_error = -pitch_error_max;
    	}

    	double hip_pitch_correction = hip_pitch_default - pitch_error;
    	double x_offset_correction = x_offset_default + (2.0*pitch_error);
    	x_offset_smoothed = (0.9 * x_offset_smoothed) + ( 0.1 * x_offset_correction);
    	x_offset_correction = x_offset_smoothed;
    	
    	double pitch_offset_correction = pitch_offset_default - pitch_error;
*/
	//Walking::GetInstance()->HIP_PITCH_OFFSET = hip_pitch_correction;
	//Walking::GetInstance()->X_OFFSET = x_offset_correction;
//Walking::GetInstance()->P_OFFSET = pitch_offset_correction;

	/*
    	if ( ++printSkipCount > 10000 )
    	{
    		printSkipCount = 0;
    		printf( "Pitch: %0.2f, Error: %0.2f, HipOffset: %0.2f, XOffset: %0.2f, PitchOffset: %0.2f\r\n", pitch_degrees, pitch_error, hip_pitch_correction, x_offset_correction, pitch_offset_correction );
    	}
    */
    	//usleep(10000);
	}

    return 0;
}

int GetCurrentPosition(CM730 &cm730)
{
	int m=Robot::READY,p,j,pos[31];
	int dMaxAngle1,dMaxAngle2,dMaxAngle3;
	double dAngle;
	int rl[6] = { JointData::ID_R_ANKLE_ROLL,JointData::ID_R_ANKLE_PITCH,JointData::ID_R_KNEE,JointData::ID_R_HIP_PITCH,JointData::ID_R_HIP_ROLL,JointData::ID_R_HIP_YAW };
	int ll[6] = { JointData::ID_L_ANKLE_ROLL,JointData::ID_L_ANKLE_PITCH,JointData::ID_L_KNEE,JointData::ID_L_HIP_PITCH,JointData::ID_L_HIP_ROLL,JointData::ID_L_HIP_YAW };

	for(p=0;p<31;p++) 
		{
		pos[p]	= -1;
		}
	for(p=0; p<6; p++)
		{
		if(cm730.ReadWord(rl[p], MX28::P_PRESENT_POSITION_L, &pos[rl[p]], 0) != CM730::SUCCESS)
			{
			printf("Failed to read position %d",rl[p]);
			}
		if(cm730.ReadWord(ll[p], MX28::P_PRESENT_POSITION_L, &pos[ll[p]], 0) != CM730::SUCCESS)
			{
			printf("Failed to read position %d",ll[p]);
			}
		}
	// compare to a couple poses
	// first sitting - page 48
	Action::GetInstance()->LoadPage(48, &Page);
	j = Page.header.stepnum-1;
	dMaxAngle1 = dMaxAngle2 = dMaxAngle3 = 0;
	for(p=0;p<6;p++)
		{
		dAngle = abs(MX28::Value2Angle(pos[rl[p]]) - MX28::Value2Angle(Page.step[j].position[rl[p]]));
		if(dAngle > dMaxAngle1)
			dMaxAngle1 = dAngle;
		dAngle = abs(MX28::Value2Angle(pos[ll[p]]) - MX28::Value2Angle(Page.step[j].position[ll[p]]));
		if(dAngle > dMaxAngle1)
			dMaxAngle1 = dAngle;
		}				
	// squating - page 15
	Action::GetInstance()->LoadPage(15, &Page);
	j = Page.header.stepnum-1;
	for(int p=0;p<6;p++)
		{
		dAngle = abs(MX28::Value2Angle(pos[rl[p]]) - MX28::Value2Angle(Page.step[j].position[rl[p]]));
		if(dAngle > dMaxAngle2)
			dMaxAngle2 = dAngle;
		dAngle = abs(MX28::Value2Angle(pos[ll[p]]) - MX28::Value2Angle(Page.step[j].position[ll[p]]));
		if(dAngle > dMaxAngle2)
			dMaxAngle2 = dAngle;
		}				
	// walkready - page 9
	Action::GetInstance()->LoadPage(9, &Page);
	j = Page.header.stepnum-1;
	for(int p=0;p<6;p++)
		{
		dAngle = abs(MX28::Value2Angle(pos[rl[p]]) - MX28::Value2Angle(Page.step[j].position[rl[p]]));
		if(dAngle > dMaxAngle3)
			dMaxAngle3 = dAngle;
		dAngle = abs(MX28::Value2Angle(pos[ll[p]]) - MX28::Value2Angle(Page.step[j].position[ll[p]]));
		if(dAngle > dMaxAngle3)
			dMaxAngle3 = dAngle;
		}				
	if(dMaxAngle1<20 && dMaxAngle1<dMaxAngle2 && dMaxAngle1<dMaxAngle3)
		m = Robot::SITTING;
	if(dMaxAngle2<20 && dMaxAngle2<dMaxAngle1 && dMaxAngle2<dMaxAngle3)
		m = Robot::READY;
	if(dMaxAngle3<20 && dMaxAngle3<dMaxAngle1 && dMaxAngle3<dMaxAngle2)
		m = Robot::SOCCER;
	printf("Sitting = %d, Squating = %d, Standing = %d\n",dMaxAngle1,dMaxAngle2,dMaxAngle3);
	printf("Robot is %s\n",m==Robot::READY?"Ready":m==Robot::SOCCER?"Soccer":m==Robot::SITTING?"Sitting":"None");
	return m;
}
