#ifndef UDP_PROC
#define UDP_PROC

#define DEBUG

//TODO: Unsafe pch handling with atoi etc.. improve.
//TODO: Not sending page time or acceleration values. Enhance.

char tempMessage[4096];

void PopulateStepCurrent( CM730 *cm730, Action::STEP *CurrentStep )
{
	int value;
	
	for(int id=0; id<31;++id)
	{
		if(id>=JointData::ID_MIN && id<=JointData::ID_MAX )
		{
			if( cm730->ReadByte( id, MX28::P_TORQUE_ENABLE, &value, 0 ) == CM730::SUCCESS )
			{
				if(value == 1)
				{
					if(cm730->ReadWord(id, MX28::P_GOAL_POSITION_L, &value, 0) == CM730::SUCCESS)		
					{		
						CurrentStep->position[id] = value;
					}
					else
					{
						CurrentStep->position[id] = Action::INVALID_BIT_MASK;
					}
				}
				else if ( value == 0 )
				{
					if(cm730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)				
					{
						CurrentStep->position[id] = value|Action::TORQUE_OFF_BIT_MASK;
					}
					else
					{
						CurrentStep->position[id] = Action::INVALID_BIT_MASK;
					}
				}
				else
				{
					CurrentStep->position[id] = Action::INVALID_BIT_MASK;
				}
			}
			else
			{
				CurrentStep->position[id] = Action::INVALID_BIT_MASK;
			}
		}
	}
}

void SetTorque(CM730 *cm730, int ServoID, int on)
{
	if ( cm730->WriteByte(ServoID, MX28::P_TORQUE_ENABLE, on, 0) != CM730::SUCCESS  )
	{
		printf( "Warning: SetTorque[%d] did not return success for servo: %d\r\n", on, ServoID );
	}
#ifdef DEBUG
	int value;
	if(cm730->ReadWord(ServoID, MX28::P_PRESENT_POSITION_L, &value, 0) != CM730::SUCCESS)
	{
		printf( "Warning: SetTorque[%d] did not return success for reading servo: %d\r\n", on, ServoID );
	}
#endif	
}

void TorqueAllServos(CM730 *cm730, char * input)
{
	int on;
	if(input[0]=='1')
	{
		on=1;
	}
	else if(input[0]=='0')
	{
		on=0;
	}
	else 
	{
		printf( "Error: TorqueAllServos could not process the value: %s\r\n", input );
		return;
	}
	
#ifdef DEBUG
	printf( "netrme: Setting TorqueAllServos to  %d\r\n", on );
#endif

	for( int i = JointData::ID_MIN; i <= JointData::ID_MAX; ++i )
	{
		SetTorque( cm730, i, on );
	}
}

void ListPose( int pose, char* response )
{
	Action::PAGE posePage;
	Action::GetInstance()->LoadPage(pose, &posePage);
	
	sprintf(response,"ALLPOSE%d:",pose);
	
	if( strlen( (char*)&posePage.header.name ) > 0 )
	{
		char* tok = strtok((char*)&posePage.header.name,"\r\n");
		sprintf(response+strlen(response),"%s:",tok);
	}
	
	for( int step = 0; step < 20; ++step )
	{
		Action::STEP *poseStep = &posePage.step[step];
		sprintf( response + strlen( response ), ";%d;", step );
		
		for( int id = JointData::ID_MIN; id <= JointData::ID_MAX; id++ )
		{
			if( ( poseStep->position[id] & Action::INVALID_BIT_MASK ) || ( poseStep->position[id] & Action::TORQUE_OFF_BIT_MASK ) )
			{
				step=20;
				break;
			}
			else sprintf(response+strlen(response),"%d:%d,",id,poseStep->position[id]);
		}
		sprintf(response+strlen(response),"P:%d,T:%d,",poseStep->pause,poseStep->time);
	}	
}

void GotoPose( CM730 *cm730, char* input )
{
	Action::STEP FinalStep;
	//GOTOPOSE:1:774,2:543,3:456,4:611,5:352,6:417,7:515,8:519,9:516,10:530,11:266,12:760,13:34,14:989,15:746,16:281,17:515,18:528,19:641,20:520,P:0,T:120,
	input+=9;
	
	char * pch;
	pch = strtok( input, ",:" );
	
	while( pch != 0 )
	{
		if( pch[ 0 ] == 'P' || pch[ 0 ] == 'T' )
		{
			pch = strtok( 0, ",:" );
			pch = strtok( 0, ",:" );
		}
		else
		{
			int index = atoi( pch );		
			pch = strtok( 0, ",:" );
			int value = atoi( pch );
			pch = strtok( 0, ",:" );
			FinalStep.position[index] = value;
		}
	}
	
	Action::PAGE tPage;
	Action::GetInstance()->ResetPage(&tPage);
	Action::STEP CurrentStep;
	PopulateStepCurrent(cm730,&CurrentStep);
	
	for( int id = JointData::ID_MIN; id <= JointData::ID_MAX; id++ )
	{
		MotionStatus::m_CurrentJoints.SetValue(id, CurrentStep.position[id]);
		tPage.step[0].position[id] = CurrentStep.position[id];
		tPage.step[1].position[id] = FinalStep.position[id];		
	}
	
	tPage.step[0].pause=0;
	tPage.step[0].time = 0;
	tPage.step[1].time = 60;
	tPage.header.stepnum = 2;
	tPage.header.repeat = 1;
	Action::GetInstance()->m_Joint.SetEnableBody(true, true);
	MotionManager::GetInstance()->SetEnable(true);
	linuxMotionTimer.Start();
	
	if(Action::GetInstance()->Start(0, &tPage) == false)
	{
		MotionManager::GetInstance()->SetEnable(false);
		return;
	}
	
	while(Action::GetInstance()->IsRunning())
	{			
		usleep(10000);	
	}
	
	MotionManager::GetInstance()->SetEnable(false);
	linuxMotionTimer.Stop();

}

void GetCurrentPose( CM730 *cm730, char* response )
{
	Action::STEP CurrentStep;
	PopulateStepCurrent( cm730,&CurrentStep );
			
	sprintf( response, "CURRENTPOSE:" );	
	for( int id = JointData::ID_MIN; id <= JointData::ID_MAX; id++ )
	{
		if( CurrentStep.position[ id ] & Action::INVALID_BIT_MASK )
		{
			sprintf( response + strlen( response ), "%d:X,", id );
		}
		else if( CurrentStep.position[ id ] & Action::TORQUE_OFF_BIT_MASK )
		{
			unsigned int pos = CurrentStep.position[ id ];
			pos &= ~Action::TORQUE_OFF_BIT_MASK;
			sprintf( response + strlen( response ), "%d:%d,", id, -pos );
		}
		else 
		{
			sprintf( response + strlen( response ), "%d:%d,", id, CurrentStep.position[ id ] );
		}
	}
}

void SetServo( CM730 *cm730, char* input )
{
	//SETSERVO:1:774
	input+=9;
	
	char * pch;
	pch = strtok( input, ",:" );
	
	while( pch != 0 )
	{		
		int index = atoi( pch );		
		pch = strtok( 0, ",:" );
		int value = atoi( pch );
		pch = strtok( 0, ",:" );
		
		int error;
		if ( cm730->WriteWord(index, MX28::P_GOAL_POSITION_L, value, &error) != CM730::SUCCESS )
		{
			printf( "Warning: SetServo did not return success for servo %d, error was %d\r\n", index, error );
		}
	}
}

void SetServoSpeeds( CM730 *cm730, char* input )
{
	//SETSPEED:0-1023
	input+=9;
	
	char * pch;
	pch = strtok( input, ",:" );
	
	if ( pch != 0 )
	{		
		int speed = atoi( pch );
		if ( speed < 0 ) speed = 0;
		if ( speed > 1023 ) speed = 1023;
		
		int error;
		if ( cm730->WriteWord(CM730::ID_BROADCAST, MX28::P_MOVING_SPEED_L, speed, &error) != CM730::SUCCESS )
		{
			printf( "Warning: SetServoSpeeds did not return success with value %d error was %d\r\n", speed, error );
		}
	}
}

void ListPages( char * message )
{
	Action::PAGE page;

	sprintf(message,"LIST:");
	for( int k=0; k<Action::MAXNUM_PAGE; ++k)
	{
		if( Action::GetInstance()->LoadPage( k, &page) == true )
		{
			if( strlen( (char*)page.header.name ) > 0 )
			{
				char* tok = strtok((char*)page.header.name,"\r\n");
				sprintf(message+strlen(message),"%d:%s,",k,tok);
			}
		}
	}
}

bool StorePage( char* input )
{
	//STOREPAGE:#:TITLE|1:774,2:543,3:456,4:611,5:352,6:417,7:515,8:519,9:516,10:530,11:266,12:760,13:34,14:989,15:746,16:281,17:515,18:528,19:641,20:520,P:0,T:120,|

	int pageNumber = -1;
	char pageName[14] = {0};
	
	Action::PAGE tPage;
	Action::GetInstance()->ResetPage(&tPage);
	
	char * pch;
	pch = strtok( input, ",:" );
	
	//Get page number
	pageNumber = atoi( pch );
	pch = strtok( 0, ",:|" );
	
	//Get page name
	strcpy( pageName, pch );
	pch = strtok( 0, ",:" );
	
	int i = 0;
	for(; i <=7 && pch != 0; ++i )
	{
		if( pch[ 0 ] == '|' )
		{
			pch++;
		}
		if(pch[0]==0)break;
		
		while( pch != 0 && pch[ 0 ] != '|')
		{
			if( pch[ 0 ] == 'P' )
			{
				pch = strtok( 0, ",:" );				
				tPage.step[ i ].pause = atoi( pch );		
				pch = strtok( 0, ",:" );						
			}
			else if( pch[ 0 ] == 'T' )
			{
				pch = strtok( 0, ",:" );		
				tPage.step[i].time = atoi( pch );;
				pch = strtok( 0, ",:" );
			}
			else
			{
				if( pch[ 0 ] > 0x39 || pch[ 0 ] < 0x30 )break;
				int index = atoi( pch );
				pch = strtok( 0, ",:" );
				int value = atoi( pch );
				pch = strtok( 0, ",:" );
				tPage.step[ i ].position[ index ] = value;
			}
		}
	}
	
	strcpy( (char*)tPage.header.name, pageName );
	
	tPage.header.stepnum = i;	
	tPage.header.repeat = 1;
	tPage.header.schedule = Action::TIME_BASE_SCHEDULE;
	tPage.header.speed = 28;
	tPage.header.accel = 255;

#ifdef DEBUG
	printf( "netrme: StorePage: name(%s) position(%d) steps(%d)\r\n", (char*)tPage.header.name, pageNumber,  tPage.header.stepnum);
#endif

	if(Action::GetInstance()->SavePage(pageNumber, &tPage) == true)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void PlayPage( CM730 *cm730, char* input )
{
	//PLAYPOSE:|1:774,2:543,3:456,4:611,5:352,6:417,7:515,8:519,9:516,10:530,11:266,12:760,13:34,14:989,15:746,16:281,17:515,18:528,19:641,20:520,P:0,T:120,|
	input+=9;
	
	Action::PAGE tPage;
	Action::GetInstance()->ResetPage(&tPage);
	
	char * pch;
	pch = strtok( input, ",:" );
	
	Action::STEP CurrentStep;
	PopulateStepCurrent(cm730,&CurrentStep);
	
	for( int id = JointData::ID_MIN; id <= JointData::ID_MAX; id++ )
	{
		MotionStatus::m_CurrentJoints.SetValue(id, CurrentStep.position[id]);
		tPage.step[0].position[id] = CurrentStep.position[id];
	}
	
	tPage.step[0].pause = 0;
	tPage.step[0].time = 30;
	
	int i = 0;
	for(; i <=7 && pch != 0; ++i )
	{		
		if( pch[ 0 ] == '|' )
		{
			pch++;
		}
		if(pch[0]==0)break;
		
		while( pch != 0 && pch[ 0 ] != '|')
		{
			if( pch[ 0 ] == 'P' )
			{
				pch = strtok( 0, ",:" );				
				tPage.step[ i ].pause = atoi( pch );		
				pch = strtok( 0, ",:" );						
			}
			else if( pch[ 0 ] == 'T' )
			{
				pch = strtok( 0, ",:" );		
				tPage.step[i].time = atoi( pch );;
				pch = strtok( 0, ",:" );
			}
			else
			{
				if( pch[ 0 ] > 0x39 || pch[ 0 ] < 0x30 )break;
				int index = atoi( pch );
				pch = strtok( 0, ",:" );
				int value = atoi( pch );
				pch = strtok( 0, ",:" );
				tPage.step[ i ].position[ index ] = value;
			}
		}
	}
	
	tPage.header.stepnum = i;	
	tPage.header.repeat = 1;
	tPage.header.schedule = Action::TIME_BASE_SCHEDULE;
	tPage.header.speed = 28;
	tPage.header.accel = 255;
	Action::GetInstance()->m_Joint.SetEnableBody(true, true);
	MotionManager::GetInstance()->SetEnable(true);
	linuxMotionTimer.Start();
	
	if(Action::GetInstance()->Start(2, &tPage) == false)
	{
		MotionManager::GetInstance()->SetEnable(false);
		return;
	}
	
	while(Action::GetInstance()->IsRunning())
	{
		usleep(10000);	
	}
	
	MotionManager::GetInstance()->SetEnable(false);
	linuxMotionTimer.Stop();
	
	GetCurrentPose( cm730, tempMessage );
	UDPBindSend( tempMessage, strlen( tempMessage ) );	
}

void UDP_Command_Handler( char * p_udpin )
{
#ifdef DEBUG
	printf( "netrme: UDP Command Packet Received: %s\r\n", p_udpin );
#endif

	if( strcmp( p_udpin, "LIST" ) == 0 )
	{
		ListPages(tempMessage);
	}
	else if( memcmp( p_udpin, "POSE", 4 ) == 0 )
	{	
		char* tok = strtok( p_udpin, " " );
		tok = strtok( 0, " " );
		int pose = atoi( tok );
		ListPose(pose, tempMessage);				
	}
	else if( strcmp( p_udpin, "GETPOSE" ) == 0 )
	{
		GetCurrentPose(&cm730,tempMessage);
	}
	else if( memcmp( p_udpin, "TORQUEALL:", 10 ) == 0 )
	{
		TorqueAllServos( &cm730, p_udpin+10 );
		sprintf( tempMessage, p_udpin );
	}
	else if( memcmp( p_udpin, "TORQUE", 6 ) == 0 )
	{
		char* tok = strtok( p_udpin, " :" );
		tok = strtok( 0, " :" );
		int servoid = atoi( tok );
		tok = strtok( 0, " :" );
		int onoff = atoi( tok );
		SetTorque( &cm730, servoid, onoff );
		GetCurrentPose( &cm730, tempMessage );
	}
	else if( memcmp( p_udpin, "GOTOPOSE:", 9 ) == 0 )
	{
		GotoPose( &cm730, p_udpin );
		return;
	}
	else if( memcmp( p_udpin, "PLAYPAGE:", 9 ) == 0 )
	{
//NOTE: For safety, especially during debugging, make sure motor torque is on before playing!
		TorqueAllServos( &cm730, (char*)"1" );		
		PlayPage( &cm730, p_udpin );
		return;
	}
	else if( memcmp( p_udpin, "STOREPAGE:", 10 ) == 0 )
	{
		if ( StorePage( p_udpin + 10 ) )
		{
			sprintf( tempMessage, "STOREPAGE:OK" );
		}
		else
		{
			sprintf( tempMessage, "STOREPAGE:FAIL" );
		}
	}
	else if( memcmp( p_udpin, "SETSERVO:", 9 ) == 0 )
	{
		SetServo( &cm730, p_udpin );
		return;
	}
	else if( memcmp( p_udpin, "SETSPEED:", 9 ) == 0 )
	{
		SetServoSpeeds( &cm730, p_udpin );
		return;
	}
	else if( memcmp( p_udpin, "REBOOT", 6 ) == 0 )
	{
		UDPBindSend( (char*)"REBOOTING", strlen( "REBOOTING" ) );
		Running=false;
		usleep(100000);		
		system("reboot");
		return;
	}
	else if( memcmp( p_udpin, "POWEROFF", 8 ) == 0 )
	{
		UDPBindSend( (char*)"POWEROFF", strlen( "POWEROFF" ) );
		Running=false;
		usleep(100000);		
		system("poweroff");
		return;
	}
	
	UDPBindSend( tempMessage, strlen(tempMessage) );
}

void UDP_Control_Handler( char * p_udpin )
{
#ifdef DEBUG
	printf( "netrme: UDP Control Packet Received: %s\r\n", p_udpin );
#endif

	//On bind torque the servos and set default torque limit and speeds for basic control
	if ( memcmp( p_udpin, "BIND", 4 ) == 0 ) 
	{
		TorqueAllServos( &cm730, (char*)"1" );
		
		if ( cm730.WriteWord(CM730::ID_BROADCAST, MX28::P_TORQUE_LIMIT_L, 684, 0) != CM730::SUCCESS  )
		{
			printf( "Warning: TODO\r\n");
		}
		if ( cm730.WriteWord(CM730::ID_BROADCAST, MX28::P_MOVING_SPEED_L, 256, 0) != CM730::SUCCESS )
		{
			printf( "Warning: TODO\r\n");
		}

		head_ctrl.SetMode( EMOHead::COMMAND );
		head_ctrl.SetLookDirection( EMOHead::LOOK_FORWARD );
	}
}


#endif
