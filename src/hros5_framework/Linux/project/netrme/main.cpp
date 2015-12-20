#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include <libgen.h>
#include <signal.h>

#include "cmd_process.h"
#include "udp.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

using namespace Robot;

int Running = 1;
LinuxCM730 linux_cm730("/dev/ttyUSB0");
CM730 cm730(&linux_cm730);
LinuxMotionTimer linuxMotionTimer;
EMOHead head_ctrl( &cm730 );
void LeaveProgram()
{
	Running=false;
  exit(0);
}

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
	printf("Sig Handled: %d\n",sig);
    exit(0);
}

bool ProcessInput(char* incoming)
{
	char *token;
  int input_len;
	char cmd[80];
  int num_param;
  int iparam[30];
  char iparams[30][10];
  //int idx = 0;
	//int apState[6]={0,0,0,0,0,0};
	char s[60];


	input_len = strlen(incoming);
  if(input_len > 0)
  {
      token = strtok( incoming, " " );
      if(token != 0)
      {
          strcpy( cmd, token );
          token = strtok( 0, " " );
          num_param = 0;
          while(token != 0)
          {
              iparam[num_param] = atoi(token);
              strncpy(iparams[num_param++],token,10);
							token = strtok( 0, " " );
          }

          if(strcmp(cmd, "exit") == 0)
          {
              if(AskSave() == false)
                  return false;
          }
          else if(strcmp(cmd, "re") == 0)
              DrawPage();
          else if(strcmp(cmd, "help") == 0)
              HelpCmd();
          else if(strcmp(cmd, "n") == 0)
              NextCmd();
          else if(strcmp(cmd, "b") == 0)
              PrevCmd();
          else if(strcmp(cmd, "time") == 0)
              TimeCmd();
          else if(strcmp(cmd, "speed") == 0)
              SpeedCmd();
          else if(strcmp(cmd, "mon") == 0)
          {
							linuxMotionTimer.Start();
							cm730.m_bIncludeTempData = true;
							cm730.MakeBulkReadPacket();// force packet to get rebuilt
							while(!kbhit(true))
							{
								MonitorServos(&cm730);
								usleep(10000);
							}

							linuxMotionTimer.Stop();
							cm730.m_bIncludeTempData = false;
							cm730.MakeBulkReadPacket();// force packet to be rebuilt
							GoToCursor(CMD_COL, CMD_ROW);
					}
          else if(strcmp(cmd, "page") == 0)
          {
              if(num_param > 0)
                  PageCmd(iparam[0]);
              else
                  PrintCmd("Need parameter");
          }
          else if(strcmp(cmd, "play") == 0)
          {
              if(num_param > 0)
                PlayCmd(&cm730, iparam[0]);
              else
								PlayCmd(&cm730,IndexPage());
          }
          else if(strcmp(cmd, "set") == 0)
          {
              if(num_param > 0)
                  SetValue(&cm730, iparam[0]);
              else
                  PrintCmd("Need parameter");
          }
          else if(strcmp(cmd, "list") == 0)
              ListCmd();
          else if(strcmp(cmd, "on") == 0)
          {
              OnOffCmd(&cm730, true, num_param, iparam, iparams);
          }
          else if(strcmp(cmd, "off") == 0)
          {
              OnOffCmd(&cm730, false, num_param, iparam, iparams);
          }
          else if(strcmp(cmd, "w") == 0)
          {
              if(num_param > 0)
                  WriteStepCmd(iparam[0]);
              else
                  PrintCmd("Need parameter");
          }
          else if(strcmp(cmd, "d") == 0)
          {
              if(num_param > 0)
                  DeleteStepCmd(iparam[0]);
              else
                  PrintCmd("Need parameter");
          }
          else if(strcmp(cmd, "i") == 0)
          {
              if(num_param == 0)
                  InsertStepCmd(0);
              else
                  InsertStepCmd(iparam[0]);
          }
          else if(strcmp(cmd, "m") == 0)
          {
              if(num_param > 1)
                  MoveStepCmd(iparam[0], iparam[1]);
              else
                  PrintCmd("Need parameter");
          }
          else if(strcmp(cmd, "copy") == 0)
          {
              if(num_param > 0)
                  CopyCmd(iparam[0]);
              else
                  PrintCmd("Need parameter");
          }
          else if(strcmp(cmd, "new") == 0)
              NewCmd();
          else if(strcmp(cmd, "g") == 0)
          {
              if(num_param > 0)
                  GoCmd(&cm730, iparam[0]);
              else
                  PrintCmd("Need parameter");
          }
          else if(strcmp(cmd, "poweroff") == 0)
					{
						cm730.DXLPowerOn(false);
					}
          else if(strcmp(cmd, "poweron") == 0)
					{
						cm730.DXLPowerOn(true);
					}
					else if(strcmp(cmd, "save") == 0)
					{
							if(num_param > 0)
                SaveCmd(iparam[0]);
              else
								SaveCmd(IndexPage());
					}
          else if(strcmp(cmd, "name") == 0)
              NameCmd();
          else
          {
							sprintf(s,"Bad command (%s:%d)! please input 'help'",cmd,strlen(cmd));
							PrintCmd(s);
					}
      }
  }
  return true;
}

#include "udp_proc.h"

int main(int argc, char *argv[])
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);


    char filename[128];

    change_current_dir();

    strcpy(filename, MOTION_FILE_PATH); // Set default motion file path
    if(Action::GetInstance()->LoadFile(filename) == false)
    {
    	printf("Can not open %s\n", filename);
    	LeaveProgram();
    }

    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Initializing Motion Manager failed!\n");
        exit(0);
    }

		MotionManager::GetInstance()->SetEnable(false);
		MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
		linuxMotionTimer.Initialize(MotionManager::GetInstance());
		linuxMotionTimer.Stop();
head_ctrl.SetMode( EMOHead::COMMAND );
head_ctrl.SetLookDirection( EMOHead::LOOK_FORWARD );
		char* b;
		initUDP( &UDP_Command_Handler, &UDP_Control_Handler, &Running );
		pthread_create( &udplistenerThread, NULL, &udplistener_Thread, NULL );

		pthread_join(udplistenerThread,(void**)&b);

    LeaveProgram();
}
