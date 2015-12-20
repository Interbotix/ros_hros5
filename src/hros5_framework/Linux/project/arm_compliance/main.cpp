#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include <libgen.h>
#include <signal.h>

#include "LinuxDARwIn.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif
#define INI_FILE_PATH       ((char *)"../../../Data/config.ini")

using namespace Robot;

LinuxCM730 linux_cm730("/dev/ttyUSB0");
CM730 cm730(&linux_cm730);
LinuxMotionTimer linuxMotionTimer;

EMOHead head_ctrl( &cm730 );

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
    struct termios term;
    tcgetattr( STDIN_FILENO, &term );
    term.c_lflag |= ICANON | ECHO;
		tcsetattr( STDIN_FILENO, TCSANOW, &term );

    exit(0);
}

void blink_with_delay( int delay )
{
    head_ctrl.BlinkNow();
    usleep( delay );
}

int main(int argc, char *argv[])
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

minIni* ini = new minIni(INI_FILE_PATH);

    char filename[128];

    change_current_dir();
    if(argc < 2)
        strcpy(filename, MOTION_FILE_PATH); // Set default motion file path
    else
        strcpy(filename, argv[1]);

    //////////////////// Framework Initialize ////////////////////////////	
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Initializing Motion Manager failed!\n");
        exit(0);
    }
    MotionManager::GetInstance()->LoadINISettings(ini);
    MotionManager::GetInstance()->SetEnable(true);
    linuxMotionTimer.Initialize(MotionManager::GetInstance());
    linuxMotionTimer.Start();

    MotionStatus::m_CurrentJoints.SetEnableRightArmOnly(true, false);
    MotionStatus::m_CurrentJoints.SetEnableLeftArmOnly(true, false);
    /////////////////////////////////////////////////////////////////////

    head_ctrl.SetMode( EMOHead::COMMAND );



    //head_ctrl.SetMode( EMOHead::DEMO );

    exit(0);
}
