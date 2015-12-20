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
    MotionManager::GetInstance()->SetEnable(false);
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());	
    linuxMotionTimer.Initialize(MotionManager::GetInstance());
    linuxMotionTimer.Stop();
    /////////////////////////////////////////////////////////////////////

    head_ctrl.SetMode( EMOHead::COMMAND );

/* BLINKS */

    for ( int i = 0; i < 10; ++i )
    {
        blink_with_delay( 200000 );
    }
    
LinuxActionScript::PlayMP3("../../../Data/mp3/chappie_chappie.mp3");

/* EYE DIRECTIONS */

    head_ctrl.SetLookDirection( EMOHead::LOOK_LEFT );
    sleep( 1 );
    head_ctrl.SetLookDirection( EMOHead::LOOK_RIGHT );
    sleep( 1 );

LinuxActionScript::PlayMP3("../../../Data/mp3/chappie_chicken.mp3");

    head_ctrl.SetLookDirection( EMOHead::LOOK_LEFT );
    sleep( 1 );
    head_ctrl.SetLookDirection( EMOHead::LOOK_RIGHT );
    sleep( 1 );
    head_ctrl.SetLookDirection( EMOHead::LOOK_FORWARD );
    sleep( 1 );

/* ANTENNA POSITIONS */

    head_ctrl.SetSpeeds( 64, 64, 64 );
    head_ctrl.SetPositions( 255, 255, 255 );

LinuxActionScript::PlayMP3("../../../Data/mp3/chappie_whats_that.mp3");

    sleep( 1 );
    head_ctrl.SetSpeeds( 8, 8, 8 );
    head_ctrl.SetPositions( 0, 0, 0 );
    sleep( 1 );
    head_ctrl.SetSpeeds( 32, 32, 32 );
    head_ctrl.SetPositions( 128, 128, 128 );
    sleep( 1 );

    head_ctrl.SetSpeeds( 8, 8, 8 );
    head_ctrl.SetPositions( 255, 0, 255 );
    sleep( 1 );
    head_ctrl.SetSpeeds( 64, 64, 64 );
    head_ctrl.SetPositions( 0, 255, 0 );
    sleep( 1 );
    head_ctrl.SetSpeeds( 32, 32, 32 );
    head_ctrl.SetPositions( 128, 128, 128 );
    sleep( 1 );
    head_ctrl.SetSpeeds( 4, 4, 4 );
    head_ctrl.SetPositions( 0, 0, 0 );

LinuxActionScript::PlayMP3("../../../Data/mp3/chappie_no_he_can_talk_hes_smart.mp3");

/* EYE DOT PATTERN */

    for ( int i = 0; i < 0xff; ++i )
    {
        head_ctrl.SetEyeDotsRight( i );
        head_ctrl.SetEyeDotsLeft( i );
        usleep( 10000 );
    }

    head_ctrl.SetEyeDots( EMOHead::DOTS_UP );
    sleep( 1 );

    head_ctrl.SetEyeDots( EMOHead::DOTS_DOWN );
    sleep( 1 );

LinuxActionScript::PlayMP3("../../../Data/mp3/chappie_i_cant_do_a_heist.mp3");

    head_ctrl.SetEyeDotsRight( EMOHead::DOTS_WORRIED_RIGHT );
    head_ctrl.SetEyeDotsLeft( EMOHead::DOTS_WORRIED_LEFT );
    sleep( 3 );

    head_ctrl.SetEyeDotsRight( EMOHead::DOTS_SAD_RIGHT );
    head_ctrl.SetEyeDotsLeft( EMOHead::DOTS_SAD_LEFT );
    sleep( 1 );

    head_ctrl.SetEyeDots( EMOHead::DOTS_STAR );
    sleep( 1 );

    head_ctrl.SetEyeDots( EMOHead::DOTS_PLUS );
    sleep( 1 );

LinuxActionScript::PlayMP3("../../../Data/mp3/chappie_clever_little_robot.mp3");

    //ASCII art style spinner on high res OLED displays =P
    for ( int i=0; i < 5; ++i )
    {
        head_ctrl.SetEyeDots( EMOHead::DOTS_BACKSLASH );
        usleep( 100000 );

        head_ctrl.SetEyeDots( EMOHead::DOTS_BAR );
        usleep( 100000 );

        head_ctrl.SetEyeDots( EMOHead::DOTS_FWDSLASH );
        usleep( 100000 );

        head_ctrl.SetEyeDots( EMOHead::DOTS_MINUS );
        usleep( 100000 );
    }
    
    head_ctrl.SetEyeDots( EMOHead::DOTS_SHIP );
    sleep( 1 );

    head_ctrl.SetEyeDots( EMOHead::DOTS_FULL );

    //head_ctrl.SetMode( EMOHead::DEMO );

    exit(0);
}
