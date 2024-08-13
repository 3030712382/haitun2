/*========================= Gamepad Control ================================*/
 /* Copyright (C) 2021 - ~, SCUT-RobotLab Development Team
  * @file    Logitech_controller.cpp
  * @author  lindongdong
  * @brief   罗技遥控器的驱动库，库目前支持罗技遥控器型号：F710、F310。
  * @note 
  *         - 注意按键值所对应的意义。
  *         - 当手柄 Mode为黄灯时，左边上下左右按键与左边遥感值互换，按键按下为对应遥感边界值（+-1）
  *           遥感推至极限为对应按键值。
  *         - 一般情况下使用非黄灯模式。
  * @method 
  *         - 1. 手柄驱动路径     ： char path[] = "/dev/input/js0";
  *         - 2. 定义一个遥控器类  : Logitech Logitech(path);
  *         - 3. 遥控器类初始化    : Logitech.init();
  *         - 4. 线程中进行数据接受 : Logitech.listen_input();
  * @warning 
  *         - At least C++11 is required.	
  *         - Only linux platform is supported for now.
*/ 												 
/*========================= Gamepad Control ================================*/

/* Includes ----------------------------------------------------------------*/
#include "Logitech_controller.h"

using namespace std;

Logitech::Logitech(char* device)
{
    dev = device;
    memset(buf, 0, sizeof buf);
}

int Logitech::init()
{
    fd = open(dev, O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "Cannot open %s: %s.\n", dev, strerror(errno));
        return EXIT_FAILURE;
    }
    /*Key Status*/

    /* 0 is released */
    /* 1 is press    */
    Keystate_map[JSKEY_A] =0;
    Keystate_map[JSKEY_B] =0;
    Keystate_map[JSKEY_X] =0;
    Keystate_map[JSKEY_Y] =0;

    /* 0 is released */
    /* 1 is press    */
    Keystate_map[JSKEY_LB] =0;
    Keystate_map[JSKEY_RB] =0;
   
    /* 0 is released */
    /* 1 is press    */
    Keystate_map[JSKEY_BACK] =0;
    Keystate_map[JSKEY_START] =0;
    Keystate_map[JSKEY_HOME] =0;
    
    /*  0 is released */
    /* -1 is the left or up button is pressed */
    /*  1 is the right or down button is pressed*/
    Keystate_map[JSKEY_CROSS_X] =0;
    Keystate_map[JSKEY_CROSS_Y] =0;

    /* the result is the value of the key(0~99)*/
    Keystate_map[JSKEY_LT] =0;
    Keystate_map[JSKEY_RT] =0;

    /* the result is the value of the key(-100~100)*/
    Keystate_map[JSKEY_LEFTSTICK_X] =0;
    Keystate_map[JSKEY_LEFTSTICK_Y] =0;
    Keystate_map[JSKEY_RIGHTSTICK_X] =0;
    Keystate_map[JSKEY_RIGHTSTICK_Y] =0;

    return 0;
}

void Logitech::listen_input()
{
    while (1) {
        memset(buf, 0, sizeof buf);
        n = read(fd, &buf, sizeof buf);
        n = n / sizeof(int);
        if (n == (ssize_t)-1) {
            if (errno == EINTR)
                continue;
            else
                break;
        }

        unsigned short btn = buf[1] >> 16;
        short val = (short)(buf[1] & 0xffff);

         /*Test for button ID*/
         //cout<<"0x"<<hex<<btn<<endl;

        if (btn == JSKEY_LT || btn == JSKEY_RT)
        {
            unsigned short prs_val = val + 32768;
            val = (unsigned short) (((long)prs_val)*100/65536);
            Keystate_map[btn]= val;
        }
        else if (btn == JSKEY_LEFTSTICK_X || btn == JSKEY_LEFTSTICK_Y ||
                 btn == JSKEY_RIGHTSTICK_X || btn == JSKEY_RIGHTSTICK_Y)
        { //左右摇杆模拟量 横x 竖y
            /* y-axis reverse */
            if(btn==JSKEY_LEFTSTICK_Y||btn == JSKEY_RIGHTSTICK_Y)
            {val=(-1)*val;}
            val = val*100/32767;
            Keystate_map[btn]= val;
        }
        else
        {
            switch (val)
            {
            case JSKEY_PRESS:
                Keystate_map[btn]=1;
                break;
            case JSKEY_RELEASE:
                Keystate_map[btn]=0;
                break;
            case JSKEY_CROSS_LOW_VALUE:
                Keystate_map[btn]=-1;
                break;
            case JSKEY_CROSS_HIGH_VALUE:
                Keystate_map[btn]=1;
                break;
            default:
                break;
            }
            /* y-axis reverse */
            if(btn==JSKEY_CROSS_Y)
            {Keystate_map[btn]=(-1)*Keystate_map[btn];}
        }
        print_key_state();
    }
}

void Logitech::print_key_state()
{
   cout<<endl;
   cout<<"JSKEY_A = "<<Keystate_map[JSKEY_A]<<endl;
   cout<<"JSKEY_B = "<<Keystate_map[JSKEY_B]<<endl;
   cout<<"JSKEY_X = "<<Keystate_map[JSKEY_X]<<endl;
   cout<<"JSKEY_Y = "<<Keystate_map[JSKEY_Y]<<endl;

   cout<<"JSKEY_LB = "<<Keystate_map[JSKEY_LB]<<endl;
   cout<<"JSKEY_RB = "<<Keystate_map[JSKEY_RB]<<endl;
   cout<<"JSKEY_BACK = "<<Keystate_map[JSKEY_BACK]<<endl;
   cout<<"JSKEY_START = "<<Keystate_map[JSKEY_START]<<endl;
   cout<<"JSKEY_HOME = "<<Keystate_map[JSKEY_HOME]<<endl;

   cout<<"JSKEY_LT = "<<Keystate_map[JSKEY_LT]<<endl;
   cout<<"JSKEY_RT = "<<Keystate_map[JSKEY_RT]<<endl;

   cout<<"JSKEY_CROSS_X = "<<Keystate_map[JSKEY_CROSS_X]<<endl;
   cout<<"JSKEY_CROSS_Y = "<<Keystate_map[JSKEY_CROSS_Y]<<endl;

   cout<<"JSKEY_LEFTSTICK_X  = "<<Keystate_map[JSKEY_LEFTSTICK_X] <<"     JSKEY_LEFTSTICK_Y   = "<<Keystate_map[JSKEY_LEFTSTICK_Y]<<endl;
   cout<<"JSKEY_RIGHTSTICK_X = "<<Keystate_map[JSKEY_RIGHTSTICK_X]<<"     JSKEY_RIGHTSTICK_Y = "<<Keystate_map[JSKEY_RIGHTSTICK_Y]<<endl;
}

