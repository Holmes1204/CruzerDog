/**
 * @file main.cpp
 * @author holmes (hdasddfgg@126.com)
 * @brief the serial communication between stm32 and ros
 * @version 0.1
 * @date 2021-05-09
 *
 * @copyright WTRobo Copyright (c) 2021
 * @todo 1. debug the receive function 2. the receive & transmit data structure
 */
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <qra_serial.h>
#include <thread>
#include <list>
#include <vector>
#include <wtr_serial/leg.h>
#include <wtr_serial/em_fb_raw.h>
#include <wtr_serial/em_ev.h>
#include <string.h>
#include <sys/time.h>

#define P_MIN -95.5f // Radians
#define P_MAX 95.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
#define KP_MIN 0.0f // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f // N-m/rad/s
#define KD_MAX 5.0f                                                                      
#define P0 32767
#define V0 2047
#define T0 2047
#define DoF 8

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

#define D_LEN 22
#define P_LEN 11

int stop = 0;
int main_run;
//
wtr_serial::em_fb_raw em_fb_data;
//
wtr_serial::em_ev em_ev_data;
uint16_t em_ev_mcu[DoF][5];
uint16_t em_ev_state;
//
enum Data_Order
{
    POS = 0,
    VEL,
    TRQ,
    KP,
    KD,
};


//4*(2*11+1) = 92 bytes
struct uart_command_t{
    float pos_des_mid[2];
    float pos_des_down[2];
    float vel_des_mid[2];
    float vel_des_down[2];
    float trq_ff_mid[2];
    float trq_ff_down[2];
    float kp_mid[2];
    float kp_down[2];
    float kd_mid[2];
    float kd_down[2];
    int state[2];
    int checksum;
};

//4*(1+2*7)= 60 bytes, only for a leg
struct uart_data_t{
    float pos_fb_mid[2];
    float pos_fb_down[2];
    float vel_fb_mid[2];
    float vel_fb_down[2];
    float trq_fb_mid[2];
    float trq_fb_down[2];
    int state[2];
    int checksum;
    void print();
};


void uart_data_t::print(){
    static unsigned long count = 0;
    //printf("--%ld--\n",count++);
    //printf("%X %X\n",*(int*)(this->pos_fb_mid),*(int*)(this->pos_fb_mid+1));
    //printf("%X %X\n",*(int*)(this->vel_fb_mid),*(int*)(this->vel_fb_mid+1));
    //printf("%X %X\n",*(int*)(this->trq_fb_mid),*(int*)(this->trq_fb_mid+1));
    printf("----\n");
    printf("%.4f %.4f\n",*(this->pos_fb_mid),*(this->pos_fb_mid+1));
    printf("%.4f %.4f\n",*(this->vel_fb_mid),*(this->vel_fb_mid+1));
    printf("%.4f %.4f\n",*(this->trq_fb_mid),*(this->trq_fb_mid+1));
    printf("%X\n",this->checksum);
}



uint8_t rcv_data[D_LEN];

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
/// Converts a float to an unsigned int, given range and number of bits ///
float uint_to_float(uint16_t x, float x_min, float x_max, int bit)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (float)(((x * span) / (float)((1 << bit) - 1)) + offset);
}


//往下位机发信息
int send_msg(uint8_t *id, uint16_t *pos, uint16_t *vel, uint16_t *torque, uint16_t *kp, uint16_t *kd, uint16_t *state)
{
    //填充装弹
    static uint8_t _serial_msg[14];
    _serial_msg[0] = 0xAA;
    _serial_msg[1] = 0x55;
    _serial_msg[2] = (*pos) >> 8;
    _serial_msg[3] = (*pos) & 0xFF;
    _serial_msg[4] = (*vel) & 0xFF;
    _serial_msg[5] = ((*vel) >> 4) & 0xF0 | (*torque) >> 8;
    _serial_msg[6] = (*torque) & 0xFF;
    _serial_msg[7] = (*kp) & 0xFF;
    _serial_msg[8] = ((*kp) >> 4) & 0xF0 | (*kd) >> 8;
    _serial_msg[9] = (*kd) & 0xFF;
    _serial_msg[10] = (*id);
    _serial_msg[11] = (*state);
    _serial_msg[12] = 0xf0;
    _serial_msg[13] = 0x0f;
    //发射
    Serial_Transmit(FONT, _serial_msg, 14);
    // if (leg_num < 2)
    //     Serial_Transmit(FONT, _serial_msg, 11);
    // else
    //     // Serial_Transmit(BACK,_serial_msg,8);
    return 1;
}

// ros 回调
void ctrl_cb(const wtr_serial::em_ev &msg)
{
    for (int i = 0; i < DoF; i++)
    {
        // receive msg
        em_ev_data.em_ev_pos.elems[i] = msg.em_ev_pos.elems[i];
        em_ev_data.em_ev_vel.elems[i] = msg.em_ev_vel.elems[i];
        em_ev_data.em_ev_trq.elems[i] = msg.em_ev_trq.elems[i];
        em_ev_data.em_ev_kp.elems[i] = msg.em_ev_kp.elems[i];
        em_ev_data.em_ev_kd.elems[i] = msg.em_ev_kd.elems[i];
        // em_ev_state = msg.em_ev_ste;
        // range params,significance
        LIMIT_MIN_MAX(em_ev_data.em_ev_pos.elems[i], P_MIN / 24, P_MAX / 24); //进一步限制坏值，除完后再转
        LIMIT_MIN_MAX(em_ev_data.em_ev_vel.elems[i], V_MIN, V_MAX);
        LIMIT_MIN_MAX(em_ev_data.em_ev_trq.elems[i], T_MIN, T_MAX);
        LIMIT_MIN_MAX(em_ev_data.em_ev_kp.elems[i], KP_MIN, KP_MAX);
        LIMIT_MIN_MAX(em_ev_data.em_ev_kd.elems[i], KD_MIN, KD_MAX);
        // float 转 uint16，inline 制作弹丸
        em_ev_mcu[i][POS] = float_to_uint(em_ev_data.em_ev_pos.elems[i], P_MIN, P_MAX, 16) - P0;
        em_ev_mcu[i][VEL] = float_to_uint(em_ev_data.em_ev_vel.elems[i], V_MIN, V_MAX, 12) - V0;
        em_ev_mcu[i][TRQ] = float_to_uint(em_ev_data.em_ev_trq.elems[i], T_MIN, T_MAX, 12) - T0;
        em_ev_mcu[i][KP] = float_to_uint(em_ev_data.em_ev_kp.elems[i], KP_MIN, KP_MAX, 12);
        em_ev_mcu[i][KD] = float_to_uint(em_ev_data.em_ev_kd.elems[i], KD_MIN, KD_MAX, 12);
        // debug
        std::cout << "id:" << i << "  pos:" << em_ev_mcu[i][POS] << "  vel:" << em_ev_mcu[i][VEL]
                  << "  trq:" << em_ev_mcu[i][TRQ] << "  kp:" << em_ev_mcu[i][KP] << "  kd:" << em_ev_mcu[i][KD] << " state:" << em_ev_state << std::endl;
        // std::cout << "id:" << i << "  pos:" << em_ev_data.em_pos_ev.elems[i]
        //     << "  vel:" << em_ev_data.em_vel_ev.elems[i]
        //     << "  trq:" << em_ev_data.em_trq_ev.elems[i]<< std::endl;
    }
    return;
}

//上位机接受信息
void msg_decode(uint8_t id, uint8_t *p_data)
{
    // uint16_t 转 float
    uint16_t pos = p_data[0] << 8 | p_data[1];
    uint16_t vel = (p_data[3] & 0xF0) << 4 | p_data[2];
    uint16_t torque = (p_data[3] & 0x0F) << 8 | p_data[4];

    em_fb_data.em_pos_fb_raw.elems[id] = uint_to_float(pos, P_MIN, P_MAX, 16);
    em_fb_data.em_vel_fb_raw.elems[id] = uint_to_float(vel, V_MIN, V_MAX, 12);
    em_fb_data.em_trq_fb_raw.elems[id] = uint_to_float(torque, T_MIN, T_MAX, 12);
    // em_fb_data.em_ste_fb_raw = p_data[6];
    // printf("%d\n",id);
    //  std::cout << "id:"<<int(id)<< "  pos:" << pos<< "  vel:" << vel<< "  trq:" << torque<< std::endl;
    //  std::cout << "id:" << i << "  pos:" << em_ev_data.em_pos_ev.elems[i]
    //      << "  vel:" << em_ev_data.em_vel_ev.elems[i]
    //      << "  trq:" << em_ev_data.em_trq_ev.elems[i]<< std::endl;
}

//接受下面上来的信息
void STM32_Recieve_Handle(int serial_num)
{
    std::cout << serial_num << " is running" << std::endl;
    // int decode_count = 0;
    uint8_t p = 0;
    int decode_count = 0;
    while (main_run)
    {
        for (uint8_t i = 0; i < DoF; i++)
        {
            if (i < 4)
            {
                send_msg(&i, &em_ev_mcu[i][POS], &em_ev_mcu[i][VEL], &em_ev_mcu[i][TRQ], &em_ev_mcu[i][KP], &em_ev_mcu[i][KD], &em_ev_state);
            }
            else
            {

            }
        }
        if (Serial_Receive(serial_num, rcv_data, D_LEN) == D_LEN)
        {
            // printf("ok1!\n");
            for (uint8_t i = 0; i < 12; i++)
            {
                if (rcv_data[i] == 0xAA)
                {
                    if (rcv_data[i + 1] == 0x55 && rcv_data[i + 9] == 0xDA && rcv_data[i + 10] == 0x5D)
                    {
                        // printf("ok3!%d\n",int(i));
                        // decode_count++;
                        // printf("decode:%d\n", decode_count);
                        // printf("%d\n", rcv_data[i]);
                        // printf("%d\n", rcv_data[i+ 1]);
                        // printf("%d\n", rcv_data[i+ 2]);
                        // printf("%d\n", rcv_data[i+ 3]);
                        // printf("%d\n", rcv_data[i+ 4]);
                        // printf("%d\n", rcv_data[i+ 5]);
                        // printf("%d\n", rcv_data[i+ 6]);
                        // printf("%d\n", rcv_data[i + 7]);
                        // printf("%d\n", rcv_data[i + 8]);
                        // printf("%d\n", rcv_data[i + 9]);
                        // printf("---------\n");
                        msg_decode(rcv_data[i + 7], &rcv_data[i + 2]);
                        break;
                    }
                }
            }
        }
    }
    printf("done!\n");
}

using namespace std;

void enumerate_ports()
{
    vector<serial::PortInfo> devices_found = serial::list_ports();

    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end())
    {
        serial::PortInfo device = *iter++;
        printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str());
    }
}

uint32_t xor_checksum(uint32_t* data, size_t len)
{
    uint32_t t = 0;
    for(int i = 0; i < len; i++)
        t = t ^ data[i];
    return t;
}
void delay_us(uint32_t delay){
    timeval t;
    gettimeofday(&t,NULL);
    uint32_t tickstart = t.tv_sec * 1000000 +t.tv_usec;
    uint32_t run_time =0;
    do{
        gettimeofday(&t,NULL);
        run_time=t.tv_sec * 1000000 +t.tv_usec;
    }
    while (run_time < tickstart+delay);
}


int main(int argc, char **argv) // argc是命令行总的参数个数 argv[]为保存命令行参数的字符串指针，其中第0个参数是程序的全名，以后的参数为命令行后面跟的用户输入的参数
{
    // enumerate_ports();
    string port0("/dev/ttyACM0");
    string port1("/dev/ttyACM1");
    ofstream file;
    uint32_t baud = 921600;
    serial::Serial my_serial1(port0, baud, serial::Timeout::simpleTimeout(5));
     serial::Serial my_serial2(port1, baud, serial::Timeout::simpleTimeout(100));
    // differ the serial port
    int clk = 0;
    timeval t1 = {0, 0}, t2 = {0, 0}, t3 = {0, 0}, t4 = {0, 0};
    double sum1 = 0,sum2 =0,sum3=0;
    size_t bytes_wrote;
    size_t bytes_read;
    //
    uart_command_t test;
    uart_data_t rcv;

    uint8_t uart_test_command_t[92];
    uint8_t tmp_rx[100];
    uint8_t tmp_tx[100];
    const int length = 100;
    uint8_t gest[length] = {0xae, 0xde};
    uint32_t cha1[length],cha2[length], tim1, tim2;
    uint8_t result[length];
    memset(&test,0,sizeof(test));
    memset(cha1, 0, sizeof(cha1));
    memset(cha2, 0, sizeof(cha2));
    
    for (int i = 0; i < 92; i++)
    {
        uart_test_command_t[i]=i;
    }
    //
    test.pos_des_mid[0]=0;
    test.vel_des_mid[0]=1;
    test.trq_ff_mid[0]=2;
    test.kp_mid[0]=3;
    test.kd_mid[0]=4;
    //
    test.pos_des_down[0]=5;
    test.vel_des_down[0]=6;
    test.trq_ff_down[0]=7;
    test.kp_down[0]=8;
    test.kd_down[0]=9;
    test.state[1]=10;
    //
    test.pos_des_mid[1]=11;
    test.vel_des_mid[1]=12;
    test.trq_ff_mid[1]=13;
    test.kp_mid[1]=14;
    test.kd_mid[1]=15;
    //
    test.pos_des_down[1]=16;
    test.vel_des_down[1]=17;
    test.trq_ff_down[1]=18;
    test.kp_down[1]=19;
    test.kd_down[1]=20;
    test.state[1]=21;
    //
    test.state[0]=14;
    test.checksum = xor_checksum((uint32_t*)&test,22);
    
    while (1)
    {
        //getchar();
        gettimeofday(&t1, NULL);
        //delay_us(250);
        bytes_wrote = my_serial1.write((uint8_t*)&test, 92);
        delay_us(25);
        bytes_wrote = my_serial2.write((uint8_t*)&test, 92);


        delay_us(500);
        gettimeofday(&t2, NULL);
        bytes_read = my_serial1.read(tmp_rx,60);
        delay_us(25);
        bytes_read = my_serial2.read(tmp_rx,60);
        //delay_us(250);
        //bytes_wrote = my_serial1.write((uint8_t*)&test, 92);
        //delay_us(250);
        //bytes_read = my_serial1.read(tmp_rx,63);
        //bytes_wrote = my_serial2.write((uint8_t*)&test,92);
        //bytes_read = my_serial1.read((uint8_t*)&tmp_rx,50);
        //bytes_read = my_serial2.read((uint8_t*)&rcv,60);
        gettimeofday(&t3, NULL);

        tim1 = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);//每次所需的时间
        sum1 = sum1*clk/(clk+1.0)+tim1/(clk+1.0);//均值
        sum3 = sum3*clk/(clk+1.0)+tim1*tim1/(clk+1.0);//平方均值
        sum2 = sqrt(sum3-sum1*sum1);
        tim2=(t3.tv_sec -t2.tv_sec)*1000000+(t3.tv_usec-t2.tv_usec);
        //sum2 = sum2*clk/(clk+1.0)+tim2/(clk+1.0);
        printf("-------\n");
        printf("%5d %5d %5lf\n",clk,tim1,sum1);
        printf("%5d %5d %5lf\n",int(bytes_read),tim2,sum2);    

        clk++;  
        //for (int i = 0; i < 200000; i++)
        //{
            //200thousans
        //}
        
        //rcv.print();
    }
    // for (int i = 0; i < 92; i++)
    // {
    //     printf("%d,%X\n",i,uart_test_command_t[i]);
    // }
    
    // file.open("/home/qardruped/Quadruped/Hdwr_Pack/src/wtr-serial/key4.txt",ios::trunc);
    // for (int i = 0; i < length; i++)
    // {
    //     file<<i<<","<<cha1[i]<<","<<cha2[i]<<"\n";
    // }
    // file.close();
    return 0;
}