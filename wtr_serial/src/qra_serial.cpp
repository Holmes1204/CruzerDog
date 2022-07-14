/**
 * @file node.cpp
 * @author holmes (hdasddfgg@126.com)
 * @brief the serial communication between stm32 and ros
 * @version 0.1
 * @date 2021-05-09
 *
 * @copyright WTRobo Copyright (c) 2021
 *
 */
#include <qra_serial.h>
#include <iostream>

serial::Serial sp[2];
const uint8_t HEADER[2] = {0x55, 0xaa};

int Serial_Init(int num, const std::string port, const uint32_t baud_rate) //串口初始化
{
    //创建timeout
    static serial::Timeout time_out = serial::Timeout::simpleTimeout(100);
    // static do not release time_out until the code finish
    //设置要打开的串口名称
    sp[num].setPort(port);
    //设置串口通信的波特率
    sp[num].setBaudrate(baud_rate);
    //串口设置timeout
    sp[num].setTimeout(time_out);
    try
    {
        //打开串口
        sp[num].open();
        //判断串口是否打开成功
        if (sp[num].isOpen())
        {
            // ROS_INFO_STREAM(port<<" is open"<<std::endl);
            std::cout<< "port: " << port << " is open" << std::endl;
            return 1;
        }
        else
        {
            return 0;
        }
    }
    catch (serial::IOException &e) //捕捉输入输出异常
    {
        // ROS_ERROR_STREAM("Unable to open port:"<<port<<std::endl);
        std::cout<< "U nable to open port: " << port << std::endl;
        return 0;
    }
}

// error handle

bool Serial_Transmit(int num, uint8_t *buf, int len)
{
    if (sp[num].write(buf, len) == len)
    {
        return true;
    }
    return false;
}

int Serial_Receive(int serial_num, uint8_t *buf, uint8_t len)
{
    try
    {
        uint8_t num = len ? len : sp[serial_num].available();
        if (num && sp[serial_num].available() >= num)
        {
            return sp[serial_num].read(buf, num);
        }
        return 0;
    }
    catch (serial::IOException &e)
    {
        // ROS_INFO("read_until error");
        return 0;
    }
}

uint8_t crc_check(uint8_t *ptr, int len)
{
    uint8_t crc = 0x00;
    while (len-- > 0)
    {
        crc = crc_table[crc ^ *ptr++];
    }
    return (crc);
}

uint16_t float64_to_uint(double x, double x_min, double x_max, int bits)
{
    /// Converts a float to an uint16, given range and numberW ///
    double span = x_max - x_min;
    double offset = x_min;
    return (uint16_t)((x - offset) * ((double)((1 << bits) - 1)) / span);
}

double uint_to_float64(int x, double x_min, double x_max, int bits)
{
    double span = x_max - x_min;
    double offset = x_min;
    return (double)(((x * span) / (double)((1 << bits) - 1)) + offset);
}
