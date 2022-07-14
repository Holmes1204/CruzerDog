#include <string>
#include <serial/serial.h>
#include <ros/ros.h>
#include <wtr_serial_msg/em_fb_raw.h>
#include <wtr_serial_msg/em_ev.h>
#include <iostream>
#include <sys/time.h>
#include <cstring>

// 4*(2*11+1) = 92 bytes
struct uart_command_t
{
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

// 4*(1+2*7)= 60 bytes, only for a leg
struct uart_data_t
{
    float pos_fb_mid[2];
    float pos_fb_down[2];
    float vel_fb_mid[2];
    float vel_fb_down[2];
    float trq_fb_mid[2];
    float trq_fb_down[2];
    int state[2];
    int checksum;
};

uint32_t xor_checksum(uint32_t *data, size_t len)
{
    uint32_t t = 0;
    for (int i = 0; i < len; i++)
        t = t ^ data[i];
    return t;
}

class serial_node
{
private:
    /* data */
    ros::Subscriber cmd_sub;
    ros::Publisher em_fb_pub;
    //数据
    wtr_serial_msg::em_fb_raw em_fb_data;
    uart_data_t s1_uart_data, s2_uart_data;
    uart_command_t s1_uart_cmd, s2_uart_cmd;
    //共用一个地址还是分开
    uint8_t *send1, *send2;
    uint8_t *rcv1, *rcv2;
    //发送用
    serial::Serial my_serial1, my_serial2;
    const uint8_t DOF = 8;

public:
    ros::NodeHandle nh;
    void callback(const wtr_serial_msg::em_ev::ConstPtr &message);
    void redecode();
    void decode(const wtr_serial_msg::em_ev::ConstPtr &msg);
    void send();
    void recopy_(float *l_mid, float *l_down, float *f_mid, float *f_down, boost::array<float, 8UL> &ref);
    template <typename type>
    void copy_(type *l_mid, type *l_down, type *f_mid, type *f_down, const boost::array<type, 8UL> &ref);
    serial_node(uint32_t baud);
    ~serial_node();
};

void print_cmd(const uart_command_t *cmd)
{
    printf("____________\n");
    printf("%.4f %.4f\n", cmd->pos_des_mid[0], cmd->pos_des_mid[1]);
    printf("%.4f %.4f\n", cmd->vel_des_mid[0], cmd->vel_des_mid[1]);
    printf("%.4f %.4f\n", cmd->trq_ff_mid[0], cmd->trq_ff_mid[1]);
    printf("%.4f %.4f\n", cmd->kp_mid[0], cmd->kp_mid[1]);
    printf("%.4f %.4f\n", cmd->kd_mid[0], cmd->kd_mid[1]);

    printf("%.4f %.4f\n", cmd->pos_des_down[0], cmd->pos_des_down[1]);
    printf("%.4f %.4f\n", cmd->vel_des_down[0], cmd->vel_des_down[1]);
    printf("%.4f %.4f\n", cmd->trq_ff_down[0], cmd->trq_ff_down[1]);
    printf("%.4f %.4f\n", cmd->kp_down[0], cmd->kp_down[1]);
    printf("%.4f %.4f\n", cmd->kd_down[0], cmd->kd_down[1]);

    printf("%4d %4d\n", cmd->state[0], cmd->state[1]);
    printf("-----------\n");
}
void print_data(const uart_data_t *cmd)
{
    printf("____________\n");
    printf("%.4f %.4f\n", cmd->pos_fb_mid[0], cmd->pos_fb_mid[1]);
    printf("%.4f %.4f\n", cmd->vel_fb_mid[0], cmd->vel_fb_mid[1]);
    printf("%.4f %.4f\n", cmd->trq_fb_mid[0], cmd->trq_fb_mid[1]);

    printf("%.4f %.4f\n", cmd->pos_fb_down[0], cmd->pos_fb_down[1]);
    printf("%.4f %.4f\n", cmd->vel_fb_down[0], cmd->vel_fb_down[1]);
    printf("%.4f %.4f\n", cmd->trq_fb_down[0], cmd->trq_fb_down[1]);

    printf("%4d %4d\n", cmd->state[0], cmd->state[1]);
    printf("-----------\n");
}

void serial_node::decode(const wtr_serial_msg::em_ev::ConstPtr &msg)
{
    //将msg解码到两个数据中
    copy_<float>(s1_uart_cmd.pos_des_mid, s1_uart_cmd.pos_des_down, s2_uart_cmd.pos_des_mid, s2_uart_cmd.pos_des_down, msg->em_ev_pos);
    copy_<float>(s1_uart_cmd.vel_des_mid, s1_uart_cmd.vel_des_down, s2_uart_cmd.vel_des_mid, s2_uart_cmd.vel_des_down, msg->em_ev_vel);
    copy_<float>(s1_uart_cmd.trq_ff_mid, s1_uart_cmd.trq_ff_down, s2_uart_cmd.trq_ff_mid, s2_uart_cmd.trq_ff_down, msg->em_ev_trq);
    copy_<float>(s1_uart_cmd.kp_mid, s1_uart_cmd.kp_down, s2_uart_cmd.kp_mid, s2_uart_cmd.kp_down, msg->em_ev_kp);
    copy_<float>(s1_uart_cmd.kd_mid, s1_uart_cmd.kd_down, s2_uart_cmd.kd_mid, s2_uart_cmd.kd_down, msg->em_ev_kd);
    for (int i = 0; i < 2; i++)
    {
        s1_uart_cmd.state[i] = msg->em_state[i];
        s2_uart_cmd.state[i] = msg->em_state[i];
    }
    // xor 计算checksum
    s1_uart_cmd.checksum = xor_checksum((uint32_t *)&s1_uart_cmd, 22);
    s2_uart_cmd.checksum = xor_checksum((uint32_t *)&s2_uart_cmd, 22);
    // print_cmd(&s1_uart_cmd);
    // print_cmd(&s2_uart_cmd);
}

void serial_node::redecode()
{
    if (s1_uart_data.checksum != xor_checksum((uint32_t *)&s1_uart_data, 14) || s2_uart_data.checksum != xor_checksum((uint32_t *)&s2_uart_data, 14))
    {
        // printf("recode function wrong!\n");
        return;
    }
    // printf("recode function true!\n");
    //将两个数据合成一个数据
    recopy_(s1_uart_data.pos_fb_mid, s1_uart_data.pos_fb_down, s2_uart_data.pos_fb_mid, s2_uart_data.pos_fb_down, em_fb_data.em_pos_fb_raw);
    recopy_(s1_uart_data.vel_fb_mid, s1_uart_data.vel_fb_down, s2_uart_data.vel_fb_mid, s2_uart_data.vel_fb_down, em_fb_data.em_vel_fb_raw);
    recopy_(s1_uart_data.trq_fb_mid, s1_uart_data.trq_fb_down, s2_uart_data.trq_fb_mid, s2_uart_data.trq_fb_down, em_fb_data.em_trq_fb_raw);
    for (int i = 0; i < 2; i++)
    {
        em_fb_data.em_state[i] = (s1_uart_cmd.state[i] | s2_uart_cmd.state[i]);
    }
    // print_data(&s1_uart_data);
    // print_data(&s2_uart_data);
}

template <typename type>
void serial_node::copy_(type *s1_mid, type *s1_down, type *s2_mid, type *s2_down, const boost::array<type, 8UL> &ref)
{
    s1_mid[0] = ref[0];
    s1_down[0] = ref[1];
    s1_mid[1] = ref[2];
    s1_down[1] = ref[3];
    s2_mid[0] = ref[4];
    s2_down[0] = ref[5];
    s2_mid[1] = ref[6];
    s2_down[1] = ref[7];
}

void serial_node::recopy_(float *s1_mid, float *s1_down, float *s2_mid, float *s2_down, boost::array<float, 8UL> &ref)
{
    ref[0] = s1_mid[0];
    ref[1] = s1_down[0];
    ref[2] = s1_mid[1];
    ref[3] = s1_down[1];
    ref[4] = s2_mid[0];
    ref[5] = s2_down[0];
    ref[6] = s2_mid[1];
    ref[7] = s2_down[1];
}

int clk = 0, outloop = 0;
timeval t1 = {0, 0}, t2 = {0, 0}, t3 = {0, 0}, t4 = {0, 0};
double sum1 = 0, sum2 = 0;
uint32_t tim1, tim2;

// send 函数抽象位数据在上下位机之间通讯的函数，更换通讯方法只需要更改此函数
void serial_node::send()
{
    //通过串口发送数据
    // int count = 0;
    // do{
    // if(count>0){
    // printf("%5d____1_______\n",count);
    //}
    gettimeofday(&t1, NULL);
    my_serial1.write(send1, 92);
    my_serial2.write(send2, 92);
    gettimeofday(&t2, NULL);
    my_serial1.read(rcv1, 60);
    my_serial2.read(rcv2, 60);
    gettimeofday(&t3, NULL);
    tim1 = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);
    sum1 = sum1 * clk / (clk + 1.0) + tim1 / (clk + 1.0);
    tim2 = (t3.tv_sec - t2.tv_sec) * 1000000 + (t3.tv_usec - t2.tv_usec);
    sum2 = sum2 * clk / (clk + 1.0) + tim2 / (clk + 1.0);
    printf("%5d____1_______\n", outloop);
    printf("%5d %5d %5lf\n", clk, tim1, sum1);
    printf("%5d %5d %5lf\n", 0, tim2, sum2);
    clk++;
    // count++;
    // printf("send1 is on going %d!\n",count);
    //}while(s1_uart_cmd.checksum!=s1_uart_data.ack&&count<10);
    // count = 0;
    // do{
    // if(count>0){
    //     printf("%5d____2_______\n",count);
    // }

    // count++;
    // printf("send2 is on going %d!\n",count);
    //}while(s2_uart_cmd.checksum!=s2_uart_data.ack&&count<10);
    // outloop++;
    // print_data(&s1_uart_data);
}

void serial_node::callback(const wtr_serial_msg::em_ev::ConstPtr &message)
{
    decode(message);
    send();
    //上位机topic更新
    redecode();
    em_fb_pub.publish(em_fb_data);
}

serial_node::serial_node(uint32_t baud) : my_serial1(std::string("/dev/ttyACM0"), baud, serial::Timeout::simpleTimeout(10)),
                                          my_serial2(std::string("/dev/ttyACM1"), baud, serial::Timeout::simpleTimeout(10))
{
    cmd_sub = nh.subscribe("/em_ev", 1, &serial_node::callback, this);
    em_fb_pub = nh.advertise<wtr_serial_msg::em_fb_raw>("/em_fb_raw", 1);
    //优化1
    rcv1 = (uint8_t *)&s1_uart_data;
    rcv2 = (uint8_t *)&s2_uart_data;
    send1 = (uint8_t *)&s1_uart_cmd;
    send2 = (uint8_t *)&s2_uart_cmd;
    //初始化
    memset(&s1_uart_cmd, 0, sizeof(uart_command_t));
    memset(&s2_uart_cmd, 0, sizeof(uart_command_t));
    memset(&s1_uart_data, 0, sizeof(uart_data_t));
    memset(&s2_uart_data, 0, sizeof(uart_data_t));
};

serial_node::~serial_node()
{
}

int main(int argc, char **argv)
{
    // ROS initilize
    ros::init(argc, argv, "serial_node");
    //由于使用了USB协议，波特率和timeout的设置没有影响
    serial_node this_node(115200);
    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
