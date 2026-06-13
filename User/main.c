#include "stm32f10x.h"
#include <stdio.h>

#define UART1_BAUDRATE 115200  // 提高波特率到115200
#define START_BYTE 0xAA        // 帧头
#define END_BYTE 0x55          // 帧尾
#define ACK_SUCCESS 0xAC       // 成功应答
#define ACK_FAILURE 0xAD       // 失败应答

// 舵机角度初始化
volatile int servo1_angle = 90;
volatile int servo2_angle = 90;
volatile int servo3_angle = 62;
volatile int servo4_angle = 90;
volatile int servo5_angle = 90;
volatile int servo6_angle = 90;

// 通信协议结构
typedef struct {
    uint8_t start_byte;   // 帧头 0xAA
    uint8_t servo_num;    // 舵机号 1-6
    uint8_t angle_high;   // 角度高字节
    uint8_t angle_low;    // 角度低字节
    uint8_t checksum;     // 校验和
    uint8_t end_byte;     // 帧尾 0x55
} ServoCommand;

// Function prototypes
void TIM2_PWM_Init(void);
void TIM3_PWM_Init(void);
void set_angle_servo(int servo_num, int angle);
void UART1_Init(void);
uint8_t UART1_ReceiveByte(void);
void UART1_SendByte(uint8_t data);
uint8_t calculate_checksum(uint8_t servo_num, uint8_t angle_high, uint8_t angle_low);
int receive_command(ServoCommand* cmd);

// 初始化TIM2用于舵机1-4的PWM输出
void TIM2_PWM_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 使能GPIOA和TIM2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 配置PA0, PA1, PA2, PA3为复用推挽模式输出PWM信号
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置TIM2的时基单元
    TIM_TimeBaseStructure.TIM_Period = 19999;  // PWM周期20ms
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;  // 预分频，系统时钟1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;         
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 配置PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    // 初始化通道1到通道4
    TIM_OCInitStructure.TIM_Pulse = 1500;  // 默认占空比（1500微秒）
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    // 使能预装载寄存器
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);  // 自动重装载寄存器
    TIM_Cmd(TIM2, ENABLE);  // 使能TIM2
}

// 初始化TIM3用于舵机5-6的PWM输出（PB0->CH3, PB1->CH4）
void TIM3_PWM_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 使能GPIOB和TIM3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // 配置PB0, PB1为复用推挽模式输出PWM信号
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 配置TIM3的时基单元
    TIM_TimeBaseStructure.TIM_Period = 19999;  // PWM周期20ms
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;  // 预分频，系统时钟1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;         
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // 配置PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    // 初始化通道3到通道4
    TIM_OCInitStructure.TIM_Pulse = 1500;  // 默认占空比（1500微秒）
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);  // 通道3 - PB0
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);  // 通道4 - PB1

    // 使能预装载寄存器
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);  // 自动重装载寄存器
    TIM_Cmd(TIM3, ENABLE);  // 使能TIM3
}

// 设置舵机角度
void set_angle_servo(int servo_num, int angle) {
    // 限制角度范围
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // 将角度转换为脉宽
    int pulse = (angle * 11) + 500;

    // 根据舵机号设置对应的通道
    switch (servo_num) {
        case 1:
            TIM_SetCompare1(TIM2, pulse);
            servo1_angle = angle;
            break;
        case 2:
            TIM_SetCompare2(TIM2, pulse);
            servo2_angle = angle;
            break;
        case 3:
            TIM_SetCompare3(TIM2, pulse);
            servo3_angle = angle;
            break;
        case 4:
            TIM_SetCompare4(TIM2, pulse);
            servo4_angle = angle;
            break;
        case 5:
            TIM_SetCompare3(TIM3, pulse);  // 通道3 - PB0
            servo5_angle = angle;
            break;
        case 6:
            TIM_SetCompare4(TIM3, pulse);  // 通道4 - PB1
            servo6_angle = angle;
            break;
        default:
            // 无效的舵机号
            break;
    }
}

// 初始化UART1用于接收控制指令
void UART1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // 使能GPIOA和USART1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    // 配置USART1 TX (PA9)为复用推挽模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置USART1 RX (PA10)为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置USART1的基本参数
    USART_InitStructure.USART_BaudRate = UART1_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);

    // 使能USART1
    USART_Cmd(USART1, ENABLE);
}

// 接收单个字节
uint8_t UART1_ReceiveByte(void) {
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    return USART_ReceiveData(USART1);
}

// 发送单个字节
void UART1_SendByte(uint8_t data) {
    USART_SendData(USART1, data);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

// 计算校验和
uint8_t calculate_checksum(uint8_t servo_num, uint8_t angle_high, uint8_t angle_low) {
    return (START_BYTE + servo_num + angle_high + angle_low) & 0xFF;
}

// 接收命令
int receive_command(ServoCommand* cmd) {
    uint8_t byte;
    int timeout_counter = 0;
    
    // 等待帧头
    while (1) {
        byte = UART1_ReceiveByte();
        if (byte == START_BYTE) {
            cmd->start_byte = byte;
            break;
        }
        
        // 简单超时处理
        timeout_counter++;
        if (timeout_counter > 1000) {
            return 0;  // 超时，接收失败
        }
    }
    
    // 接收舵机号
    cmd->servo_num = UART1_ReceiveByte();
    
    // 接收角度高字节
    cmd->angle_high = UART1_ReceiveByte();
    
    // 接收角度低字节
    cmd->angle_low = UART1_ReceiveByte();
    
    // 接收校验和
    cmd->checksum = UART1_ReceiveByte();
    
    // 接收帧尾
    cmd->end_byte = UART1_ReceiveByte();
    
    // 验证帧尾
    if (cmd->end_byte != END_BYTE) {
        return 0;  // 帧尾错误
    }
    
    // 验证校验和
    uint8_t calculated_checksum = calculate_checksum(cmd->servo_num, cmd->angle_high, cmd->angle_low);
    if (calculated_checksum != cmd->checksum) {
        return 0;  // 校验和错误
    }
    
    return 1;  // 接收成功
}

// 主函数
int main(void) {
    SystemInit();  // 初始化系统时钟
    ServoCommand cmd;
    int success;

    // 初始化所有外设
    TIM2_PWM_Init();
    TIM3_PWM_Init();
    UART1_Init();

    // 初始化舵机默认角度
    set_angle_servo(1, servo1_angle);
    set_angle_servo(2, servo2_angle);
    set_angle_servo(3, servo3_angle);
    set_angle_servo(4, servo4_angle);
    set_angle_servo(5, servo5_angle);
    set_angle_servo(6, servo6_angle);

    while (1) {
        // 接收命令
        success = receive_command(&cmd);
        
        if (success) {
            // 解析舵机号和角度
            uint8_t servo_num = cmd.servo_num;
            uint16_t angle = (cmd.angle_high << 8) | cmd.angle_low;
            
            // 验证范围并设置舵机角度
            if ((servo_num >= 1 && servo_num <= 6) && (angle <= 180)) {
                set_angle_servo(servo_num, angle);
                UART1_SendByte(ACK_SUCCESS);  // 发送成功应答
            } else {
                UART1_SendByte(ACK_FAILURE);  // 发送失败应答
            }
        } else {
            UART1_SendByte(ACK_FAILURE);  // 发送失败应答
        }
    }
}
