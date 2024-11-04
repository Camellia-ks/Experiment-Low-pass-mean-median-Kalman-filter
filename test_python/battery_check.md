# 2024.11.4电池检测

在使用STM32的ADC功能时发现电压测量存在较为明显的波动，因此尝试低通滤波器、中值滤波器、均值滤波器、卡尔曼滤波器进行采样值进行处理。

本次实验通过试验室稳压电源供电3V3，连接18.953K和9.751K电阻两端，通过STM32F103RCT6的ADC1进行采集。

![battery_Kalman_Filter_data](D:\myproject\Projects\doing\无人机\STM32四轴飞行器\test_python\电路.png)

以下是实验函数代码及实验结果：

```c
uint16_t battery_Low_pass_filter(uint16_t battery_voltage)
{
    static uint16_t out_battery_voltage = 0;
    int a = 10;  // 平滑系数a, 代表 0.1 的缩放（10倍）
    
    out_battery_voltage = (a * battery_voltage + (100 - a) * out_battery_voltage) / 100;

    return out_battery_voltage;
}

#define N 10  // 滤波窗口大小
uint16_t battery_Moving_Average_Filter(uint16_t battery_voltage)
{
    static uint16_t buffer[N] = {0};  // 用于存储最近N次电压值
    static uint8_t index = 0;
    uint32_t sum = 0;
    int i;

    // 将新值存入当前索引位置
    buffer[index] = battery_voltage;
    
    // 累加所有N个历史值
    for(i = 0; i < N; i++)
    {
        sum += buffer[i];
    }
    
    // 更新索引，循环使用buffer
    index = (index + 1) % N;
    
    // 返回平均值
    return (uint16_t)(sum / N);
}



#define battery_Median_Filter_N 5  // 滤波窗口大小

uint16_t battery_Median_Filter(uint16_t battery_voltage)
{
    static uint16_t buffer[battery_Median_Filter_N] = {0};  // 用于存储最近N次电压值
    uint16_t temp[battery_Median_Filter_N];
    int i, j;

    // 将新值插入buffer，移位存储最近N次值
    for(i = battery_Median_Filter_N - 1; i > 0; i--)
    {
        buffer[i] = buffer[i - 1];
    }
    buffer[0] = battery_voltage;

    // 将buffer的值复制到temp中，用于排序
    for(i = 0; i < battery_Median_Filter_N; i++)
    {
        temp[i] = buffer[i];
    }

    // 冒泡排序，或其他排序算法，把temp数组按从小到大排序
    for(i = 0; i < battery_Median_Filter_N - 1; i++)
    {
        for(j = i + 1; j < battery_Median_Filter_N; j++)
        {
            if(temp[i] > temp[j])
            {
                uint16_t swap = temp[i];
                temp[i] = temp[j];
                temp[j] = swap;
            }
        }
    }

    // 返回中间值
    return temp[battery_Median_Filter_N / 2];
}


// 定义卡尔曼滤波的状态变量
float battery_Q = 0.001;   // 过程噪声协方差（Q越小，信任模型预测越多）
float battery_R = 0.1;     // 测量噪声协方差（R越小，信任测量值越多）
float x_last = 0;  // 上一次的状态估计
float P_last = 1;  // 上一次的误差协方差

uint16_t battery_Kalman_Filter(uint16_t measurement)
{
    // 初始预测
    float x_predict = x_last;
    float P_predict = P_last + battery_Q;

    // 计算卡尔曼增益
    float K = P_predict / (P_predict + battery_R);

    // 更新估计值
    float x_now = x_predict + K * (measurement - x_predict);
    
    // 更新误差协方差
    float P_now = (1 - K) * P_predict;

    // 将当前状态作为下一次的“上一次”值
    x_last = x_now;
    P_last = P_now;

    // 返回滤波后的结果
    return (uint16_t)x_now;
}
```

每次实验串口采集5000次左右电压值

**低通滤波：**

```c
a(低通滤波)=10,11,15,20,25,30;
```



![battery_Low_pass_filter_data](D:\myproject\Projects\doing\无人机\STM32四轴飞行器\test_python\battery_Low_pass_filter\battery_Low_pass_filter_data.png)

![battery_Low_pass_filter_data_big](D:\myproject\Projects\doing\无人机\STM32四轴飞行器\test_python\battery_Low_pass_filter\battery_Low_pass_filter_data_big.png)

实验发现平滑系数在11到15效果较好

**均值滤波：**

```C
N(均值滤波窗口) =10,20,30,40,50,100,200,300,500
```

![battery_Moving_Average_Filter_data_big](D:\myproject\Projects\doing\无人机\STM32四轴飞行器\test_python\battery_Low_pass_filter\battery_Moving_Average_Filter_data_big.png)![battery_Moving_Average_Filter_data](D:\myproject\Projects\doing\无人机\STM32四轴飞行器\test_python\battery_Low_pass_filter\battery_Moving_Average_Filter_data.png)



![battery_Moving_Average_Filter_data_other](D:\myproject\Projects\doing\无人机\STM32四轴飞行器\test_python\battery_Low_pass_filter\battery_Moving_Average_Filter_data_other.png)

分析发现均值滤波窗口在50左右效果较好，但是理论上应该是窗口越大越好，实验过程中发现，当窗口过大时，会影响输出电压，例如上图中窗口为500时，输出电压反而稳定在1800左右。

**中值滤波：**

```c
battery_Median_Filter_N = 5,10,20,30,40,50,500
```

![battery_Median_Filter_data](D:\myproject\Projects\doing\无人机\STM32四轴飞行器\test_python\battery_Low_pass_filter\battery_Median_Filter_data.png)

![battery_Median_Filter_data_big](D:\myproject\Projects\doing\无人机\STM32四轴飞行器\test_python\battery_Low_pass_filter\battery_Median_Filter_data_big.png)

中值滤波实验过程中发现，滤波窗口越大，稳定效果越好，但是需要耗费一定的时间。



**卡尔曼滤波：**

```C
battery_Q=0.1,0.001,0.0001,0.00001
battery_R=0.001,0.1,0.1,0.1
```

![battery_Kalman_Filter_data](D:\myproject\Projects\doing\无人机\STM32四轴飞行器\test_python\battery_Low_pass_filter\battery_Kalman_Filter_data.png)

![battery_Kalman_Filter_data_big](D:\myproject\Projects\doing\无人机\STM32四轴飞行器\test_python\battery_Low_pass_filter\battery_Kalman_Filter_data_big.png)

卡尔曼滤波中因为实际测量误差较大所以尽可能相信预测值，实验同预计一样Q值越小，越稳定。

**四个滤波器比较：**

滤波器参数选择：

```c
a = 11;
N = 50;
battery_Median_Filter_N = 50;
battery_Q=0.00001;battery_R=0.1;
```



![battery_Kalman_Filter_data](D:\myproject\Projects\doing\无人机\STM32四轴飞行器\test_python\battery_Low_pass_filter\four_filter.png)

![battery_Kalman_Filter_data_big](D:\myproject\Projects\doing\无人机\STM32四轴飞行器\test_python\battery_Low_pass_filter\four_filter_big.png)

**总结**

​	本次通过实际实验测试低通、均值、中值、卡尔曼滤波器的滤波效果，在实际使用中根据需求使用。特别是中值滤波器，窗口越大耗费的时间也越长，不适合用于实时性较高的场景。