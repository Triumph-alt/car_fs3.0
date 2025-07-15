# car_fs3.0


## 环岛的判定
问题1：出环后识别到入环
解决办法：用定时器限时判定

问题2：出环判定几个角太像了
解决办法：得用靠近直道的区域用VL和HL判定




## 完赛参数记录

1. 低速度55 = 0.5m/s

```c
	pid_init(&SpeedPID, 50.0f, 0.2f, 0.0f, 5000.0f, 6000.0f);      //初始化速度PID
	pid_init(&TurnPID, 70.0f, 0.0f, 7.5f, 0.0f, 6000.0f);          //初始化位置PID

TrackWeights track_weights[4] = {
    // 普通直道

    {0.20f, 0.35, 0.00f, 0.20f, 0.70f, 30, "直道"},
    
    // 直角弯道
    {0.20f, 0.30f, 0.00f, 0.40f, 1.00f, 50, "直角弯道"},
    
    // 十字圆环
    {0.35f, 0.25f, 0.00f, 0.15f, 0.90f, 40, "十字圆环"},
    
    // 环岛
    {0.20f, 0.35f, 0.00f, 0.20f, 1.00f, 50, "环岛"}
};


volatile uint8_t r_position = 30;
volatile uint16_t r_distance = 7400;
volatile uint16_t s_distance = 5500;

```

2. 低速度60
```c
	pid_init(&SpeedPID, 100.0f, 0.25f, 0.0f, 5000.0f, 6000.0f);      //初始化速度PID
	pid_init(&TurnPID, 70.0f, 0.0f, 8.0f, 0.0f, 6000.0f);          //初始化位置PID

TrackWeights track_weights[4] = {
    // 普通直道

    {0.20f, 0.35, 0.00f, 0.20f, 0.70f, 30, "直道"},
    
    // 直角弯道
    {0.20f, 0.30f, 0.00f, 0.40f, 1.00f, 50, "直角弯道"},
    
    // 十字圆环
    {0.35f, 0.25f, 0.00f, 0.15f, 0.90f, 40, "十字圆环"},
    
    // 环岛
    {0.20f, 0.35f, 0.00f, 0.20f, 1.00f, 50, "环岛"}
};


volatile uint8_t r_position = 30;
volatile uint16_t r_distance = 7400;
volatile uint16_t s_distance = 5500;

```

3. 低速度65
```c
	pid_init(&SpeedPID, 70.0f, 0.3f, 0.0f, 5000.0f, 6000.0f);      //初始化速度PID
	pid_init(&TurnPID, 80.0f, 0.0f, 11.0f, 0.0f, 6000.0f);          //初始化位置PID

TrackWeights track_weights[4] = {
    // 普通直道

    {0.20f, 0.35, 0.00f, 0.20f, 0.70f, 30, "直道"},
    
    // 直角弯道
    {0.20f, 0.30f, 0.00f, 0.40f, 1.00f, 50, "直角弯道"},
    
    // 十字圆环
    {0.35f, 0.25f, 0.00f, 0.15f, 0.90f, 40, "十字圆环"},
    
    // 环岛
    {0.20f, 0.35f, 0.00f, 0.20f, 1.00f, 50, "环岛"}
};


volatile uint8_t r_position = 30;
volatile uint16_t r_distance = 7400;
volatile uint16_t s_distance = 5500;

```


4. 中速度70
```c
	pid_init(&SpeedPID, 55.0f, 0.22f, 0.0f, 8000.0f, 9000.0f);      //初始化速度PID
	pid_init(&TurnPID, 95.0f, 0.0f, 13.0f, 0.0f, 9000.0f);          //初始化位置PID

TrackWeights track_weights[4] = {
    // 普通直道

    {0.20f, 0.35, 0.00f, 0.20f, 0.70f, 30, "直道"},
    
    // 直角弯道
    {0.0f, 0.45f, 0.00f, 0.60f, 1.00f, 50, "直角弯道"},
    
    // 十字圆环
    {0.35f, 0.25f, 0.00f, 0.15f, 0.90f, 40, "十字圆环"},
    
    // 环岛
    {0.20f, 0.35f, 0.00f, 0.20f, 1.00f, 50, "环岛"}
};


volatile uint8_t r_position = 30;
volatile uint16_t r_distance = 7400;
volatile uint16_t s_distance = 5500;

```
