
##### Environment:Matlab2021b
# English 

## content of this work
1. PID controller
2. Collision avoidance by ECBF(exponential control barrier function)
Simuation in this code will show a UAV track the moving target with PID controller. Moreover, the collision avoidance of moving target will be practice by ECBF in this work. ECBF will moduled the accerelation command to make sure UAV will not hit anything during tracking.

## RUN
press run in matlab in main.m
you can see 4 figure:
- figure 1 -> the comparison of the position between target and UAV
- figure 8 -> the comparison of the accerelation commands between generating by controller and moduled by ECBF
- figure 13 -> the comparison of the position between target and UAV
- figure 14 -> the animation of simulation

## Code
1. pid controller contents 5 codes.
 - altitude_control        -> give force command
 - LateralPositionControl  -> give accerelation commands of x and y-axis
 - ecbf                    -> change force command in accerelation commands of z-axis
 - RollPitchControl        -> give desired omega command 
 - BodyRateControl         -> give moment command
2. quad_dynamics is dynamics model of UAV

# Chinese

## 內容
1. PID 控制器
2. 用ECBF進行避障
Simuation in this code will show a UAV track the moving target with PID controller. Moreover, the collision avoidance of moving target will be practice by ECBF in this work. ECBF will moduled the accerelation command to make sure UAV will not hit anything during tracking.

## RUN
進到main按下run就會自己跑slx不用特別點開檔案跑，如果要檢查的話slx檔案是2021b版本
- 第一張圖片 figure 1 是目標位置與現在無人機位置的比較圖
- 第二張圖片 figure 8 是原始加速度命令與經過ECBF優化後的比較
- 第三張圖片 figure 13 是全程的軌跡變化
- 第四張圖片 figure 14 是多張圖，會快速跳出每0.01秒的圖片

## 程式講解
1. pid controller 內部包含五支程式
 - altitude_control        -> 給force命令
 - LateralPositionControl  -> 給左右方向加速度命令
 - ecbf                    -> 將升力命令轉為垂直加速度，結合LateralPositionControl得到三軸加速度，並優化加速度命令
 - RollPitchControl        -> 算出期望角速度
 - BodyRateControl         -> 給出moment命令

2. quad_dynamics 為無人機運動模型

# ref
[controller](https://github.com/haragg2/FCND-Controls.git)
[ECBF paper](https://ieeexplore.ieee.org/abstract/document/8463194)

