# ECE-Senior-Design - Aaron Contributions Only
Code repository for ECE Senior Design Project, with only the parts of the project I helped create.

Project was a system that tracks a bowling ball as it travels down a lane and calculates the initial speed of the ball as well as mapping the trajectory. I was responsible for creating the system that displayed the speed on the on-system LCD display, as well as creating the systems to trigger and read distances from the LiDAR.


--Original Notes--
Microcontroller ports and internal systems required for LiDar/Speed Tracking (Aaron)         
PB10 - I2C2_SCL (Lidar Green)               
PB11 - I2C2_SDA (Lidar Blue)           
PA5 - SPI1_SCK (Display SCK)         
PA7 - SPI1_MOSI (Display MOSI)         
PA4 - SPI1_NSS (Display NSS)          
I2C1: Fast Mode  
SPI1: 10 Bits (may need to change depending on if we buy another LCD Screen)  
TIM7: PSC 8000 -1, Counter period 65535 (Can be any timer, just uses TIM7 in code as example)  
Note: Code uses TIM16 for code speed calculation, is not needed in final project  
