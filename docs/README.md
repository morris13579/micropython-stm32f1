# Micropython-STM32-STM32F103

在linux上安裝  
sudo apt-get install gcc-arm-none-eabi  
sudo apt-get install gcc  

打開cmd  
切換到目錄ports/stm32  
make BOARD=F103  

目前功能  
pyb.freq() ok  
switch ok  
timer ok  
timer pwm ok  
led ok  
rtc ok  
dac ok  
adc ok  
extint ok  
pin ok  
uart ok  
spi ok  
can ok not test  
i2c not test  

