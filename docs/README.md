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
i2c ok  
sdcard ok  
usb MSC ok  
usb cdc ok 需要開啟DTR  
can not test  


目前問題  
有些外設使用DMA會有問題  
i2c 使用dma會重啟  
