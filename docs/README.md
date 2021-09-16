# Micropython-STM32-STM32F103

## Micropython 1.11 version  

## Linux 
```  
sudo apt-get install gcc-arm-none-eabi  
sudo apt-get install gcc  
```
## How to compiler 
Switch to ports/stm32 directory and open terminal 
```
make BOARD=F103  
```

## Use board
![Alt text](/docs/board.png)


###  Now working
1. pyb.freq()  
2. switch  
3. timer  
4. timer pwm   
5. led  
6. rtc  
7. dac  
8. adc  
9. extint  
10. pin  
11. uart  
12. spi  
13. i2c  
14. sdcard  
15. usb MSC  
16. usb cdc  ***(Need open the DTR)***  
### Not tested
1. CAN   
### Problem
1. Some peripherals will have problems using DMA  
2. i2c will restart when using dma  

## Result
![Alt text](/docs/result.png)
