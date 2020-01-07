# f051bldc
brushless motor control stm32f051
January 2020 update, fresh start.
Project has been moved to cubeIDE so entire directory can be downloaded and can be opened in that IDE.
For other IDE's linux probably its best to open the cubeMX file and genereate peripheral setup code for the appropriate ide then copy main.c and _it.c 

Currently working:
Vehicle modes, 
1) eeprom mode or multirotor (saved settings), dshot commands to enable 3d mode or motor direction.
               
2) crawler, thruster mode (reversible, full brake and no freewheeling, sinusoidal startup)
               
3) rc car mode, reverse applies adjustable brake, double tap to reverse 
               
5) No eeprom , to test settings 

Complementary pwm on/off option.

Input autodetection , dshot 300, 600 or pwm  ( others are in firmware but disabled for now, oneshot, multishot, proshot)

Variable frequency pwm output, matches motor rotation for most used range (varies on the fly from 24-48khz).

Slow start sinusoidal modes, for heavy loads, crawlers

dshot commands

bi-dir dshot - dshot300 only.

adjustable braking strength in car modes

Brushed motor modes

bi polar pwm

