# bankswitch-for-wavtrigger
A bank switch extension for floorboard use designed for the WavTrigger board by robertsonic.com

This project is based on a Atmel ATtiny2313A, which connects to up to 6+1 momentary switches, a 7-segment LCD and the WavTrigger board's serial RxD line and 5V power, to implement a floorboard for use on stage. It can trigger up to 6*6 tracks of the WavTrigger board, has a dedicated bank switch input to make a direct choice of the next selected bank, shows the bank number and the triggered track number on a 7 segment LED.
Additionally, the volumes of the tracks can be edited directly on the board and are stored permanently in the EEPROM of the MCU.
