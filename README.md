# temperature sensor v3
Third temperature sensor implementation : for a ATTiny84a + LM35 + low cost 433Mhz transmitter.
This version uses a enhanced version of the ATTinyx4 serie, the ATTiny84a. With 8k of flash, it
is possible to use the Arduino library (included in this repository, i was unable to link it in
Atmel Studio). A bruteforce technique was used to guess good Manchester timing for emission, since
the provided setupTransmit does not seems to be functional, using 8Mhz or 8/CKDIV8 = 1 Mhz clock.
