# I2C Accelerometer Simulation

## Description
This project was mainly done to refresh my knowledge of I2C protocol while interfacing with an accelerometer on Arduino as well as communicating between Arduino and Matlab via Serial (UART). 
It involves having the accelerometer output data at a higher frequency than the process loop time of Matlab such that a handshake needs to be performed in order for timing to match-up, otherwise it will result in I/O buffers overflowing and data going missing. The accelerometer's current orientation would be updated live on the vizualizer as well as learning to implement a first order low pass filter from the reference book to smoothen out the result.

## My Learnings
Rather than creating a Matlab script, I wanted to learn how to create a Matlab App using the App Designer or GUIDE. It is actually very easy to learn and intuitive as the system automatically creates variables, maintains scope and specifies callbacks for each UI element. I also just recently bought the reference book: [Kalman Filter for Beginners by Phil Kim](https://www.amazon.com/Kalman-Filter-Beginners-MATLAB-Examples/dp/1463648359) and wanted to try out the filters that are covered in the first chapter - which are Average Filters, Moving Average Filters and First Order Low Pass Filter. Along with refreshing my knowledge on I2C and Accelerometer usage & calibration, I learnt a lot about interfacing with Arduino from a Matlab GUI and will definitely want to expand on this further in the future by adding gyroscopes for more accurate tracking.

## Knowledge
* Basic I2C Protocol
* Serial Communication
* Accelerometers
* Matlab App Designer
* Arduino

## Equipment
* MMA8452Q Accelerometer ([Datasheet](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf))
* Arduino Mega 2560 ([Pinout](https://www.arduino.cc/en/Hacking/PinMapping2560))
* 4 Female-to-Male jumper cables
* USB-A to USB-B Arduino power cable

## Software
* Matlab
* Arduino

## References
* [Accelerometer](https://www.digikey.com/en/articles/using-an-accelerometer-for-inclination-sensing)
* [Filtering](https://www.amazon.com/Kalman-Filter-Beginners-MATLAB-Examples/dp/1463648359)
