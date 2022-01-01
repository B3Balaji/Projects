// Arduino Implementation File
// DataSheet: https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf

// I2C Library
#include <Wire.h>
 
// MMA8452Q I2C address (use i2c_scanner.ino to verify sensor location)
#define Addr 0x1D
 

// Run Once
void setup()
{
  // Initialise I2C communication as MASTER
  Wire.begin();
  
  // Initialise Serial Communication (9600 Baud Rate)
  Serial.begin(9600);
 
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  
  // Select control register 1 (All register locations defined on datasheet)
  Wire.write(0x2A);
  
  // Remove all existing Settings
  Wire.write(0x00);
  
  // Stop I2C Transmission
  Wire.endTransmission();
 
  
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  
  // Select control register 1
  Wire.write(0x2A);
  
  // Active mode (Detailed bit breakdown on datasheet)
  Wire.write(0x01);
  
  // Stop I2C Transmission
  Wire.endTransmission();
 
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  
  // Select control register 2
  Wire.write(0x0E);
  
  // Set range to +/- 2g (Can be 4g or 8g but loses sensitivity)
  Wire.write(0x00);
  
  // Stop I2C Transmission
  Wire.endTransmission();
  
  delay(300);
}


// Run Multiple Times
void loop()
{
  
  // Initialise buffer for raw values
  unsigned int data[7];
  
  // Get 7 bytes of data from Sensor
  Wire.requestFrom(Addr, 7);
 
  // Read 7 bytes of data (refer to datasheet for order)
  // Status, X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB
  if(Wire.available() == 7) 
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();
    data[6] = Wire.read();
  }
 
  // Data is 2's complement, convert to 12-bit int.
  int xAccl = ((data[1] * 256) + data[2]) / 16;
  if (xAccl > 2047)
  {
    xAccl -= 4096;
  }
 
  int yAccl = ((data[3] * 256) + data[4]) / 16;
  if (yAccl > 2047)
  {
    yAccl -= 4096;
  }
 
  int zAccl = ((data[5] * 256) + data[6]) / 16;
  if (zAccl > 2047)
  {
    zAccl -= 4096;
  }
 
  // Output data to serial output buffer (x, y, z\n)
    Serial.print(xAccl);
    Serial.print(" ,");
    Serial.print(yAccl);
    Serial.print(" ,");
    Serial.println(zAccl);
  
  // Handshake with Matlab only to proceed to next loop ONLY once Matlab has processed the current data
  // This is to prevent timing issue as Sensor ODR is well faster than Matlab processing time.
   while(Serial.read() != 'c');
}
