/*********
  Complete project details at https://randomnerdtutorials.com
  
  This is an example for our Monochrome OLEDs based on SSD1306 drivers. Pick one up today in the adafruit shop! ------> http://www.adafruit.com/category/63_98
  This example is for a 128x32 pixel display using I2C to communicate 3 pins are required to interface (two I2C and one reset).
  Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries, with contributions from the open source community. BSD license, check license.txt for more information All text above, and the splash screen below must be included in any redistribution. 
*********/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "EasyBuzzer.h"
#include <FunctionalInterrupt.h>
#include <MPU9250_asukiaaa.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define BUTTON_A 14
#define BUTTON_B 25
#define BEEP_IO  26
#define LDRA_IO 39
#define LDRB_IO 36
#define THERMISTOR_IO 34


// Series resistor value
#define SERIESRESISTOR 10000
// Number of samples to average
#define SAMPLERATE 5
// Nominal resistance at 25C
#define THERMISTORNOMINAL 10000
// Nominal temperature in degrees
#define TEMPERATURENOMINAL 25
// Beta coefficient
#define BCOEFFICIENT 3380


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

unsigned int frequency = 1000;  
unsigned int onDuration = 50;
unsigned int offDuration = 100;
unsigned int beeps = 2;
unsigned int pauseDuration = 500;
unsigned int cycles = 10;

int Button_Flag = 0;

class Button
{
public:
  Button(uint8_t reqPin) : PIN(reqPin){
    pinMode(PIN, INPUT_PULLUP);
    attachInterrupt(PIN, std::bind(&Button::isr,this), FALLING);
  };
  ~Button() {
    detachInterrupt(PIN);
  }

  void IRAM_ATTR isr() {
    numberKeyPresses += 1;
    pressed = true;
  }

  void checkPressed() {
    if (pressed) {
      if(PIN==BUTTON_A)
      {
        Button_Flag=1;
      }
      else if(PIN==BUTTON_B)
      {
        Button_Flag=2;
      }
     
      pressed = false;
    }
  }

private:
  const uint8_t PIN;
    volatile uint32_t numberKeyPresses;
    volatile bool pressed;
};

Button button1(BUTTON_A);
Button button2(BUTTON_B);

void done() {
  Serial.print("Done!");
  
}
int getTemp() {
    double thermalSamples[SAMPLERATE];
    double average, kelvin, resistance, celsius;
    int i;
    // Collect SAMPLERATE (default 5) samples
    for (i=0; i<SAMPLERATE; i++) {
        thermalSamples[i] = analogRead(THERMISTOR_IO);
        delay(10);
    }
    // Calculate the average value of the samples
    average = 0;
    for (i=0; i<SAMPLERATE; i++) {
        average += thermalSamples[i];
    }
    average /= SAMPLERATE;
    // Convert to resistance
    resistance = 4095 / average - 1;
    resistance = SERIESRESISTOR / resistance;
    /*
     * Use Steinhart equation (simplified B parameter equation) to convert resistance to kelvin
     * B param eq: T = 1/( 1/To + 1/B * ln(R/Ro) )
     * T  = Temperature in Kelvin
     * R  = Resistance measured
     * Ro = Resistance at nominal temperature
     * B  = Coefficent of the thermistor
     * To = Nominal temperature in kelvin
     */
    //kelvin = resistance/THERMISTORNOMINAL;                              // R/Ro
    kelvin = THERMISTORNOMINAL/resistance;                              // R/Ro
    kelvin = log(kelvin);                                                                   // ln(R/Ro)
    kelvin = (1.0/BCOEFFICIENT) * kelvin;                                   // 1/B * ln(R/Ro)
    kelvin = (1.0/(TEMPERATURENOMINAL+273.15)) + kelvin;  // 1/To + 1/B * ln(R/Ro)
    kelvin = 1.0/kelvin;  
    // Convert Kelvin to Celsius
    celsius = kelvin - 273.15;
    // Send the value back to be displayed
    return celsius;
}
void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.display();
  
}

void setup() {
  Serial.begin(115200);
  EasyBuzzer.setPin(BEEP_IO);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);


  testdrawstyles();    // Draw 'stylized' characters
  EasyBuzzer.beep(
    frequency,    // Frequency in hertz(HZ). 
    onDuration,   // On Duration in milliseconds(ms).
    offDuration,  // Off Duration in milliseconds(ms).
    beeps,      // The number of beeps per cycle.
    pauseDuration,  // Pause duration.
    cycles,     // The number of cycle.
    done      // [Optional] Callback. A function to call when the sequence ends.
  );
   
  /* Always call this function in the loop for EasyBuzzer to work. */
  EasyBuzzer.update();
  delay(1000);
  EasyBuzzer.stopBeep();

  #ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
  #endif

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;


}

void loop() {
  button1.checkPressed();
  button2.checkPressed();
  

  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  
  if(Button_Flag==1)
  {
    //display.println(F("BUTTON_A"));
    int potValue = analogRead(LDRA_IO);
    display.setCursor(0,0);             // Start at top-left corner
    display.print("LDR_A=");
    display.println(potValue);
    potValue = analogRead(LDRB_IO);
    display.setCursor(0,8);
    display.print("LDR_B=");
    display.println(potValue);
    display.setCursor(0,16);
    // Call the function to get the temperature in degrees celsius3
    int temp = getTemp();
    display.print("Temperature: ");
    display.print(temp);
    display.println("C");
    delay(300);
 
  }
   else if(Button_Flag==2)
   {
      //display.println(F("BUTTON_B"));
  
      
      display.setCursor(0,0);
      if (mySensor.accelUpdate() == 0) {
        aX = mySensor.accelX();
        aY = mySensor.accelY();
        aZ = mySensor.accelZ();
        aSqrt = mySensor.accelSqrt();
       
        display.println("accelX: " + String(aX)+" ");
        display.println("accelY: " + String(aY)+" ");
        display.println("accelZ: " + String(aZ)+" ");
               
      } else {
        display.println("Cannod read accel values");
      }
      
      display.setCursor(0,24);
      if (mySensor.gyroUpdate() == 0) {
        gX = mySensor.gyroX();
        gY = mySensor.gyroY();
        gZ = mySensor.gyroZ();
        display.println("gyroX: " + String(gX)+" ");
        display.println("gyroY: " + String(gY)+" ");
        display.println("gyroZ: " + String(gZ)+" ");
      } else {
        display.println("Cannot read gyro values");
      }
      display.setCursor(0,48);
      if (mySensor.magUpdate() == 0) {
        mX = mySensor.magX();
        mY = mySensor.magY();
        mZ = mySensor.magZ();
        mDirection = mySensor.magHorizDirection();
        display.print("X: " + String(mX)+" ");
        display.println("Y: " + String(mY)+" ");
        display.println("Z: " + String(mZ)+" ");
        
      } else {
        display.println("Cannot read mag values");
      }

      /*uint8_t sensorId;
      display.setCursor(0,0); 
      if (mySensor.readId(&sensorId) == 0) {
        display.println("sensorId: " + String(sensorId));
      } else {
        display.println("Cannot read sensorId");
      }*/
      
     
      
      delay(300);
   }
   
   display.display();

 

  
}
