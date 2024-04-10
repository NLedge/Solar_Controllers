/* Power Monitor V3.0 for ECE 481 Capstone
 *  Backwoods Solar Project
 *  4 October 2022
 *  Dahmen Garner
 *  
 *  Here is some information on this program: -- it has been slightly calibrated
 *  
 *  Calibration parameters:
 *  
 *      Sensor Sensitivity:
 *        The first group of parameters, variables ending with sens, is used for calibrating the hall effect sensor, 
 *        the ACS712 chip.  These values are taken by noting the output voltage of the sensor with ZERO current
 *        flowing through it.  Then, you pass EXACTLY 1 amp through the sensor, and note the output voltage.
 *        Then, you put the change in voltage over change in current (note that change in current is 1 since we passed
 *        only 1 amp through the sensor), as the sense value.  This must be calibrated for each current hall effect sensor.
 *      
 *      Sensor Zero Current output voltage (zervol)
 *        This calibration parameter is straightforward.  Measure the output voltage of the current sensor with
 *        ZERO current flowing through it.  Place this value as the variable for each sensor
 *      
 *      Zero Current Arduino Analog Read Value
 *        This test calibrates the analog read value of the arduino with zero current flowing through each of the shunts.
 *        Simply use the analogRead() function and Serial.print() function to read and print the values into the console
 *        with zero current flowing through each of the sensors.
 *        
 *      Vratio
 *        This is the voltage ratio from the voltage divider.  This voltage divider is used to divide the voltage
 *        down from battery voltage levels to a safe value for an analog input pin.  For this project, a ratio of 
 *        15/5 or 3 was used to drop a max battery voltage of 15 volts down to the save level of 5 volts.  Simply
 *        use a resistor divider circuit.
 *        
 *      mini
 *        This is the minimumn current to measure in the program.  It is used to exclude noisy current signals by
 *        excluding read values less than this
 *        
 *  Input Pins:
 *    These are the analog input pins for each of the current sensors, as well as the voltage divider input.
 *    They may be changed around if required.
 *      
 *  Additional function descriptions can be found in the comments of the code    
 * 
 */
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4); //sets the LCD size to 20x4.  Replace the 20 and 4 depending on the size of the LCD 

//Calibration Parameters -- Hall Effect Sensors.  Note that the sensitivity for these is in V/A
//sensor sensitivity
float invsens = 0.068; //V/A, the measured sensitivity of the ACS712 chip
float solsens = 0.068;
float usbsens = 0.185; //for the 5 amp hall effect sensor, manual recommends 0.185
float _12vsens = 0.068;

//sensor zero current output voltage
float invzervol = 2.5094;
float solzervol = 2.5064;
float usbzervol = 2.4997; //the measured output voltage of the ACS712 chip with zero current
float _12vzervol = 2.5117;

//zero current arduino analog read value
float invzerval = 510; //the analog read value of zero current passing through the ACS712 chip
float solzerval = 510;
float usbzerval = 508; //keep this
float _12vzerval = 511;

float vratio = 15/5; //for voltage measuring
float mini = 0.09; //minimum current, for keeping things clean

//Input Pins
int vbattpin = A0; //battery resistor divider --green
int isolarpin = A1; //solar charge controller --yellow
int iinvpin = A2; //inverter --orange
int iusbpin = A3; //usb --red --not used, keeps crashing
int i12vpin = A6; //12v aux cig lighter --brown

//testing pins
//int vbattpin = A0; //12v sense
//int isolarpin = A1; //solar shunt
//int iinvpin = A1; //inverter shunt
//int iusbpin = A2; 
//int i12vpin = A1;


int button = 2; //select button, I am using digital pin 2 since pins 0 and 1 are for rx and tx data

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(button, INPUT); //use this to setup the digital input
  lcd.init();
  lcd.backlight();
  lcd.begin(20,4);
  lcd.setCursor(3,1); //column, row
  lcd.print("Zurg Rush...");
  Serial.println("xxSerialxx hello there...");
  delay(2000);
  


}

float readcurrent(int pin, float sens, float zervol, float zerval, float mini){
  float val = 0;
  float current = 0;
  int thresh = 0;
  val = analogRead(pin);
  current = abs(((val * zervol/zerval) - zervol) * (1/sens)); //absolute takes care if the reading comes out negative
  if (current < mini){
    current = 0;
  }
  
  return current;
  
}

float readvoltage(int pin){ //reads and calculates the battery voltage
  float val = 0;
  float voltage = 0;
  val = analogRead(pin);
  //delay(10);
  voltage = vratio * 5/1024 * val;
  return voltage;
  
}

void battparams(float vbatt, float isolar, float iloads) { //to display current battery voltage, and charging info
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Battery Info");
  lcd.setCursor(0,1); //prints the battery voltage
  lcd.print("Voltage:  ");
  lcd.print(vbatt);
  lcd.print("V");
  lcd.setCursor(0,2);
  lcd.print("Charge:");
  lcd.print(isolar);
  lcd.print("A/");
  lcd.print(isolar*vbatt);
  lcd.print("W");
  lcd.setCursor(0,3);
  lcd.print("Loads:");
  lcd.print(iloads);
  lcd.print("A/");
  lcd.print(vbatt*iloads);
  lcd.print("W");
  delay(1500);
  
}

void loads(float iinv, float i12v, float iusb, float vbatt) { //displays the load currents
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("Loads: ");

  lcd.setCursor(0,1);
  lcd.print("Inverter:   ");
  lcd.print(iinv*vbatt);
  lcd.print("W");
  
  lcd.setCursor(0,2);
  lcd.print("12 V Out:   ");
  lcd.print(i12v*vbatt);
  lcd.print("W");
  
  lcd.setCursor(0,3);
  lcd.print("USB Output: ");
  lcd.print(iusb*vbatt);
  lcd.print("W");
  
  delay(1500);
  lcd.clear();
  
  lcd.setCursor(0,0);
  lcd.print("Loads: ");
  
  lcd.setCursor(0,1);
  lcd.print("Inverter:   ");
  lcd.print(iinv);
  lcd.print("A");
  
  lcd.setCursor(0,2);
  lcd.print("12 V Out:   ");
  lcd.print(i12v);
  lcd.print("A");
  
  lcd.setCursor(0,3);
  lcd.print("USB Output: ");
  lcd.print(iusb);
  lcd.print("A");
  
  delay(1500);
 
}

void loop() {

  float vbatt = 0;
  float isolar = 0;
  float iinv = 0;
  float iusb = 0;
  float i12v = 0;
  int loops = 10;

  for (int i = 1; i <= loops; i++){ //reads and averages all parameters loop# of times, delays 50msec
    vbatt = vbatt + readvoltage(vbattpin);
    isolar = isolar + readcurrent(isolarpin, solsens, solzervol, solzerval, mini); //recall, readcurrent relies on: int pin, float sens, float zervol, float zerval
    iinv = iinv + readcurrent(iinvpin, invsens, invzervol, invzerval, mini);
    iusb = iusb + readcurrent(iusbpin, usbsens, usbzervol, usbzerval, mini);
    i12v = i12v + readcurrent(i12vpin, _12vsens, _12vzervol, _12vzerval, mini);
    delay(75);
  }
  //average voltages/currents
  vbatt = vbatt/loops;
  isolar = isolar/loops;
  iinv = iinv/loops;
  iusb = iusb/loops;
  i12v = i12v/loops;
  float itot = iinv + iusb + i12v; //total current

  //the following is the menu selection:
  if (digitalRead(button) == LOW){
    battparams(vbatt, isolar, itot);
  }

  if (digitalRead(button) == HIGH){
    loads(iinv, i12v, iusb, vbatt);
  }

}