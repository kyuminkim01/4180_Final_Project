### Remotely Controlled Robot with Alarm

Team Members: Kyumin Kim and Yifan Wang

Georgia Institute of Technology

Watch our demo video:

Demonstration : https://youtu.be/-xvhX-ossBY



***

### Table of Contents  
1. [Project Overview](https://github.com/kyuminkim01/4180_Final_Project/wiki#project-overview)  
2. [Parts List](https://github.com/kyuminkim01/4180_Final_Project/wiki#parts-list)
3. [Schematic and Circuitry](https://github.com/kyuminkim01/4180_Final_Project/wiki#schematic-and-circuitry)
4. [Source Code](https://github.com/kyuminkim01/4180_Final_Project/wiki#source-code)
5. [Future Direction](https://github.com/kyuminkim01/4180_Final_Project/wiki#future-direction)




***

### Project Overview

<img width="777" alt="image" src="https://user-images.githubusercontent.com/122822330/234151412-47ec72e2-5fe6-406b-885e-9d3181978a53.png">

Th user will use the control pad on the "BluefruitConnect" mobile app to control the speed and directions of the robot movement. Button 1-4 set the speed level to 0.0 , 0.3 , 0.6 , 0.9 m/s respectively. "Forward arrow" will set the movement to forward direction and "reverse arrow" will set the movement to reverse direction. "Left arrow" will set the robot to turn left and "Right arrow" will set the robot to turn right.

This is a picture of our robot.
![Robot](https://user-images.githubusercontent.com/113227334/234109507-9f8b1897-de77-4165-ad78-6fca4d7d8666.jpg)

This is the robot block diagram.
![block diagram](https://user-images.githubusercontent.com/122822330/234113114-e671c4bf-98e9-4767-a168-f0c624a5a74d.png)


***

### Parts List

* mbed LPC1768, https://www.sparkfun.com/products/9564
* Distance Sensor - HC-SR04, https://www.sparkfun.com/products/15569
* Serial Miniature LCD Module, https://www.sparkfun.com/products/11377
* Audio Amp - TPA2005D1, https://www.sparkfun.com/products/11044
* Speaker - PCB Mount, https://www.sparkfun.com/products/11089
* LED - RGB, https://www.sparkfun.com/products/105
* Bluetooth Low Energy, https://www.adafruit.com/product/2479#description
* Hobby Gearmotor, https://www.sparkfun.com/products/13302
* Motor Driver - Dual TB6612FNG, https://www.sparkfun.com/products/14450
* Shadow Chassis, https://www.sparkfun.com/products/13301

***


### Schematic and Circuitry

This is a schematic of our project
![unnamed](https://user-images.githubusercontent.com/113227334/234116802-96ebb0b3-f97a-4aeb-9828-0cb5992712ca.png)


&nbsp;
&nbsp;



| Battery 2     |     Battery 1 | Mbed          |     Bluetooth |    HBridge    |    Motor1     |   Motor2      | Class D Amp       |  Speaker      | 
| ------------- | ------------- | ------------- | ------------- | ------------- |-------------  |-------------  |-------------      |-------------  |
|               | +             | VIN           |VIN (3.3-16V)  |               |               |               |       Power+      |               |
|               |               | VOUT          |               |     VCC,STBY  |               |               |                   |               |
| GND           | GND           |     GND       |     GND       |       GND     |               |               | In -, Power-      |               |
|               |               |     p28       |     RXI       |               |               |               |                   |               |
|               |               |     p27       |     TXO       |               |               |               |                   |               |
|               |               |     GND       |     CTO       |               |               |               |                   |               |
|     +         |               |               |               |       VMOT    |               |               |                   |               |
|               |               |      p23      |               |        PWMA   |               |               |                   |               |
|               |               |      p6       |               |        AIN1   |               |               |                   |               |
|               |               |      p5       |               |        AIN2   |               |               |                   |               |
|               |               |      p22      |               |        PWMB   |               |               |                   |               |
|               |               |      p12      |               |        BIN1   |               |               |                   |               |
|               |               |      p11      |               |        BIN2   |               |               |                   |               |
|               |               |               |               |       AOUT1   |       +       |               |                   |               |
|               |               |               |               |       AOUT2   |        -      |               |                   |               |
|               |               |               |               |       BOUT1   |               |      +        |                   |               |
|               |               |               |               |       BOUT2   |               |       -       |                   |               |
|               |               |        p24    |               |               |               |               | In +              |               |
|               |               |               |               |               |               |               |   OUT+            | speaker +     |
|               |               |               |               |               |               |               |   OUT-            | speaker -     |

Table 1. Circuitry for the connection of Mbed, bluetooth, hbridge, motors, class D amp and speaker

&nbsp;
&nbsp;




|     Battery 1 | Mbed          |  Sonar Sensor |    RGB Led    |    uLCD       |   
| ------------- | ------------- | ------------- | ------------- |-------------  |
| +             | VIN           |      VCC      |               |      +5V      | 
|  GND          |   GND         |      GND      |      GND      |      GND      |  
|               |     p30       |    Ecco       |               |               | 
|               |      p29      |   Trig        |               |               |
|               |      p26      |               |     Red       |               |   
|               |      p25      |               |     Green     |               |  
|               |      p21      |               |     Blue      |               |   
|               |      p9       |               |               |     Tx        |   
|               |      p10      |               |               |     Rx        |   
|               |      p8       |               |               |     Res       | 
 
Table 2. Circuitry for the connection of Mbed, sonar sensor, RGB Led, and uLCD

(Note: Use of external 5v power is REQUIRED, because we need enough current to drive servos and the LED array. Connect everything to a common ground.)



***

### Source Code

All source code is included in this repository.

```

#include "mbed.h"
#include "rtos.h"
#include "SDFileSystem.h"
#include "Motor.h"
#include <cstdint>
#include "uLCD_4DGL.h"
#include "SongPlayer.h"
#include "ultrasonic.h"


Motor m1(p23, p6, p5); // pwm, fwd, rev
Motor m2(p22, p12, p11);
Serial blue(p28, p27);

//uLCD
uLCD_4DGL LCD(p9,p10,p8); // serial tx, serial rx, reset pin;

float note[1]= {1700.0};
float duration[1]= {0.5};
//Declare global variables
volatile int current_distance = 500;
volatile float current_speed = 0.0;
volatile float m1_direction = 1.0;
volatile float m2_direction = 1.0;
// mutex to make the lcd lib thread safe
Mutex lcd_mutex;
SongPlayer mySpeaker(p24);
//Distance fucntion for sonar sensor
volatile int s = 0;

class RGBLed
{
public:
    RGBLed(PinName redpin, PinName greenpin, PinName bluepin);
    void write(float red,float green, float blue);
private:
    PwmOut _redpin;
    PwmOut _greenpin;
    PwmOut _bluepin;
};

RGBLed::RGBLed (PinName redpin, PinName greenpin, PinName bluepin)
    : _redpin(redpin), _greenpin(greenpin), _bluepin(bluepin)
{
    //50Hz PWM clock default a bit too low, go to 2000Hz (less flicker)
    _redpin.period(0.0005);
}

//Distance fucntion for sonar sensor
void dist(int distance)
{
    //printf("Distance %d mm\r\n", distance);
    current_distance = distance;
}

ultrasonic mu(p29, p30, .1, 1, &dist);    //Set the trigger pin to D8 and the echo pin to D9
                                        //have updates every .1 seconds and a timeout after 1
                                        //second, and call dist when the distance changes

void RGBLed::write(float red,float green, float blue)
{
    _redpin = red;
    _greenpin = green;
    _bluepin = blue;
}
//class could be moved to include file

RGBLed myRGBled(p26,p25,p21); //RGB PWM pins

void RGBlight(void const *argument){
    while(1){
        if(s == 1){
            myRGBled.write(0.0, 1.0, 1.0); // Yellow for reverse movement
        }
        if(s == 2){
           myRGBled.write(0.0, 0.0, 1.0);  // RED for stop
        }
        if(s == 3){
           myRGBled.write(0.0, 1.0, 0.0);  // GREEN for forward movement
        }
        if(s == 4){
           myRGBled.write(1.0, 0.0, 0.0);  // BLUE for turning left/right
        }

        Thread::wait(50);
    }
}


//Sonar Sensor Thread
void Sensor_Thread(void const *args)
{
    mu.startUpdates();//start measuring the distance
    while(1)
    {
        mu.checkDistance();     //call checkDistance() as much as possible, as this is where
                                //the class checks if dist needs to be called.
        Thread::wait(50);
    }
}

//uLCD thread
void uLCD_Thread(void const *args)
{
    while(1) {       // thread loop
        lcd_mutex.lock();
        LCD.color(GREEN);
        LCD.locate(0,0);
        LCD.text_height(2);
        LCD.text_width(2);
        LCD.printf("Distance:");
        LCD.locate(0,2);
        LCD.printf("%2.0d mm         ",current_distance);
        LCD.locate(0,4);
        LCD.color(RED);
        LCD.printf("Speed:");
        LCD.locate(0,6);
        LCD.printf("%2.0f m/s         ",10*current_speed);
        lcd_mutex.unlock();
        Thread::wait(50);
    }
}

// Audio
void Audio_Thread(void const *args)
{
    while(1) {         // thread loop
        if (current_distance <= 500){
                mySpeaker.PlaySong(note,duration);
                wait(0.9);
                
        } 
        Thread::wait(100);
    }
}


int main() {
    char bnum = 0;
     //start threads
    Thread thread1(RGBlight);
    Thread sonar(Sensor_Thread);
    Thread uLCD(uLCD_Thread);
    Thread audio(Audio_Thread);

    while(1) {
        if(blue.readable()){
            lcd_mutex.lock();
            if(blue.getc() == '!'){
                if(blue.getc() == 'B'){
                    bnum = blue.getc();
                    switch(bnum){
                        case '1':
                            current_speed = 0.0;
                            m1.speed(m1_direction*current_speed);
                            m2.speed(m2_direction*current_speed);
                            s = 2;
                            break;
                        case '2':
                            current_speed = 0.3;
                            m1.speed(m1_direction*current_speed);
                            m2.speed(m2_direction*(current_speed+0.02));
                            if((m1_direction == 1.0) &&(m2_direction == 1.0)) s = 3;
                            else s = 1;
                            break;
                        case '3':
                            current_speed =0.6;
                            m1.speed(m1_direction*current_speed);
                            m2.speed(m2_direction*(current_speed+0.02));
                            if((m1_direction == 1.0) &&(m2_direction == 1.0)) s = 3;
                            else s = 1;
                            break;
                        case '4':
                            current_speed = 0.9;
                            m1.speed(m1_direction*current_speed);
                            m2.speed(m2_direction*(current_speed+0.02));
                            if((m1_direction == 1.0) &&(m2_direction == 1.0)) s = 3;
                            else s = 1;
                            break;

                        case '5':
                            m1.speed(float(current_speed));
                            m2.speed(float(current_speed));
                            s = 3;
                            m1_direction = 1.0;
                            m2_direction = 1.0;
                            break;
                        case '6':
                            m1.speed(float(-current_speed));
                            m2.speed(float(-current_speed));
                            s = 1;
                            m1_direction = -1.0;
                            m2_direction = -1.0;
                            break;
                        case '7':
                            m1.speed(float(0.0));
                            m2.speed(float(0.3));
                            s = 4;
                            break;
                        case '8':
                            m1.speed(float(0.3));
                            m2.speed(float(0.0));
                            s = 4;
                            break;
                        default:
                            break;
                }
                }
                    }
                    lcd_mutex.unlock();
                }
                 Thread::wait(50);
            }
    }

```


***

### Future Direction

* Robot response can be slow occasionally. We will improve the wireless connection and optimize responding time.
* Distance sensor might give wrong reading if the speed changes rapidly. We will modify the mbed codes, so the speed changing process is smoother. 
* The robot is not moving in a straight line when in reverse movement. We will do more mechanical examination and testing to fix the issue.

