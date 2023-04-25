
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
