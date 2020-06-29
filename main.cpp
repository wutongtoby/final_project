#include "mbed.h"
#include "bbcar.h"
#include "bbcar_rpc.h"

#define LEFT 1
#define RIGHT 0
#define FORWARD 1
#define BACKWARD 0

// Servo
Ticker servo_ticker;
PwmOut pin13(D13), pin10(D10);
BBCar car(pin13, pin10, servo_ticker);

// Encoder
DigitalIn pin4(D4), pin5(D5);
Ticker encoder_ticker0;
Ticker encoder_ticker1;
parallax_encoder en1(pin4, encoder_ticker1);
parallax_encoder en0(pin5, encoder_ticker0);

// Ping
DigitalInOut pin6(D6);
parallax_ping ping(pin6);

// Xbee
RawSerial xbee(D1, D0); // Tx, Rx

// OpenMv
Serial OpenMv(D12, D11); // Tx, Rx

// LED
DigitalOut Led(LED1);

void identify_picture(void); // B
void identify_object(void);  // C

// for the log message
bool other_mission = false;
void log(void);
Thread t(osPriorityHigh);
char log_message;

// for car control
void ping_WalkUntil(float distance, bool Is_Forward);
void lr_turn(bool Is_left);

int main(void) 
{
    // set the baud rate for both
    xbee.baud(9600);
    OpenMv.baud(9600);
    t.start(&log);

    xbee.printf("Begin\r\n");
    // set the LED
    Led = 1;

    // for D13 left, servo0
    double pwm_table0[] = {-40, -32, -24, -16, -8, 0, 8, 16, 24, 32, 40};
    double speed_table0[] = {-19.059, -14.355, -10.367, -5.343, -1.754, 0, 2.153, 7.018, 11.165, 16.109, 20.096};     
    // for D10 right, servo1
    double pwm_table1[] = {-120, -96, -72, -48, -24, 0, 24, 48, 72, 96, 120};
    double speed_table1[] = {-16.108, -15.312, -13.557, -9.950, -4.227, 0, 4.944, 10.367, 13.956, 15.631, 16.348};
    

    // first and fourth argument : length of table
    car.setCalibTable(11, pwm_table0, speed_table0, 11, pwm_table1, speed_table1);

    // start 
    
    // go until at the front of wall
    log_message = 'F';
    ping_WalkUntil(15, FORWARD);
    
    
    log_message = 'L';
    lr_turn(LEFT);
    wait(0.5);

    // go until at the target place, then turn right + go backward
    log_message = 'F';
    car.goStraightCalib(10);
    wait(4.5);
    
    log_message = 'S';
    car.stop();
    wait(1);
    
    log_message = 'R';
    lr_turn(RIGHT);
    wait(0.5);
    
    log_message = 'b';
    ping_WalkUntil(35, BACKWARD);
    wait(0.5);

    // go to the next mission
    log_message = 'F';
    ping_WalkUntil(15, FORWARD);
    

    log_message = 'L';
    lr_turn(LEFT);
    wait(0.5);
    
    log_message = 'F';
    car.goStraightCalib(10);
    wait(5.0);

    log_message = 'F';
    ping_WalkUntil(5, FORWARD);
    wait(0.5);

    log_message = 'R';
    lr_turn(RIGHT);
    wait(1.0);
    
    log_message = 'b';
    car.goStraightCalib(-10);
    wait(1);
    car.stop();
    
    identify_picture();
    
    log_message = 'F';
    ping_WalkUntil(15, FORWARD);
    wait(1.0);
    
    log_message = 'R';
    lr_turn(RIGHT);
    wait(1.0);

    log_message = 'F';
    car.goStraightCalib(10);
    wait(5);

    log_message = 'F';
    ping_WalkUntil(30, FORWARD);
    wait(0.5);

    log_message = 'R';
    lr_turn(RIGHT);
    wait(1.0);

    log_message = 'F';
    ping_WalkUntil(12.5, FORWARD);
    car.stop();
    wait(1);

    log_message = 'R';
    lr_turn(RIGHT);
    wait(1.0);

    log_message = 'F';
    car.goStraightCalib(10);
    wait(2.35);

    log_message = 'S';
    car.stop();
    wait(1.0);
    
    log_message = 'R';
    lr_turn(RIGHT);
    wait(1.0);
    
    log_message = 'F';
    car.goStraightCalib(10);
    wait(2.5);

    log_message = 'S';
    car.stop();
    wait(1.0);

    identify_object();
    wait(1.0);

    // go to the end
    log_message = 'L';
    lr_turn(LEFT);
    wait(1.0);
    
    log_message = 'F';
    car.goStraightCalib(10);
    wait(2.35);

    log_message = 'L';
    lr_turn(LEFT);
    wait(0.5);

    log_message = 'F';
    ping_WalkUntil(15, FORWARD);    
    wait(0.5);

    log_message = 'R';
    lr_turn(RIGHT);
    wait(0.5);
    
    log_message = 'F';
    ping_WalkUntil(15, FORWARD);
    wait(0.5);

    log_message = 'R';
    lr_turn(RIGHT);
    wait(0.5);

    log_message = 'F';
    car.goStraightCalib(10);
    wait(12.0);
    car.stop();

    other_mission = true;
    xbee.printf("The End\r\n");
    return 0;
}

void ping_WalkUntil(float distance, bool Is_Forward)
{
    if (Is_Forward) {
        car.servo0.set_speed_by_cm(10);
        car.servo1.set_speed_by_cm(-9.8);
        car.controlWheel();
    }
        //car.goStraightCalib(10);
    else 
        car.goStraightCalib(-10);

    while (1) {
        if (Is_Forward && float(ping) <= distance)
            break;
        else if (!Is_Forward && float(ping) >= distance)
            break;
        wait(0.1);
    };
    car.stop();
}

void lr_turn(bool Is_Left)
{
    if (Is_Left) {
        car.servo0.set_speed_by_cm(-10);
        car.servo1.set_speed_by_cm(-10);
        car.controlWheel();
        wait(1.1);
    }
    else {
        car.servo0.set_speed_by_cm(10);
        car.servo1.set_speed_by_cm(10);
        car.controlWheel();
        wait(1.2);
    }
    car.stop();
}

void log(void) {
    int count = 0;
    while (1) {
        if (!other_mission)
            xbee.printf("%d %c\r\n", count++, log_message);
        wait(1.0);
    }
}

void identify_object(void)
{
    log_message = 'C';
    float value[3];
    float difference_1, difference_2;

    // turn right, for 30 deg
    car.servo0.set_speed_by_cm(10);
    car.servo1.set_speed_by_cm(10);
    car.controlWheel();
    wait(1.12 / 3);
    car.stop();
    value[0] = float(ping);
    wait(1.0);

    // turn left for 30 deg 
    car.servo0.set_speed_by_cm(-10);
    car.servo1.set_speed_by_cm(-10);
    car.controlWheel();
    wait(1.12 / 3);
    car.stop();
    value[1] = float(ping);
    wait(1.0);

    // turn left for 30 deg 
    car.servo0.set_speed_by_cm(-10);
    car.servo1.set_speed_by_cm(-10);
    car.controlWheel();
    wait(1.12 / 3);
    car.stop();
    value[2] = float(ping);
    wait(1.0);

    // turn right for 30 deg, back to origin
    car.servo0.set_speed_by_cm(10);
    car.servo1.set_speed_by_cm(10);
    car.controlWheel();
    wait(1.12 / 3);
    car.stop();
    
    difference_1 = value[1] - value[0];
    other_mission = true;
    if (difference_1 <= 0)
        xbee.printf("The object is M\r\n");
    else {
        if (difference_2 >= 0)
            xbee.printf("The object is Right triangle\r\n");
        else {
            if (abs(difference_1) - abs(difference_2) < 2)
                xbee.printf("The object is Square\r\n");
            else 
                xbee.printf("The object is Equal triangle\r\n");
        }
    }
    other_mission = false;
    // mission complete
    for (int i = 0; i < 5; i++) {
        Led = 0;
        wait(0.1);
        Led = 1;
        wait(0.1);
    }
}

void identify_picture(void)
{
    log_message = 'B';
    char s[25];
    sprintf(s, "image_classification");
    OpenMv.puts(s);
    wait(5.0);
    
    other_mission = true;

    // will get only one number
    if (OpenMv.readable()) {
        char recv = OpenMv.getc();
        xbee.printf("The number is ");
        xbee.putc(recv);
        xbee.printf("\r\n");
    }
    
    other_mission = false;
    // mission complete
    for (int i = 0; i < 5; i++) {
        Led = 0;
        wait(0.1);
        Led = 1;
        wait(0.1);
    }
}