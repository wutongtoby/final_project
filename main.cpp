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

void identify_matrix(void); 
void identify_picture(void);
void identify_object(void); 

// for the log message
bool other_mission = false;
void log(void);
Thread t;
char log_message;

// for car control
void ping_WalkUntil(float distance);
void lr_turn(bool Is_left);

int main(void) 
{
    // set the baud rate for both
    xbee.baud(9600);
    OpenMv.baud(9600);
    t.start(&log);

    // set the LED
    Led = 1;

    // for D13 
    double pwm_table0[] = {-40, -32, -24, -16, -8, 0, 8, 16, 24, 32, 40};
    double speed_table0[] = {-19.059, -14.355, -10.367, -5.343, -1.754, 0, 2.153, 7.018, 11.165, 16.109, 20.096};     
    // for D10
    double speed_table1[] = {-16.108, -15.312, -13.557, -9.950, -4.227, 0, 4.944, 10.367, 13.956, 15.631, 16.348};
    double pwm_table1[] = {-120, -96, -72, -48, -24, 0, 24, 48, 72, 96, 120};

    // first and fourth argument : length of table
    car.setCalibTable(11, pwm_table0, speed_table0, 11, pwm_table1, speed_table1);

    // start 
    
    // get the information of matrix at 25cm
    ping_WalkUntil(25, FORWARD);
    identify_matrix();
    log_message = 'F';
    wait(1);
    
    // go until at the front of wall
    ping_WalkUntil(5, FORWARD);
    log_message = 'F';
    lr_turn(LEFT);
    log_message = 'L';

    // go until at the target place, then turn right + go backward
    ping_WalkUntil(66, FORWARD);
    log_message = 'F';
    lr_turn(RIGHT);
    log_message = 'R';
    ping_WalkUntil(55, BACKWARD);
    log_message = 'B';

    // go forward to identify the picture
    ping_WalkUntil(47.5, FORWARD);
    log_message = 'F';
    identify_picture();
    wait(1);

    // go to the next mission
    ping_WalkUntil(5, FORWARD);
    log_message = 'F';
    lr_turn(RIGHT);
    log_message = 'R';
    ping_WalkUntil(30, FORWARD);
    log_message = 'F';
    lr_turn(RIGHT);
    log_message = 'R';
    ping_WalkUntil(5, FORWARD);
    log_message = 'F';
    lr_turn(RIGHT);
    ping_WalkUntil(80, FORWARD);
    log_message = 'F';
    lr_turn(RIGHT);
    log_message = 'R';
    // need to be careful since the distance may be wrong due to the slope object
    ping_WalkUNtil(35, FORWARD);
    log_message = 'F';
    identify_object();

    // go to the end
    lr_turn(LEFT);
    log_message = 'L';
    ping_WalkUntil(25, FORWARD);
    log_message = 'F';
    lr_turn(LEFT);
    log_message = 'L';
    ping_WalkUntil(5, FORWARD);
    log_message = 'F';
    lr_turn(RIGHT);
    log_message = 'R';
    pint_WalkUntil(5, FORWARD);
    log_message = 'F';
    lr_turn(RIGHT);
    log_message = 'R';
    car.goStraightCalib(10);
    log_message = 'F';
    wait(10);
    car.stop();

   /*
    for (int i = 0; i < 4; i ++) {
        car.goStraightCalib(10);
        wait(4.5);
        car.servo0.set_speed_by_cm(10);
        car.servo1.set_speed_by_cm(10);
        car.controlWheel();
        wait(1.12);
    }
    car.stop();
    */
}

void ping_WalkUntil(float distance, bool Is_Forward)
{
    if (Is_forward)
        car.goStraightCalib(10);
    else 
        car.goStraightCalib(-10);

    while (1) {
        if (float(ping) <= distance)
            break;
        wait(0.1);
    };
    car.stop();
}

void lr_turn(bool IsLeft)
{
    if (IsLeft) {
        car.servo0.set_speed_by_cm(-10);
        car.servo1.set_speed_by_cm(-10);
        car.controlWheel();
        wait(1.12);
    }
    else {
        car.servo0.set_speed_by_cm(10);
        car.servo1.set_speed_by_cm(10);
        car.controlWheel();
        wait(1.12);
    }
    car.stop();
}

void log(void) {
    int count = 0;
    while (1) {
        if (!other_mission) {
            xbee.printf("%c\r\n", log_message, count++);
            wait(1.0);
        }
    }
}

void identify_matrix(void)
{
    other_mission = true;
    char s[20];
    sprintf(s, "data_matrix");
    OpenMv.puts(s);
    wait(5);
    sprintf(s, "stop");
    OpenMv.puts(s);
    wait(1);

    if (OpenMv.readable()) {
        char temp;
        char rev[20];
        int count = 0;
        
        while (1) {
            temp = OpenMv.getc();
            if (temp != '\r') {
                rev[count++] = temp;
            }
            else {
                break;
            }
        }
        for (int i = 0; i < count; i++) {
            xbee.putc(rev[i]);
        }
        xbee.printf("\r\n");
    }

    // mission complete
    for (int i = 0; i < 5; i++) {
        Led = 0;
        wait(0.1);
        Led = 1;
        wait(0.1);
    }
    other_mission = false;
}

void identify_picture(void)
{
    other_mission = true;
    
    char s[20];
    sprintf(s, "image_classification");
    OpenMv.puts(s);
    wait(5);
    
    // will get only one number
    if (OpenMv.readable()) {
        char recv = OpenMv.getc();
        xbee.putc(recv);
        xbee.printf("\r\n");
    }

    // mission complete
    for (int i = 0; i < 5; i++) {
        Led = 0;
        wait(0.1);
        Led = 1;
        wait(0.1);
    }
    other_mission = false;
}

void identify_object(void)
{
    other_mission = true;

    float value[3];
    float difference_1, difference_2;

    // turn right, for 30 deg
    car.servo0.set_speed_by_cm(10);
    car.servo1.set_speed_by_cm(10);
    car.controlWheel();
    wait(1.12 / 3);
    car.stop();
    value[0] = float(ping);
    wait(0.5);

    // turn left for 30 deg 
    car.servo0.set_speed_by_cm(-10);
    car.servo1.set_speed_by_cm(-10);
    car.controlWheel();
    wait(1.12 / 3);
    car.stop();
    value[1] = float(ping);
    wait(0.5);

    // turn left for 30 deg 
    car.servo0.set_speed_by_cm(-10);
    car.servo1.set_speed_by_cm(-10);
    car.controlWheel();
    wait(1.12 / 3);
    car.stop();
    value[2] = float(ping);
    wait(0.5);

    // turn left for 30 deg 
    car.servo0.set_speed_by_cm(-10);
    car.servo1.set_speed_by_cm(-10);
    car.controlWheel();
    wait(1.12 / 3);
    car.stop();
    value[0] = float(ping);
    wait(0.5);

    // turn right for 30 deg, back to origin
    car.servo0.set_speed_by_cm(10);
    car.servo1.set_speed_by_cm(10);
    car.controlWheel();
    wait(1.12 / 3);
    car.stop();
    
    difference_1 = value[1] - value[0];
    
    if (difference_1 <= 0)
        xbee.printf("M\r\n");
    else {
        if (difference_2 >= 0)
            xbee.printf("Right triangle\r\n");
        else {
            if (abs(difference_1) - abs(difference_2) < 2)
                xbee.printf("Square\r\n");
            else 
                xbee.printf("Equal triangle\r\n");
        }
    }

    // mission complete
    for (int i = 0; i < 5; i++) {
        Led = 0;
        wait(0.1);
        Led = 1;
        wait(0.1);
    }
    other_mission = false;
}