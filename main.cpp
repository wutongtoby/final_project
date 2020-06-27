#include "mbed.h"
#include "bbcar.h"
#include "bbcar_rpc.h"

Ticker servo_ticker;
PwmOut pin13(D13), pin10(D10);
Serial xbee(D1, D0); // Tx, Rx
BBCar car(pin13, pin10, servo_ticker);
Ticker encoder_ticker1;
Ticker encoder_ticker2;
DigitalInOut pin6(D6);
parallax_ping ping(pin6);

//DigitalIn pin3(D3);
//DigitalIn pin4(D4);

// just wait for rpc call from the xbee!
int main(void) 
{
    
    // for D13 
    double pwm_table0[] = {-50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50};
    double speed_table0[] = {-20.177, -16.189, -11.484, -5.742, -0.799, 0, 4.785, 10.288, 15.551, 20.974, 25.440};     
    // for D10
    double pwm_table1[] = {-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150};
    double speed_table1[] = {-16.928, -15.950, -15.072, -12.281, -6.061, 0, 5.822, 11.723, 14.913, 16.02, 16.428};

    // first and fourth argument : length of table
    car.setCalibTable(11, pwm_table0, speed_table0, 11, pwm_table1, speed_table1);

    // let both servo go straight with the speed, 
    
    
    car.goStraightCalib(10);
    while (1) {
        if (float(ping) <= 15)
            break;
        wait(0.1);
    };
    
    car.stop();
}
