#include "mbed.h"
#include "EthernetInterface.h"
#include <cmath>
#include <string>
#include "MbedJSONValue.h"
#include "PCA9685.h"

#define REMOTE_PORT 5000
#define LOCAL_PORT 5001
#define BUFF_SIZE 256

#define I2C_ADDR_PCA9685   (0x00) // The i2c Address. On MIKROE-3133 this can
                                  // canbe defined with solder jumpers.
                                  // 0x00 equals to default address on the 
                                  // MIKROE-3133
                                  

#define WRITE              (0x00) // i2c Write bit
#define READ               (0x01) // i2c Read bit

PCA9685 pwm( I2C_SDA, I2C_SCL );  // I2C_SDA and I2C_SCL are defined in the 
                                  // target board definition

bool I2CScan( I2C *i2c, int Device_Adress )
{
    char ucdata_write[2];
    
    ucdata_write[0]=0;
    ucdata_write[1]=0;
    
    // try writing to the device
    if (!i2c->write((Device_Adress|WRITE), ucdata_write, 1, 0))// Check for ACK from i2c Device NB I am 'ORing' the Write Bit
        {
        // all good
        return true;
        }
    // no device found
    return false;
}


int I2CScan( I2C *i2c )
{
    short count=0;
    
    // scan all channels
    printf("====================================================\n\r");
    printf("I2CScan (ALL Channels)\n\r");
    for (int Device_Address=0; Device_Address<=0xFE; Device_Address+=2)//Stepping in 2 Because Read/Write use LSB
        {
        if( I2CScan(i2c, Device_Address) )
            {
            printf("I2CScan: %#4x\n\r", Device_Address );
            count++;
            }
        }
    printf("%d Devices detected!\n\r",count);
    printf("====================================================\n\r\n\r");
    
    return count;
}


void PCA9685_initDriver() {
    pwm.begin();
    // The formulas for the prescale and pwm freq on the PCA9685 datasheet p.25
    pwm.setPrescale(128);     // set T=20ms period for generic servos = 121
                              // Increase the 121 by 6%. The clock is 6% faster!
    //pwm.setPWMFreq(50);     // 20 ms -> 50 Hz for normal servos
    pwm.frequencyI2C(400000); //400kHz fast I2C communication
}

EthernetInterface eth;

Thread recv_thread;
Thread indicators_thread;

SocketAddress clientUDP;
UDPSocket serverUDP;

void udpReceive( void );
void indicators( void );

char in_data[BUFF_SIZE];

DigitalOut ledIndicatorRight(D8); //led indicating turn right
DigitalOut ledIndicatorLeft(D9); //led indicating turn left
DigitalOut lights(D10); //front and back lights

DigitalOut led(LED2); //onboard green led

int steering = 32000;
int speed = 32000;
int gearshift = 32000;
int indicatorLeft = 1;
int indicatorRight = 1;
int mainBeam = 1;

int servoSteer = 0;
int servoSpeed = 0;
int servoGear = 0;

int speedTemp = 0;
int isRunning = 0;



// main() runs in its own thread in the OS
int main()
{
    ledIndicatorLeft.write(1);
    ledIndicatorRight.write(1);
    I2C i2c(I2C_SDA, I2C_SCL);       //I2C Class Pin Assignments see I2C.h
    
    unsigned int i2cFrequency=400000; //400KHz for i2c Max
   
    // debug
   
    printf("\n\r ---- I2C Scanner ---- \n\r");
    
    // setup i2c
    i2c.frequency(i2cFrequency);
    printf("I2C: %uKHz\n\r", i2cFrequency/1000);
    
    // scan
    I2CScan(&i2c);
    
    // Initialize the PCA9685 on the default addr
    PCA9685_initDriver();


    //eth.set_network("IP, mask, gateway");
    eth.connect();

    SocketAddress netAddress;
    eth.get_ip_address(&netAddress);
    printf("\nIP Address: %s\n", netAddress.get_ip_address());
    printf("\nMAC address: %s\n", eth.get_mac_address());

    serverUDP.open(&eth);
    int err = serverUDP.bind(REMOTE_PORT);
    printf("Port status is: %d\n",err);
    
    recv_thread.start(udpReceive);

    printf("ready for commands:\n");

    ledIndicatorLeft.write(0);
    ledIndicatorRight.write(0);

    indicators_thread.start(indicators);
    
    while (true) {
        if (isRunning == 1){
            if (mainBeam == 0){
                lights.write(1);
            } 
            if (mainBeam == 1){
                lights.write(0);
            }
            led = !led;
            if (speedTemp == servoSpeed){
                servoSpeed = 32000;
                servoSteer = 32000;
                servoGear = 32000;
                pwm.setPWM(0,0,servoSteer);
                pwm.setPWM(1,0,servoSpeed);
                pwm.setPWM(3,0,servoGear);
            }
            speedTemp = servoSpeed;
        }
        ThisThread::sleep_for(1000ms);
    }
}

void indicators(){
    while(true){
        if (indicatorLeft == 0){
            for (int i = 0; i < 3; i=i+1){
                ledIndicatorLeft.write(1);
                ThisThread::sleep_for(1000ms);
                ledIndicatorLeft.write(0);
                ThisThread::sleep_for(1000ms);
            }
        }
        if (indicatorRight == 0){
            for (int i = 0; i < 3; i=i+1){
                ledIndicatorRight.write(1);
                ThisThread::sleep_for(1000ms);
                ledIndicatorRight.write(0);
                ThisThread::sleep_for(1000ms);
            }
        }
        ThisThread::sleep_for(500ms);
    }
}


void udpReceive()
{
    int bytes;
    MbedJSONValue jsondata;
    while(1) {
        bytes = serverUDP.recvfrom(&clientUDP, &in_data, BUFF_SIZE);
        if (isRunning == 0){
            isRunning = 1;
        }
        parse(jsondata, in_data);
        steering = jsondata["steering"].get<int>();
        speed = jsondata["speed"].get<int>();
        gearshift = jsondata["gearshift"].get<int>();
        indicatorLeft = jsondata["indicatorLeft"].get<int>();
        indicatorRight = jsondata["indicatorRight"].get<int>();
        mainBeam = jsondata["mainBeam"].get<int>();

        servoSteer = steering * -0.0043373 + 446.88;
        servoSpeed = speed * -0.0043373 + 446.88;
        servoGear = gearshift * -0.0053012 + 477.96;

        pwm.setPWM(0,0,servoSteer);
        pwm.setPWM(1,0,servoSpeed);
        pwm.setPWM(3,0,servoGear);
        }
}



