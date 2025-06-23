#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <softPwm.h>
#include <stdint.h>
#include <wiringPi.h>
#include <pthread.h> 
#include <fcntl.h>
#include <unistd.h>

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x6
#define LEDALL_ON_L 0xFA
#define PIN_ALL 16
#define PIN_BASE 300
static void myPwmWrite(struct wiringPiNodeStruct *node, int pin, int value);
static void myOnOffWrite(struct wiringPiNodeStruct *node, int pin, int value);
static int myOffRead(struct wiringPiNodeStruct *node, int pin);
static int myOnRead(struct wiringPiNodeStruct *node, int pin);
int baseReg(int pin);
double K0 = (double) 0.98; //0.98
double K1 = (double) 0.02;
int fd_mpu;
int fd_pca;
int acclX, acclY, acclZ;
int gyroX, gyroY, gyroZ;
double accl_scaled_x, accl_scaled_y, accl_scaled_z;
double gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;
double gyro_offset_x, gyro_offset_y;
double gyro_total_x, gyro_total_y;
double gyro_x_delta, gyro_y_delta;
double rotation_x, rotation_y;
double last_x, last_y;
unsigned long long timer, t;
double deltaT;
struct timeval tv, tv2;
int read_word_2c(int addr)
{
  int val;
  val = wiringPiI2CReadReg8(fd_mpu, addr);
  val = val << 8;
  val += wiringPiI2CReadReg8(fd_mpu, addr+1);
  if (val >= 0x8000)
    val = -(65536 - val);

  return val;
}
void mpu_setup(){
  fd_mpu = wiringPiI2CSetup(0x68);
  wiringPiI2CWriteReg8(fd_mpu,0x6B,0x00);
}

double dist(double a, double b)
{
  return sqrt((a*a) + (b*b));
}
double get_y_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(x, dist(y, z));
  return -(radians * (180.0 / M_PI));
}
double get_x_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(y, dist(x, z));
  return (radians * (180.0 / M_PI));
}
void read_all()
{
    acclX = read_word_2c(0x3B);
    acclY = read_word_2c(0x3D);
    acclZ = read_word_2c(0x3F);
    accl_scaled_x = acclX / 16384.0;
    accl_scaled_y = acclY / 16384.0;
    accl_scaled_z = acclZ / 16384.0;
    gyroX = read_word_2c(0x43);
    gyroY = read_word_2c(0x45);
    gyroZ = read_word_2c(0x47);
    gyro_scaled_x = gyroX / 131.0;
    gyro_scaled_y = gyroY / 131.0;
    gyro_scaled_z = gyroZ / 131.0;
}
unsigned long long  getTimestamp()
{
  gettimeofday(&tv, NULL);
  return (unsigned long long) tv.tv_sec * 1000000 + tv.tv_usec;
}

void x_y_angle(){
  t = getTimestamp();
        deltaT = (double) (t - timer)/1000000.0;
        timer = t;
        read_all();
        gyro_scaled_x -= gyro_offset_x;
        gyro_scaled_y -= gyro_offset_y;
        gyro_x_delta = (gyro_scaled_x * deltaT);
        gyro_y_delta = (gyro_scaled_y * deltaT);
        gyro_total_x += gyro_x_delta;
        gyro_total_y += gyro_y_delta;
        rotation_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
        rotation_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
        last_x = K0 * (last_x + gyro_x_delta) + (K1 * rotation_x);
        last_y = K0 * (last_y + gyro_y_delta) + (K1 * rotation_y);
        printf("%lf\t%lf\n", last_y, last_x);

}
int baseReg(int pin)
{
	return (pin >= PIN_ALL ? LEDALL_ON_L : LED0_ON_L + 4 * pin);
}
void pca9685PWMFreq(int fd, float freq)
{

	freq = (freq > 1000 ? 1000 : (freq < 40 ? 40 : freq));
	int prescale = (int)(25000000.0f / (4096 * freq) - 0.5f);
	int settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1) & 0x7F;	
	int sleep	= settings | 0x10;								
	int wake 	= settings & 0xEF;							
	int restart = wake | 0x80;										
	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, sleep);
	wiringPiI2CWriteReg8(fd, PCA9685_PRESCALE, prescale);
	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, wake);
	delay(1);
	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, restart);
}
 
int pca9685Setup(const int pinBase, const int i2cAddress, float freq)
{
	struct wiringPiNodeStruct *node = wiringPiNewNode(pinBase, PIN_ALL + 1);

	if (!node)
		return -1;

	int fd = wiringPiI2CSetupInterface("/dev/i2c-3", i2cAddress);
	//int fd = wiringPiI2CSetup(i2cAddress);
	if (fd < 0)
		return fd;

	int settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1) & 0x7F;
	int autoInc = settings | 0x20;

	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, autoInc);
	

	if (freq > 0)
		pca9685PWMFreq(fd, freq);
	

	node->fd			= fd;
	node->pwmWrite		= myPwmWrite;
	node->digitalWrite	= myOnOffWrite;
	node->digitalRead	= myOffRead;
	node->analogRead	= myOnRead;

	return fd;
}

void pca9685PWMReset(int fd)
{
	wiringPiI2CWriteReg16(fd, LEDALL_ON_L	 , 0x0);
	wiringPiI2CWriteReg16(fd, LEDALL_ON_L + 2, 0x1000);
}

void pca9685PWMWrite(int fd, int pin, int on, int off)
{
	int reg = baseReg(pin);
	wiringPiI2CWriteReg16(fd, reg	 , on  & 0x0FFF);
	wiringPiI2CWriteReg16(fd, reg + 2, off & 0x0FFF);
}

void pca9685PWMRead(int fd, int pin, int *on, int *off)
{
	int reg = baseReg(pin);
	if (on)
		*on  = wiringPiI2CReadReg16(fd, reg);
	if (off)
		*off = wiringPiI2CReadReg16(fd, reg + 2);
}

void pca9685FullOff(int fd, int pin, int tf)
{
	int reg = baseReg(pin) + 3;		
	int state = wiringPiI2CReadReg8(fd, reg);


	state = tf ? (state | 0x10) : (state & 0xEF);

	wiringPiI2CWriteReg8(fd, reg, state);
}

void pca9685FullOn(int fd, int pin, int tf)
{
	int reg = baseReg(pin) + 1;		// LEDX_ON_H
	int state = wiringPiI2CReadReg8(fd, reg);


	state = tf ? (state | 0x10) : (state & 0xEF);

	wiringPiI2CWriteReg8(fd, reg, state);

	
	if (tf)
		pca9685FullOff(fd, pin, 0);
}

static void myPwmWrite(struct wiringPiNodeStruct *node, int pin, int value)
{
	int fd   = node->fd;
	int ipin = pin - node->pinBase;

	if (value >= 4096)
		pca9685FullOn(fd, ipin, 1);
	else if (value > 0)
		pca9685PWMWrite(fd, ipin, 0, value);	
	else
		pca9685FullOff(fd, ipin, 1);
}

static void myOnOffWrite(struct wiringPiNodeStruct *node, int pin, int value)
{
	int fd   = node->fd;
	int ipin = pin - node->pinBase;

	if (value)
		pca9685FullOn(fd, ipin, 1);
	else
		pca9685FullOff(fd, ipin, 1);
}

static int myOffRead(struct wiringPiNodeStruct *node, int pin)
{
	int fd   = node->fd;
	int ipin = pin - node->pinBase;

	int off;
	pca9685PWMRead(fd, ipin, 0, &off);

	return off;
}

static int myOnRead(struct wiringPiNodeStruct *node, int pin)
{
	int fd   = node->fd;
	int ipin = pin - node->pinBase;

	int on;
	pca9685PWMRead(fd, ipin, &on, 0);

	return on;
}
void balance(int fd){
	pca9685PWMWrite(fd, 15, 0, 250);
	delay(100);
	pca9685PWMWrite(fd, 14, 0, 350);
	delay(100);
	pca9685PWMWrite(fd, 13, 0, 330);
	delay(100);
	pca9685PWMWrite(fd, 12, 0, 220);
	delay(100);
	pca9685PWMWrite(fd, 7, 0, 180);
	delay(100);
	pca9685PWMWrite(fd, 6, 0, 300); 
	delay(100);
	pca9685PWMWrite(fd, 5, 0, 400);
	delay(100);
	pca9685PWMWrite(fd, 4, 0, 280);
	delay(100);
	pca9685PWMWrite(fd, 11, 0, 400); 
	delay(100);
	pca9685PWMWrite(fd, 10, 0, 280); 
	delay(100);
	pca9685PWMWrite(fd, 9, 0, 200);
	delay(100);
	pca9685PWMWrite(fd, 8, 0, 240);
	delay(100);
}
void set_stand_pose() {
    // Hông cố định theo yêu cầu
    pca9685PWMWrite(fd_b, 12, 0, 220); // FL
    pca9685PWMWrite(fd_b, 13, 0, 350); // RL
    pca9685PWMWrite(fd_b, 14, 0, 350); // FR
    pca9685PWMWrite(fd_b, 15, 0, 220); // RR

    // Upper & Lower trung tính
    pca9685PWMWrite(fd_b, 4, 0, 320);   // FL
    pca9685PWMWrite(fd_b, 5, 0, 430);   // RL
    pca9685PWMWrite(fd_b, 6, 0, 280);   // FR
    pca9685PWMWrite(fd_b, 7, 0, 120);   // RR

    pca9685PWMWrite(fd_b, 8,  0, 280);  // FL
    pca9685PWMWrite(fd_b, 9,  0, 300);  // RL
    pca9685PWMWrite(fd_b, 10, 0, 300);  // FR
    pca9685PWMWrite(fd_b, 11, 0, 320);  // RR
}

void walk_step(int swing_leg) {
    int uppers[] = {4, 5, 6, 7};       // FL, RL, FR, RR
    int lowers[] = {8, 9, 10, 11};
    int hips[]   = {12, 13, 14, 15};


    int hip_fixed[4] = {220, 330, 350, 250};


    int upper_stand[4] = {320, 430, 280, 120};
    int lower_stand[4] = {280, 300, 300, 320};


    int upper_swing[4] = {240, 350, 360, 200};  /
    int lower_swing[4] = {200, 200, 400, 400};


    for (int i = 0; i < 4; i++) {
        pca9685PWMWrite(fd_b, hips[i], 0, hip_fixed[i]);
    }


    for (int i = 0; i < 4; i++) {
        if (i != swing_leg) {
            pca9685PWMWrite(fd_b, uppers[i], 0, upper_stand[i]);
            pca9685PWMWrite(fd_b, lowers[i], 0, lower_stand[i]);
        }
    }
    delay(200);


    pca9685PWMWrite(fd_b, lowers[swing_leg], 0, lower_swing[swing_leg]);
    delay(200);


    pca9685PWMWrite(fd_b, uppers[swing_leg], 0, upper_swing[swing_leg]);
    delay(300);


    pca9685PWMWrite(fd_b, lowers[swing_leg], 0, lower_stand[swing_leg]);
    delay(150);
}

void walk_loop() {
    int order[] = {1, 0, 3, 2}; 

    while (1) {
        for (int i = 0; i < 4; i++) {
            walk_step(order[i]);
        }
    }
}
int main()
{
    wiringPiSetup();
    mpu_setup();

  int fd = pca9685Setup(PIN_BASE, 0x40, 50);
	if (fd < 0)
	{
		printf("Error in setup\n");
		return fd;
	}
	pca9685PWMReset(fd);      
	pca9685PWMFreq(fd, 50);
    timer = getTimestamp();
    deltaT = (double) (getTimestamp() - timer)/1000000.0;
    read_all();
    last_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
    last_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
    gyro_offset_x = gyro_scaled_x;
    gyro_offset_y = gyro_scaled_y;
    gyro_total_x = last_x - gyro_offset_x;
    gyro_total_y = last_y - gyro_offset_y;

    int pulse_90 = 307;         
    int pulse_0  = 103;

    while(1) {
	x_y_angle();

	balance(fd);
    }
    return 0;
}
