#include <Arduino.h>
#include <Wire.h>

int16_t raw_x = 0, raw_y = 0, raw_z = 0; // for reading raw data
float angle_gx = 0, angle_gy = 0;		 // extracted angle from gyro data by actan function
float angle_ax = 0, angle_ay = 0;		 // extracted angle from accl data by integration

// variables for gyro data
float offset_gx = 0, offset_gy = 0, offset_gz = 0; // for calibration
int cali_count = 0;								   // for loop counter
float gx = 0, gy = 0, gz = 0;					   // for final data

// variables for accl data
// no need to calibrate
float ax = 0, ay = 0, az = 0; // for final data

// mpu functions
void mpu_init(void);
bool mpu_connected(void);
void mpu_cali(void);
void mpu_readData(void);
void mpu_init(void);

void setup()
{
	Serial.begin(115200);
	Serial.println("Connecting.............");
	do
	{
		mpu_connect();
		delay(250);
	} while (!mpu_connected());

	mpu_init();
	Serial.println("Connected.............");

	Serial.println("Calibrating.............");
	mpu_cali();
	Serial.println("Calibrated.............");
}

void loop()
{
	mpu_readData();

	Serial.print(angle_gx);
	Serial.print(", ");
	Serial.println(angle_gy);

	delay(50);
}

void mpu_connect(void)
{

	// start comm with mpu
	Wire.begin();
	Wire.setClock(400000);
	Wire.beginTransmission(0x68);

	// power management register
	Wire.write(0x6B);

	// set to 0 to wake up the mpu
	Wire.write(0x00);
	Wire.endTransmission(true);
}

bool mpu_connected(void)
{
	// start comm with mpu
	Wire.beginTransmission(0x68);

	// power management register
	Wire.write(0x75);
	Wire.endTransmission();

	// requesting 1 byte of data
	Wire.requestFrom(0x68, 1);

	// reading data and returning bool
	return (Wire.read() == 0x68);
	// return true;
}

void mpu_init()
{
	// filter
	Wire.beginTransmission(0x68);
	Wire.write(0x1A); // turn on low-pass filter (works for both gyro and accl)
	Wire.write(0x05); // set low-pass filter to 10Hz / DLPF value 5
	Wire.endTransmission();

	// configuring accl output
	Wire.beginTransmission(0x68);
	Wire.write(0x1C); // accl configuration are stored in 0x1C
	Wire.write(0x10); // selectiong a full scale range of +/- 8g
	Wire.endTransmission();

	// configuring gyro output
	Wire.beginTransmission(0x68);
	Wire.write(0x1B); // gyro configuration are stored in 0x1B
	Wire.write(0x08); // set sensitivity scale factor to 65.5 LSB/deg/s
	Wire.endTransmission();
}

void mpu_cali(void)
{
	// calibrating
	for (cali_count = 0; cali_count < 2000; cali_count++)
	{
		// finally reading gyro data
		Wire.beginTransmission(0x68);
		Wire.write(0x43); // start reading from register 0x43
		Wire.endTransmission();

		Wire.requestFrom(0x68, 6); // requesting 6 bytes of data

		raw_x = Wire.read() << 8 | Wire.read();
		raw_y = Wire.read() << 8 | Wire.read();
		raw_z = Wire.read() << 8 | Wire.read();

		// converting raw gyro data to deg/s
		gx = (float)raw_x / 65.5;
		gy = (float)raw_y / 65.5;
		gz = (float)raw_z / 65.5;

		// summing up data
		offset_gx += gx;
		offset_gy += gy;
		offset_gz += gz;

		delay(3);
	}

	offset_gx /= 2000;
	offset_gy /= 2000;
	offset_gz /= 2000;
}

void mpu_readData(void)
{

	// finally reading gyro data
	Wire.beginTransmission(0x68);
	Wire.write(0x43); // start reading from register 0x43
	Wire.endTransmission();

	Wire.requestFrom(0x68, 6); // requesting 6 bytes of data

	raw_x = Wire.read() << 8 | Wire.read();
	raw_y = Wire.read() << 8 | Wire.read();
	raw_z = Wire.read() << 8 | Wire.read();

	// converting raw gyro data to deg/s
	gx = (float)raw_x / 65.5;
	gy = (float)raw_y / 65.5;
	gz = (float)raw_z / 65.5;

	// compensating for offsets
	gx -= offset_gx;
	gy -= offset_gy;
	gz -= offset_gz;

	// finally reading accl data
	Wire.beginTransmission(0x68);
	Wire.write(0x3B); // start reading from register 0x3B
	Wire.endTransmission();

	Wire.requestFrom(0x68, 6); // requesting 6 bytes of data

	raw_x = Wire.read() << 8 | Wire.read();
	raw_y = Wire.read() << 8 | Wire.read();
	raw_z = Wire.read() << 8 | Wire.read();

	// converting raw accl data to g
	ax = (float)raw_x / 4096 - 0.03;
	ay = (float)raw_y / 4096 - 0.025;
	az = (float)raw_z / 4096 + 0.09;

	// getting angles
	// integrating gyro rate to get angle
	angle_gx += gx * 0.05;
	angle_gy += gy * 0.05;

	angle_gx -= angle_gy * sin(gz * (0.05 * 0.0174533));
	angle_gy += angle_gx * sin(gz * (0.05 * 0.0174533));

	// extracting angle from accl data
	angle_ax = atan(ay / sqrt(ax * ax + az * az)) * 57.29577951;
	angle_ay = atan(ax / sqrt(ay * ay + az * az)) * 57.29577951;
}
