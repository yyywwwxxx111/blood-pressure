// team member: Wenxiang Yu(wy2128@nyu.edu) & Yi Xue(yx2589@nyu.edu)
//use i2c to communiction, SDA connects to PC_9, SCL connects to PA_8, POWER connects to 3V, GND connects to GND
#include <mbed.h>
#include "drivers/LCD_DISCO_F429ZI.h"
#define BACKGROUND 1
#define FOREGROUND 0
#define GRAPH_PADDING 5
#define SENSOR_ADDRESS 0b0011000
//MPRLS0300YG00001BB, Breakout board with 0 mmHg to 300 mmHg gage sensor, long port, with gel, I2C = 0x18, transfer function B(2.5% to 22.5% of 2^24 counts)
#define pMin 0
#define pMax 300
// 2^24 * 2.5%
#define outMin 419430
// 2^24 * 22.5%
#define outMax 3774874
#define READY_FLAG 0b1
using namespace std::chrono;
Ticker tic;
EventFlags flags;
DigitalIn button(USER_BUTTON);
DigitalInOut sdaDummy(PinName = PC_9, PinMode = PullUp);
DigitalIn sclDummy(PA_8, PullUp);
I2C Wire(PC_9, PA_8);
static constexpr uint8_t read_addr = ((SENSOR_ADDRESS << 1u) | 1U);
static constexpr uint8_t write_addr = (SENSOR_ADDRESS << 1U);
static uint8_t read_buf[4];
static const uint8_t read_command[3] = {0xAA, 0x00, 0x00};

uint8_t status = 0;
uint8_t SensorData1;
uint8_t SensorData2;
uint8_t SensorData3;
Timer timer;
Timer rater;
float Deflation[500];
float Time[500];
float slope[500];
char display_buf[3][60];
// screen
LCD_DISCO_F429ZI lcd;
// call back function
void cb()
{
	flags.set(READY_FLAG);
}
// read Raw Pressure from sensor
int readData()
{
	int raw_pressure = 0;
	status = 0;
	SensorData1 = 0;
	SensorData2 = 0;
	SensorData3 = 0;
	// step 1
	Wire.write(write_addr, (const char *)read_command, 3);
	// step 2
	wait_us(5000);
	// step 3
	Wire.read(read_addr, (char *)&read_buf, 4);
	status = (uint8_t)read_buf[0];
	//check status
	if (status & (1 << 5))
	{
		return -1; // device is busy
	}
	if (status & (0 << 6))
	{
		return -2; // device is not powered
	}
	if (status & (1 << 2))
	{
		return -3; //  integrity test failed
	}
	if (status & (1 << 0))
	{
		return -4; // Internal Math Saturation Occurred
	}
	SensorData1 = (uint8_t)read_buf[1];
	SensorData2 = (uint8_t)read_buf[2];
	SensorData3 = (uint8_t)read_buf[3];
	raw_pressure |= (SensorData1 << 16);
	raw_pressure |= (SensorData2 << 8);
	raw_pressure |= SensorData3;
	return raw_pressure;
}
//use raw data to calculate real pressure(datasheet)
float getRealPressure(int raw_pressure)
{
	float real_pressure = (((float)raw_pressure - outMin) * (pMax - pMin)) / (outMax - outMin) + pMin;
	return real_pressure;
}
//check status error
void checkStatusError(int32_t errorNumber)
{

	switch (errorNumber)
	{
	case -1:
		// device is busy
		printf("Device is busy");
		break;
	case -2:
		// device is not powered
		printf("Device is not powered");
		break;
	case -3:
		// integrity test failed
		printf("Integrity test failed");
		break;
	case -4:
		// Internal Math Saturation Occurred
		printf("Internal Math Saturation Occurred");
		break;
	default:
		break;
	}
}
//get real pressure
float getPressure()
{
	int32_t rawPressure;
	rawPressure = readData();
	if (rawPressure < 0)
	{
		checkStatusError(rawPressure);
		return -1;
	}
	return getRealPressure(rawPressure);
}
//diplay data of inflation in terminal
int inflationMeasurement()
{
	// Give user time to prepare for inflation
	lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Start Inflation in", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"5 seconds...", LEFT_MODE);
	printf("\nStart Inflation in 5 seconds");
	wait_us(1'000'000);
	lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"Start Inflation in", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"4 seconds...", LEFT_MODE);
	printf("\nStart Inflation in 4 seconds");
	wait_us(1'000'000);
	lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"Start Inflation in", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"3 seconds...", LEFT_MODE);
	printf("\nStart Inflation in 3 seconds");
	wait_us(1'000'000);
	lcd.DisplayStringAt(0, LINE(11), (uint8_t *)"Start Inflation in", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"2 seconds...", LEFT_MODE);
	printf("\nStart Inflation in 2 seconds");
	wait_us(1'000'000);
	lcd.DisplayStringAt(0, LINE(13), (uint8_t *)"Start Inflation in", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(14), (uint8_t *)"1 seconds...", LEFT_MODE);
	printf("\nStart Inflation in 1 seconds");
	wait_us(1'000'000);
	lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"Start Inflation NOW", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(16), (uint8_t *)"Pump Pressure Cuff", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(17), (uint8_t *)"Till 150 mmHg", LEFT_MODE);
	printf("\nStart Inflation NOW\n");
	printf("Start Pumping Pressure Cuff Till 150 mmHg\n");
	// while loop that keeps on measuring pressure until pressure is above 150
	while (1)
	{
		float calculatedPressure;
		calculatedPressure = getPressure();
		if (calculatedPressure < 0)
			return -1;
		if (calculatedPressure > 150)
			break;

		printf("%f\n", calculatedPressure);
		//get pressure data every 200ms
		wait_us(200'000);
	}
	return 0;
}
//display data of deflation in terminal and control the reducing rate
int deflationMeasurement(unsigned long *totalTime)
{
	// Average deflation variables
	float previousPersec;
	size_t numOfMeasurements = 0;
	float totalDifference = 0;
	float perSecDifference = 0;
	float ave_rate = 0.0;
	float calculatedPressure;

	// Give user time to prepare for deflation
	lcd.Clear(LCD_COLOR_BLUE);
	lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Start Deflation in", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"5 seconds...", LEFT_MODE);
	printf("\nStart Deflation in 5 seconds");
	wait_us(1'000'000);
	lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"Start Deflation in", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"4 seconds...", LEFT_MODE);
	printf("\nStart Deflation in 4 seconds");
	wait_us(1'000'000);
	lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Start Deflation in", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"3 seconds...", LEFT_MODE);
	printf("\nStart Deflation in 3 seconds");
	wait_us(1'000'000);
	lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"Start Deflation in", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"2 seconds...", LEFT_MODE);
	printf("\nStart Deflation in 2 seconds");
	wait_us(1'000'000);
	lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"Start Deflation in", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"1 seconds...", LEFT_MODE);
	printf("\nStart Deflation in 1 seconds");
	wait_us(1'000'000);
	lcd.DisplayStringAt(0, LINE(11), (uint8_t *)"Start Deflation NOW", LEFT_MODE);
	printf("\nStart Deflation NOW\n");

	calculatedPressure = getPressure();
	if (calculatedPressure < 0)
		return -1;
	previousPersec = calculatedPressure;

	timer.start();
	tic.attach(cb, 1000ms);
	//   while loop that keeps on measuring pressure until pressure is below 30
	int i = 0;
	while (1)
	{
		timer.reset();

		calculatedPressure = getPressure();
		if (calculatedPressure < 0)
			return -1;

		if (calculatedPressure < 30)
			break;

		numOfMeasurements++;

		printf("%f \n", calculatedPressure);
		Deflation[i++] = calculatedPressure;
		wait_us(200'000);
		Time[i] = duration_cast<milliseconds>(timer.elapsed_time()).count();
		*totalTime += Time[i];
		// every sec to check deflation rate, you can do this, bu it is not efficient, since 1s is too fast to person to fix the speed
		if (flags.get() == READY_FLAG)
		{
			perSecDifference = previousPersec - calculatedPressure;
			totalDifference += perSecDifference;
			previousPersec = calculatedPressure;
			if (perSecDifference >= 5)
			{
				lcd.Clear(LCD_COLOR_BLUE);
				lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Please slow down!", CENTER_MODE);
				//printf("\nPlease slow down!\n");
			}
			// Deflation Rate Too Slow
			else if (perSecDifference <= 3)
			{
				lcd.Clear(LCD_COLOR_BLUE);
				lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Please speed up!", CENTER_MODE);
				//printf("\nPlease speed up!\n");
			}
			else
			{
				lcd.Clear(LCD_COLOR_BLUE);
				lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Measuring...", CENTER_MODE);
				//printf("\nMeasuring...\n");
			}
			flags.clear(READY_FLAG);
		}
	}
	//And also display the reducing rate in the end
	printf("\n************\n");
	unsigned long timePerMeasurement = *totalTime / numOfMeasurements;
	printf("Average Time (ms) per Measurement is: %lu", timePerMeasurement);
	ave_rate = totalDifference / ((float)*totalTime / 1000);
	printf("\nAverage Deflation Rate is: ");
	printf("%f", ave_rate);
	printf(" mmHg/s");

	printf("\nTotal Time in (s): ");
	printf("%lu\n", *totalTime / 1000);

	// Deflation Rate Too Fast
	if (ave_rate >= 4.5)
		return -2;
	// Deflation Rate Too Slow
	if (ave_rate <= 3.5)
		return -3;

	return 0;
}
//check error when deflating
void checkDeflationError(int errorNumber)
{
	/*
	  Control deflation rate using measured data
	*/
	switch (errorNumber)
	{
	case -1:
		printf("Pressure Sensor Problem");
		lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"Pressure Sensor Probl", LEFT_MODE);
		lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"em", LEFT_MODE);
		break;
	case -2:
		printf("Deflation Rate Too Fast");
		lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"Deflation Rate Too Fa", LEFT_MODE);
		lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"st", LEFT_MODE);
		break;
	case -3:
		printf("Deflation Rate Too Slow");
		lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"Deflation Rate Too Sl", LEFT_MODE);
		lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"ow", LEFT_MODE);
		break;
	default:
		break;
	}
}
//to calculate systolic and diastolic values, we should detect and quantify pulse oscillations and find MAP
void findMap(float *map, int *mapIndex)
{
	float timeDiff = 0;
	float pressDiff = 0;
	for (int i = 1; i < 499; i++)
	{
		timeDiff = Time[i];
		pressDiff = Deflation[i] - Deflation[i - 1];
		//detect and quantify pulse oscillations
		if (timeDiff != 0)
		{
			slope[i - 1] = pressDiff / timeDiff;
		}
	}
	for (int i = 1; i < 499; i++)
	{
		if (slope[i - 1] > *map)
		{
			*map = slope[i - 1];
			*mapIndex = i;
		}
	}
}
//to calculate systolic, pressure changes systolic is close to MAP * 0.5 
float systolic(float map, int mapIndex, int *systolicIndex)
{
	float sysSlopeThreshold = 0.5 * map;
	float slopeDiff = 0.0;
	float minDiffSlope = float(65000);

	for (int i = 1; i < mapIndex - 1; i++)
	{
		if ((slope[i] > 0.0) && (slope[i] > slope[i + 1]) && (slope[i] > slope[i - 1]))
		{
			slopeDiff = abs(slope[i] - sysSlopeThreshold);
			if (slopeDiff < minDiffSlope)
			{
				minDiffSlope = slopeDiff;
				*systolicIndex = i + 1;
			}
		}
	}

	printf("\nsystolic value is %f\n", Deflation[*systolicIndex]);
	lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"systolic value is ", LEFT_MODE);
	snprintf(display_buf[0], 60, "%f", Deflation[*systolicIndex]);
	lcd.DisplayStringAt(0, LINE(7), (uint8_t *)display_buf[0], LEFT_MODE);
	return Deflation[*systolicIndex];
}
//to calculate diastolic, pressure changes of diastolic is close to MAP * 0.8 
float diastolic(float map, int mapIndex, int *diastolicIndex)
{
	float diaSlopeThreshold = 0.8 * map;
	float slopeDiff = 0.0;
	float minDiffSlope = float(65000);

	for (int i = mapIndex + 1; i < 500; i++)
	{
		if ((slope[i] > 0.0) && (slope[i] > slope[i + 1]) && (slope[i] > slope[i - 1]))
		{
			slopeDiff = abs(slope[i] - diaSlopeThreshold);
			if (slopeDiff < minDiffSlope)
			{
				minDiffSlope = slopeDiff;
				*diastolicIndex = i + 1;
			}
		}
	}
	printf("\ndiastolic value is %f\n", Deflation[*diastolicIndex]);
	lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"diastolic value is ", LEFT_MODE);
	snprintf(display_buf[1], 60, "%f", Deflation[*diastolicIndex]);
	lcd.DisplayStringAt(0, LINE(10), (uint8_t *)display_buf[1], LEFT_MODE);
	return Deflation[*diastolicIndex];
}
//to calculate heartRate, we take the data between systolic and diastolic, to count the number of beat and divide by the time between this period
float calHeartRate(int diastolicIndex, int systolicIndex)
{
	int numOfPeeks = 0;
	float heartRate = 0.0;
	unsigned long heart_time = 0;
	for (int i = systolicIndex; i < diastolicIndex - 1; i++)
	{
		if ((slope[i] > slope[i + 1]) && (slope[i - 1] < slope[i]) && (slope[i] > 0))
		{
			numOfPeeks++;
		}
		heart_time += Time[i + 1];
	}

	heartRate = ((float)numOfPeeks / ((float)heart_time / 1000)) * 60.0;
	printf("\nheartRate value is %f\n", heartRate);

	lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"heartRate value is ", LEFT_MODE);
	snprintf(display_buf[2], 60, "%f", heartRate);
	lcd.DisplayStringAt(0, LINE(13), (uint8_t *)display_buf[2], LEFT_MODE);

	return heartRate;
}
// screen start display the author
void DisplayStartPage()
{
	lcd.Clear(LCD_COLOR_BLUE);
	lcd.SetBackColor(LCD_COLOR_BLUE);
	lcd.SetTextColor(LCD_COLOR_WHITE);
	lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"WELCOME", CENTER_MODE);
	lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Author : Wenxiang Yu", CENTER_MODE);
	lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"Yi Xue", CENTER_MODE);
}
int main()
{
	DisplayStartPage();
	wait_us(2'000'000);
	lcd.Clear(LCD_COLOR_BLUE);
	lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"*********************", LEFT_MODE);
	lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"Please Press User But", CENTER_MODE);
	lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"ton To Start Program", CENTER_MODE);
	lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"*********************", LEFT_MODE);
	printf("\n************\nPlease Press User Button To Start Program\n************\n");
	unsigned long totalTime = 0;
	float map = 0.0;
	int mapIndex = 0;
	int systolicIndex = 0;
	int diastolicIndex = 0;
	while (1)
	{	//when the program starts, we should press the user_button to start inflation
		if (button)
		{
			// Start Pumping Pressure Cuff and Keep Track of inflating pressure
			inflationMeasurement();

			// Start deflating Pressure Cuff and Keep Track of deflating pressure
			int deflationStatus = deflationMeasurement(&totalTime);
			// screen to display
			lcd.Clear(LCD_COLOR_BLUE);
			lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"*********************", LEFT_MODE);

			if (deflationStatus < 0)
			{
				checkDeflationError(deflationStatus);
				printf("\nMeasurement Unsuccessful, Try Again\n");
				printf("************\n");
				lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"Measurement Unsuccess", LEFT_MODE);
				lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"ful, Try Again", LEFT_MODE);
				lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"*********************", LEFT_MODE);
				//  if Unsuccess, we do not display result
				break;
			}
			else
			{
				printf("\nSuccessful Measurement\n************\n");
				lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"Successful Measuremen", LEFT_MODE);
				lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"t", LEFT_MODE);
			}
			// calculate heartbeat diastolic Systolic
			findMap(&map, &mapIndex);
			systolic(map, mapIndex, &systolicIndex);
			diastolic(map, mapIndex, &diastolicIndex);
			calHeartRate(diastolicIndex, systolicIndex);
			printf("************\n");
			lcd.DisplayStringAt(0, LINE(14), (uint8_t *)"*********************", LEFT_MODE);
			break;
		}
	}
	return 0;
}