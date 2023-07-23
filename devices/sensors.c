#include "sensors.h"
#include "lsm303dlhc_driver.h"
#include "vcln4000_driver.h"
#include "vector.h"
#include "adc.h"

uint32_t ProximityOffset = 0;
static uint32_t HeadingCnt = 3, ProximityCnt = 10, OtherSensorsCnt = 0, LiveCalibrationCnt = 0;
static uint32_t ProximityChange = 0, HeadingChange = 0, InclinationChange = 0, LightIntensityChange = 0;
float ProximityMaxCoverCoeff = 0.0;

static uint32_t ProximityAverageArray[15], ProximityAverageArrayCnt = 0;

uint32_t sensorsProximityGet(void);
uint32_t sensorsLightIntensityGet(void);

static vector from;

static vector a; // accelerometer readings
static vector m; // magnetometer readings
volatile vector m_max; // maximum magnetometer values, used for calibration
volatile vector m_min; // minimum magnetometer values, used for calibration

extern ColorRGB_t rgb;

AccAxesRaw_t AccRawDataMean;
MagAxesRaw_t MagRawDataMean;

AccAxesRaw_t AccRawData;
MagAxesRaw_t MagRawData;


SysSensorsData_TypeDef SysSensorsData, SysSensorsDataOld;

void LSM303DLHC_Init(void);
void LSM303DLHC_DeInit(void);
void LSM303DLHC_GetData(void);
void LSM303DLHC_6DFunctionInit(void);
void VCLN4000_Init(void);

void sensorsInit(void)
{
  from.x = 0;
  from.y = -1;
  from.z = 0;

  /*
	m_min.x = -711;
	m_min.y = -796;
	m_min.z = -1025;

	m_max.x = +698;
	m_max.y = +744;
	m_max.z = +355;
   */
  //	m_min.x = -119;
  //	m_min.y = -114;
  //	m_min.z = -156;
  //
  //	m_max.x = +116;
  //	m_max.y = +120;
  //	m_max.z = +91;

  m_min.x = 2047;
  m_min.y = 2047;
  m_min.z = 2047;

  m_max.x = -2047;
  m_max.y = -2047;
  m_max.z = -2047;

  m.x = 0;
  m.y = 0;
  m.z = 0;

  MAX9814_SHDN_ON;
  timerDelay_ms(10);
  LSM303DLHC_Init();
  timerDelay_ms(10);
  VCLN4000_Init();

  SysSensorsData.Voltage = 390;


}

void sensorsDeInit(void)
{
  MAX9814_SHDN_ON;
  LSM303DLHC_DeInit();
}


static int32_t InclinationMeanArray[4]={85,85,85,85};

void sensorsGetAll(void)
{
  static uint32_t ProximityMeanArray[PROXIMITY_MEAN_SIZE];
  static float HeadingMeanArray[HEADING_MEAN_SIZE];
  static uint32_t ProximityMean, i, ProximityAverage;


  static int32_t InclinationMean;

  if (HeadingCnt >= 2)
    {
      for (i=0; i<3; i++) InclinationMeanArray[i] = InclinationMeanArray[i+1];
      InclinationMeanArray[3] = sensorsInclinationGet(AXIS_Z) + 90;
      InclinationMean = 0;
      for (i=0; i<4; i++) InclinationMean += InclinationMeanArray[i];
      InclinationMean = InclinationMean / 4;
      SysSensorsData.Inclination = InclinationMean - 90;

      if (!InclinationChange && abs(SysSensorsData.Inclination - SysSensorsDataOld.Inclination) > 10 )
        {
          InclinationChange = 1;
        }
      SysSensorsDataOld.Inclination = SysSensorsData.Inclination;
      for (i=0; i<HEADING_MEAN_SIZE-1; i++)
        {
          HeadingMeanArray[i] = HeadingMeanArray[i+1];
        }
      HeadingMeanArray[HEADING_MEAN_SIZE-1] = (float)sensorsCompassGetHeading();

      float y_part = 0.f, x_part = 0.f;

      for (i = 0; i < HEADING_MEAN_SIZE; i++)
        {
          x_part += cos(HeadingMeanArray[i] * (M_PI / 180.f));
          y_part += sin(HeadingMeanArray[i] * (M_PI / 180.f));
        }

      SysSensorsData.Heading = (uint32_t)((atan2(y_part / HEADING_MEAN_SIZE, x_part / HEADING_MEAN_SIZE) * (180 / M_PI))+180.f);
      HeadingCnt=0;

      if (!HeadingChange && abs(((int32_t)SysSensorsData.Heading) - ((int32_t)SysSensorsDataOld.Heading)) > 30 )
        {
          HeadingChange = 1;
        }
      SysSensorsDataOld.Heading = SysSensorsData.Heading;
    }
  HeadingCnt++;

  if (ProximityCnt >= 2)
    {
      for (i=0; i<PROXIMITY_MEAN_SIZE - 1; i++) {ProximityMeanArray[i] = ProximityMeanArray[i+1];}
      ProximityMeanArray[PROXIMITY_MEAN_SIZE - 1] = sensorsProximityGet();
      ProximityMean = 0;

      for (i=0; i<PROXIMITY_MEAN_SIZE; i++) {ProximityMean = ProximityMean + ProximityMeanArray[i];}

      ProximityMean = ProximityMean / PROXIMITY_MEAN_SIZE;


      SysSensorsData.Proximity = ProximityMean;

      if (!ProximityChange && abs(((int32_t)SysSensorsData.Proximity) - ((int32_t)SysSensorsDataOld.Proximity)) > 30)
        {
          ProximityChange = 1;
        }
      SysSensorsDataOld.Proximity = SysSensorsData.Proximity;
      SysSensorsData.CoverPercents = sensorsProximityGetCover();
      ProximityCnt=0;


      //		ProximityAverageArray[ProximityAverageArrayCnt] = SysSensorsData.Proximity;
      //		ProximityAverageArrayCnt++;
    }
  ProximityCnt++;

  if (OtherSensorsCnt >= 3)
    {
      //printf("P=%d, %d\r\n", ProximityOffset, ProximityMean);
      //printf("%% = %d\r\n", SysSensorsData.CoverPercents);
      //outputsRGBLedSet(RGB_ALL, 100, 0, 0);
      //		SysSensorsData.LightIntensity = sensorsLightIntensityGet();
      //
      //		if (!LightIntensityChange && abs(((int32_t)SysSensorsData.LightIntensity) - ((int32_t)SysSensorsDataOld.LightIntensity)) > 200)
      //		{
      //			LightIntensityChange = 1;
      //		}
      //		SysSensorsDataOld.LightIntensity = SysSensorsData.LightIntensity;

      if (!GetADCState())
        {
          ADC_On();
          SysSensorsData.Voltage = (sensorsSystemGetVoltage())/10;
          SysSensorsData.Temperature = sensorsTemperatureGet();
          ADC_Off();
        }
      else
        {
          SysSensorsData.Voltage = (sensorsSystemGetVoltage())/10;
          SysSensorsData.Temperature = sensorsTemperatureGet();
        }

      //outputsRGBLedSet(RGB_ALL, 0, 0, 0);

      //printf("T=%d\r\n", SysSensorsData.Temperature);
      OtherSensorsCnt = 0;
    }
  OtherSensorsCnt++;

  //printf("H = %f\r\n", heading());
  //printf("P = %u\r\n", vcln4000_driverPerformProximityMeasurment());
  //printf("L = %u\r\n", vcln4000_driverPerformLightMeasurment());
  //printf("I = %d\r\n", sensorsInclinationGet(AXIS_Z));
  //printf("P = %u\r\n", sensorsProximityGetCover());


  //	if (LiveCalibrationCnt >= 17)
  //	{
  //		if (ProximityChange == 0 && HeadingChange == 0 && InclinationChange == 0 /*&& LightIntensityChange == 0 */)
  //		{
  //			ProximityAverage = 0;
  //			for (i=1; i<ProximityAverageArrayCnt-1; i++) ProximityAverage += ProximityAverageArray[i];
  //			ProximityOffset = ProximityAverage / (ProximityAverageArrayCnt-2);
  //
  //			//ProximityOffset+=5;
  //
  //			outputsFadeIn(0, RGB_LED2, RGB_LED3, 2);
  //			outputs IRLedCommand(0x40, 0x58);
  //			outputsFadeOut(0, RGB_LED2, RGB_LED3, 2);
  //		}
  //		else
  //		{
  //		ProximityChange = 0;
  //		HeadingChange = 0;
  //		InclinationChange = 0;
  //		LightIntensityChange = 0;
  //		}
  //
  //		ProximityAverageArrayCnt=0;
  //		LiveCalibrationCnt=0;
  //	}
  //	LiveCalibrationCnt++;

}


uint32_t sensorsSystemGetVoltage(void)
{
  uint32_t AdcMVoltCoef = 0, adc = 0;

  adc=Read_ADC1(ADC_Channel_Vrefint);
  AdcMVoltCoef = 1200000 / adc;
  adc=Read_ADC1(ADC_Channel_9);

  return ((adc * AdcMVoltCoef)/1000)*2;
}


uint32_t sensorsGetAudioSample(void)
{
  /* ADC freq = 9000000 MHz	*/
  /* ADC SampleTime = 239.5 cycles	*/
  /* Resulting freq = 9000000 MHz / 239.5 = 37578 Hz	*/
  /* To get more lower freq we perform double ADC conversion with averaging	*/
  /* So we have: 37578 Hz / 2 = 18789 Hz	*/
  uint32_t sample;

  sample = Read_ADC1(1);

  sample += Read_ADC1(1);
  return sample / 2;
}



uint8_t sensorsGet6DPosition(void)
{
  POSITION_6D_t position;
  if (Get6DPosition(&position) == MEMS_SUCCESS) return position;
  else return MEMS_ERROR;
}


void LSM303DLHC_Init(void)
{
  uint8_t response;
  //set ODR_ACCELEROMETER (turn ON device)
  response = SetODR(ODR_50Hz);

  //set ODR_MAGNETOMETER (turn ON device)
  response = SetODR_M(ODR_30Hz_M);

  //set PowerMode
  response = SetMode(NORMAL);

  //set MagnetometerMode
  response = SetModeMag(CONTINUOUS_MODE);

  //set Fullscale
  response = SetFullScale(FULLSCALE_2);

  //set Magnetometer Gain
  response = SetGainMag(GAIN_1100_M);

  //set axis Enable
  response = SetAxis(X_ENABLE | Y_ENABLE | Z_ENABLE);

  response = SetTemperature(MEMS_ENABLE);
}

void sensorsMotionDetectInit(void)
{
  timerDelay_ms(1);
  SetHPFMode(HPM_NORMAL_MODE_RES);
  timerDelay_ms(1);
  HPFAOI1Enable(MEMS_ENABLE);
  timerDelay_ms(1);
  SetInt1Pin(CLICK_ON_PIN_INT1_DISABLE | I1_INT1_ON_PIN_INT1_ENABLE |
      I1_INT2_ON_PIN_INT1_DISABLE | I1_DRDY1_ON_INT1_DISABLE | I1_DRDY2_ON_INT1_DISABLE     |
      WTM_ON_INT1_DISABLE | INT1_OVERRUN_DISABLE   ) ;

  timerDelay_ms(1);
  Int1LatchEnable(ENABLE);
  timerDelay_ms(1);
  SetInt1Threshold(2);
  timerDelay_ms(1);
  SetInt1Duration(0);
  timerDelay_ms(1);
  DummyReadReferenceReg();


  SetInt1Configuration(INT_ZHIE_ENABLE | INT_ZLIE_DISABLE |
      INT_YHIE_ENABLE | INT_YLIE_DISABLE |
      INT_XHIE_ENABLE | INT_XLIE_DISABLE );
  //SetInt6D4DConfiguration(INT1_6D_ENABLE);
  timerDelay_ms(1);
  //SetIntMode(INT_MODE_OR); //INT_MODE_OR
  //timerDelay_ms(1);
}

void LSM303DLHC_DeInit(void)
{
  //set PowerMode
  SetMode(POWER_DOWN);
}

void VCLN4000_Init(void)
{
  vcln4000_driverInit();
}

void sensorsCompassCalibration(void)
{
  uint32_t i=0;

  m_min.x = 2047;
  m_min.y = 2047;
  m_min.z = 2047;

  m_max.x = -2047;
  m_max.y = -2047;
  m_max.z = -2047;

  rgb.r = 70;
  rgb.g = 0;
  rgb.b = 0;

  for (i=0; i<70; i++)
    {
      outputsRGBLedSet(RGB_LED2, rgb.r, rgb.g, rgb.b);
      outputsRGBLedSet(RGB_LED3, rgb.r, rgb.g, rgb.b);
      if (sensorsCompassCalibrate())
        {
          outputsRGBLedSet(RGB_LED2, 50, 50, 50);
          outputsRGBLedSet(RGB_LED3, 50, 50, 50);
          timerDelay_ms(20);
          outputsRGBLedSet(RGB_LED2, rgb.r, rgb.g, rgb.b);
          outputsRGBLedSet(RGB_LED3, rgb.r, rgb.g, rgb.b);
        }
      timerDelay_ms(50);

      if (sensorsCompassCalibrate())
        {
          outputsRGBLedSet(RGB_LED2, 50, 50, 50);
          outputsRGBLedSet(RGB_LED3, 50, 50, 50);
          timerDelay_ms(20);
          outputsRGBLedSet(RGB_LED2, rgb.r, rgb.g, rgb.b);
          outputsRGBLedSet(RGB_LED3, rgb.r, rgb.g, rgb.b);
        }
      timerDelay_ms(50);

      if (sensorsCompassCalibrate())
        {
          outputsRGBLedSet(RGB_LED2, 50, 50, 50);
          outputsRGBLedSet(RGB_LED3, 50, 50, 50);
          timerDelay_ms(20);
          outputsRGBLedSet(RGB_LED2, rgb.r, rgb.g, rgb.b);
          outputsRGBLedSet(RGB_LED3, rgb.r, rgb.g, rgb.b);
        }
      timerDelay_ms(50);

      rgb.r--;
      rgb.g++;
    }

  for (i=0; i<70; i++)
    {
      timerDelay_ms(4);
      rgb.g--;
      outputsRGBLedSet(RGB_ALL, rgb.r, rgb.g, rgb.b);
    }

}

void sensorsProximityCalibration(void)
{
  int32_t counts = 0, counts_avr = 0;

  int i=0;
  ProximityOffset = 0;

  for (i=0; i<25; i++)
    {
      ProximityOffset += sensorsProximityGet();
      outputsRGBLedSet(RGB_LED2, i*10, i*10, i*10);
      outputsRGBLedSet(RGB_LED3, i*10, i*10, i*10);
      timerDelay_ms(25);
    }
  ProximityOffset = ProximityOffset / 25;

  //printf("ProximityOffset = %d\r\n", ProximityOffset);

  while (counts_avr < PROXIMITY_TOUCH_THRESHOLD_COUNTS)
    {
      timerDelay_ms(100);
      counts_avr = 0;
      for (i=0; i<3; i++)
        {
          counts = sensorsProximityGet();
          timerDelay_ms(10);
          counts = counts - ProximityOffset;
          if (counts < 0) counts=0;

          counts_avr += counts;
        }
      counts_avr = counts_avr / 3;
    };

  timerDelay_ms(1000);


  counts_avr = 0;
  for (i=25; i>0; i--)
    {
      timerDelay_ms(100);
      outputsRGBLedSet(RGB_LED2, i*10, i*10, i*10);
      outputsRGBLedSet(RGB_LED3, i*10, i*10, i*10);
      counts = 0;
      counts = sensorsProximityGet();
      if (counts > ProximityOffset) counts = counts - ProximityOffset;
      if (counts < 0) counts=0;
      //printf("Counts #%d = %d\r\n", i, counts);
      counts_avr += counts;
    }

  counts_avr = counts_avr / 25;
  //printf("Counts_avr = %d\r\n", counts_avr);

  ProximityMaxCoverCoeff = (float)counts_avr / (float)100.0;

  //printf("ProximityMaxCoverCoeff = %f\r\n", ProximityMaxCoverCoeff);
}

uint32_t sensorsProximityGetCover(void)
{
  int32_t counts = 0;

  uint32_t percents = 0;

  counts = SysSensorsData.Proximity;
  counts = counts - ProximityOffset;
  if (counts < 0) counts=0;

  percents = (uint32_t)(((float)counts) / ProximityMaxCoverCoeff);
  if (percents >= 100) percents = 100;
  return percents;
}

uint32_t sensorsProximityGet(void)
{
  uint32_t i=0, proxy_val=0;


  for (i=0; i<PROXIMITY_RAW_DATA_MEAN_SIZE; i++)
    {
      proxy_val += vcln4000_driverPerformProximityMeasurment();
    }

  return proxy_val / PROXIMITY_RAW_DATA_MEAN_SIZE;
}

uint32_t sensorsLightIntensityGet(void)
{
  uint32_t LightMeasurmentResult = 0;


  LightMeasurmentResult = vcln4000_driverGetLightMeasurmentResult();
  vcln4000_driverStartLightMeasurment();

  return LightMeasurmentResult;

  //	return vcln4000_driverPerformLightMeasurment();
}


// Returns the number of degrees from the From vector projected into
// the horizontal plane is away from north.
//
// Description of heading algorithm:
// Shift and scale the magnetic reading based on calibration data to
// to find the North vector. Use the acceleration readings to
// determine the Down vector. The cross product of North and Down
// vectors is East. The vectors East and North form a basis for the
// horizontal plane. The From vector is projected into the horizontal
// plane and the angle between the projected vector and north is
// returned.
uint16_t sensorsCompassGetHeading(void)
{
  // shift and scale
  m.x = (m.x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
  m.y = (m.y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
  m.z = (m.z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;

  vector temp_a = a;
  // normalize
  vector_normalize(&temp_a);

  // compute E and N
  vector E;
  vector N;
  vector_cross(&m, &temp_a, &E);
  vector_normalize(&E);
  vector_cross(&temp_a, &E, &N);

  // compute heading
  int heading = 0;
  heading = round(atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI);
  if (heading < 0) heading += 360;
  return heading;
}

uint32_t sensorsCompassCalibrate(void)
{
  uint32_t change = 0;
  LSM303DLHC_GetData();
  /*
	m.x++;
	m.y++;
	m.z++;
   */
  if (m.x > m_max.x) {m_max.x = m.x; change=1;}
  if (m.y > m_max.y) {m_max.y = m.y; change=1;}
  if (m.z > m_max.z) {m_max.z = m.z; change=1;}

  if (m.x < m_min.x) {m_min.x = m.x; change=1;}
  if (m.y < m_min.y) {m_min.y = m.y; change=1;}
  if (m.z < m_min.z) {m_min.z = m.z; change=1;}

  /*
	if (change == 1)
	{
		printf("MIN X = %.2f, Y = %.2f, Z = %.2f\n\r", m_min.x, m_min.y, m_min.z);
		printf("MAX X = %.2f, Y = %.2f, Z = %.2f\n\r", m_max.x, m_max.y, m_max.z);
	}
   */

  return change;
}

void LSM303DLHC_GetData(void)
{
  /*
	uint32_t i=0;

	AccRawData.AXIS_X = 0;
	AccRawData.AXIS_Y = 0;
	AccRawData.AXIS_Z = 0;

	MagRawData.AXIS_X = 0;
	MagRawData.AXIS_Y = 0;
	MagRawData.AXIS_Z = 0;

	for (i=0; i<ACC_MAG_RAW_DATA_MEAN_SIZE; i++)
	{
		GetAccAxesRaw(&AccRawDataMean);
		AccRawData.AXIS_X += AccRawDataMean.AXIS_X;
		AccRawData.AXIS_Y += AccRawDataMean.AXIS_Y;
		AccRawData.AXIS_Z += AccRawDataMean.AXIS_Z;

		GetMagAxesRaw(&MagRawDataMean);
		MagRawData.AXIS_X += MagRawDataMean.AXIS_X;
		MagRawData.AXIS_Y += MagRawDataMean.AXIS_Y;
		MagRawData.AXIS_Z += MagRawDataMean.AXIS_Z;
	}


	AccRawData.AXIS_X /= ACC_MAG_RAW_DATA_MEAN_SIZE;
	AccRawData.AXIS_Y /= ACC_MAG_RAW_DATA_MEAN_SIZE;
	AccRawData.AXIS_Z /= ACC_MAG_RAW_DATA_MEAN_SIZE;

	MagRawData.AXIS_X /= ACC_MAG_RAW_DATA_MEAN_SIZE;
	MagRawData.AXIS_Y /= ACC_MAG_RAW_DATA_MEAN_SIZE;
	MagRawData.AXIS_Z /= ACC_MAG_RAW_DATA_MEAN_SIZE;
   */

  GetAccAxesRaw(&AccRawData);


  GetMagAxesRaw(&MagRawData);

  a.x = AccRawData.AXIS_X;
  a.y = AccRawData.AXIS_Y;
  a.z = AccRawData.AXIS_Z;

  //	printf("X = %.2f, Y = %.2f, Z = %.2f\n\r", a.x, a.y, a.z);

  m.x = MagRawData.AXIS_X;
  m.y = MagRawData.AXIS_Y;
  m.z = MagRawData.AXIS_Z;
  //	printf("X = %d, Y = %d, Z = %d\n\r", MagRawData.AXIS_X, MagRawData.AXIS_Y, MagRawData.AXIS_Z);
  //	printf("X = %.2f, Y = %.2f, Z = %.2f\n\r", m.x, m.y, m.z);
}

int32_t sensorsInclinationGet(AXIS_t axis)
{

  LSM303DLHC_GetData();
  switch (axis)
  {
  case AXIS_X: return (int32_t)(a.x / 12.0);
  case AXIS_Y: return (int32_t)(a.y / 12.0);
  case AXIS_Z: return (int32_t)(a.z / 12.0);
  default:
    break;
  }
  return 0;
}

int32_t sensorsTemperatureGet(void)
{
  int16_t temp = 0;
  GetTempRaw(&temp);
  return temp;
}

uint32_t sensorsCheckOverturnChange(POSITION_t *previousPosition)
{
  POSITION_t currentPosition;
  uint32_t positionChanged;

  if (SysSensorsData.Inclination <= 0) currentPosition = POSITION_DOWN;
  else currentPosition = POSITION_UP;

  if (currentPosition != *previousPosition) positionChanged = 1;
  else positionChanged = 0;

  *previousPosition = currentPosition;

  return positionChanged;
}

