#pragma once

#include "bme280.h"
#include "bme280_selftest.h"
#include <algorithm>
#include <vector>
#include <math.h>
#include <Wire.h>
//#include "HelperFunctions.h" // misc functions called to help out

#define SAMPLE_SIZE 25 
//#define USING_STANDARD_DEVIATION  true

#ifdef USING_STANDARD_DEVIATION
  #define STANDARD_DEVIATION_FILTER 2.0  // we will toss any value that isnt within this number of standard deviations of the past BUFFER_SIZE readings
#else
  #define RANGE_BASED_FILTER 10.0  // total band 
#endif

const int LEDPIN2 = 16;
//struct bme280_dev dev; // I2C
//bme280 bmesensors(&oneWire); // I2C

namespace BME280_Sensors {

class BME_Sensors  
{  
  public:
  struct bme280_dev dev;
  char sensorName[20];
  uint8_t deviceaddress[8];
  int  DeviceNum;
  double hiTemp;
  double DHhiTemp;
  double DHhiPressure;
  double DHhiHumidity;
  double loTemp;
  double DHloTemp;
  double DHloPressure;
  double DHloHumidity;
  double temp;
  double celsius;
  double fahrenheit;
  double avgTemp;
  double energy;
  double Pressure;
  double Mb;
  double Amb_sealevel;
  double humidity;
  double Reletivehumidity;
  double DewPoint;
  double WetBulb;
  double Altitude;
  double Altitude2;
  double SeaLevelPressure;
  int bstart_check;
  std::vector<double> hiArray;
  std::vector<double> loArray;
  bool addHiTemp(double newVal);
  bool addLoTemp(double newVal);
  double getTemp();
  double getAvgTemp();
  void setAvgTemp(double num);
  BME_Sensors();
  ~BME_Sensors();
  void PrintBmeSettings(struct bme280_dev *dev);
  void print_sensor_data(struct bme280_data *comp_data);
  int8_t SetSensorsForcedSettings(struct bme280_dev *dev);
  int8_t SetSensorsNormalSettings(struct bme280_dev *dev);
  int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev);
  int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev);
  void updateDailyTempHiandLow();
  void updateDailyPressureHiandLow();
  void updateDailyHumidityHiandLow();
  float dew_point_calc(float in_temp, float in_humid);
  float wetbulb_calc(float in_temp, float in_humid);
  float relitive_humidity_calc(float in_temp, float in_wbtemp, float P);
  double altitude(double P, double P0);
  double sealevel(double A, double P, float T);
  float readFloatAltitudeFeet( void );
  float readFloatAltitudeMeters( void );
  int UpdateRedings(struct bme280_dev *dev);
};

void BME_Sensors::setAvgTemp(double num)
{
  avgTemp = num;
}

double BME_Sensors::getAvgTemp()
{
 double _temp =  avgTemp;
 return _temp; 
}
 
  bool BME_Sensors::addHiTemp(double newVal)
  {
    if (hiArray.size() < 10)
    {
      hiArray.push_back(newVal);
      return true;
    }
    double sum = std::accumulate(hiArray.begin(), hiArray.end(), 0.0);
    double mean = sum / hiArray.size();
    
#ifdef USING_STANDARD_DEVIATION
    std::vector<double> delta(hiArray.size());
    std::transform(hiArray.begin(), hiArray.end(), delta.begin(), [mean](double x) {
      return x - mean;
    });
    double squareSum = std::inner_product(delta.begin(), delta.end(), delta.begin(), 0.0);
    double stdDeviation = std::sqrt(squareSum / hiArray.size());
    if (std::abs(newVal - mean) > STANDARD_DEVIATION_FILTER * stdDeviation)
    {
      return false;
    }
#else
    if (std::abs(newVal - mean) > RANGE_BASED_FILTER / 2.0)
    {
        return false;
    }
#endif

    hiArray.insert(hiArray.begin(), newVal);
    if (hiArray.size() >= SAMPLE_SIZE)
    {
      hiArray.pop_back();
    }
    return true;
  }
  bool BME_Sensors::addLoTemp(double newVal)
  {
    if (loArray.size() < 10)
    {
      loArray.push_back(newVal);
      return true;
    }
    double sum = std::accumulate(loArray.begin(), loArray.end(), 0.0);
    double mean = sum / loArray.size();
    
#ifdef USING_STANDARD_DEVIATION
    std::vector<double> delta(loArray.size());
    std::transform(loArray.begin(), loArray.end(), delta.begin(), [mean](double x) {
      return x - mean;
    });
    double squareSum = std::inner_product(delta.begin(), delta.end(), delta.begin(), 0.0);
    double stdDeviation = std::sqrt(squareSum / loArray.size());
    if (std::abs(newVal - mean) > STANDARD_DEVIATION_FILTER * stdDeviation)
    {
        return false;
    }
#else
    if (std::abs(newVal - mean) > RANGE_BASED_FILTER / 2.0)
    {
        return false;
    }
#endif

    loArray.insert(loArray.begin(), newVal);
    if (loArray.size() >= SAMPLE_SIZE)
    {
      loArray.pop_back();
    }
    return true;
  }


 BME_Sensors::BME_Sensors()
{
  int8_t rslt = 0;
  hiTemp = 0;
  loTemp = 0;
  temp = 0;
  energy = 0;
  celsius = -20;
  fahrenheit = -20;
  avgTemp = -20;
  Pressure = 0;
  Mb = 22;
  humidity = 0;
  bstart_check = 0;
  bme280_init(&dev); 
  //return rslt;
}

BME_Sensors::~BME_Sensors(){
}

int8_t BME_Sensors::SetSensorsForcedSettings (struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, dev);
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);

    return rslt;

}

int8_t BME_Sensors::SetSensorsNormalSettings (struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;
    /* Recommended mode of operation: Indoor navigation */
  dev->settings.osr_h = BME280_OVERSAMPLING_1X;
  dev->settings.osr_p = BME280_OVERSAMPLING_16X;
  dev->settings.osr_t = BME280_OVERSAMPLING_2X;
  dev->settings.filter = BME280_FILTER_COEFF_16;
  dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

  settings_sel = BME280_OSR_PRESS_SEL;
  settings_sel |= BME280_OSR_TEMP_SEL;
  settings_sel |= BME280_OSR_HUM_SEL;
  settings_sel |= BME280_STANDBY_SEL;
  settings_sel |= BME280_FILTER_SEL;

  
  
  rslt = bme280_set_sensor_settings(settings_sel, dev);
  rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

  return rslt;
}
    
int8_t BME_Sensors::stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
     int8_t rslt;
     struct bme280_data comp_data;
     
     
    /* Wait for the measurement to complete and print data @25Hz */
        
    //dev->delay_ms(4000);
        
    Serial.println(rslt);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
    Serial.println(rslt);
    rslt = bme280_crc_selftest(dev);
    //Serial.println(rslt);
    if (rslt == 0){
    print_sensor_data(&comp_data);
    }else{
      Serial.println("CrC error in data, result is: ");
      print_sensor_data(&comp_data);
      Serial.print("Result is :");
      Serial.println(rslt);
    }
       
    return rslt;
}

int8_t BME_Sensors::stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
  int8_t rslt;
  uint8_t data_sel;
  struct bme280_data comp_data;

  data_sel = BME280_PRESS;
  data_sel |= BME280_TEMP;
  data_sel |= BME280_HUM;
  
  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
  
    /* Delay while the sensor completes a measurement */
    //dev->delay_ms(1000);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
    //Serial.println(rslt);
    //Serial.print("t_fine: ");
    //Serial.println(dev->calib_data.t_fine);
    rslt = bme280_crc_selftest(dev);
    rslt = 0;
    //Serial.println(rslt);
    if (rslt == 0){
    print_sensor_data(&comp_data);
    celsius = comp_data.temperature;
    fahrenheit = (comp_data.temperature * 9) / 5 + 32;
    Pressure = comp_data.pressure;
    Mb = ((comp_data.pressure / 100) / 33.8638866667) + 4.5619;
    Amb_sealevel = sealevel(1394, (comp_data.pressure * 0.01), comp_data.temperature);
    readFloatAltitudeMeters();
    Altitude2 = (comp_data.pressure, Amb_sealevel); // this should be adjusted to your local forcase
    humidity = comp_data.humidity;
    WetBulb = wetbulb_calc(comp_data.temperature, comp_data.humidity);
    DewPoint = dew_point_calc(comp_data.temperature, comp_data.humidity);
    Reletivehumidity = relitive_humidity_calc(comp_data.temperature, comp_data.humidity, (comp_data.pressure / 100));
    

    }else{
      Serial.print("CrC error in data, result is: ");
      Serial.println(rslt);
      Serial.println("Data Collected is ");
      print_sensor_data(&comp_data);
      
      celsius = NAN;
      fahrenheit = NAN;
      Pressure = NAN;
      humidity = NAN;
      Mb = NAN;
      Amb_sealevel = NAN;
      Altitude = NAN;
      Altitude2 = NAN;
      WetBulb = NAN;
      Reletivehumidity = NAN;
      DewPoint = NAN;
      
    }

  return rslt;
}

void BME_Sensors::print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
        //printf("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
        Serial.print(comp_data->temperature);
        Serial.print(", ");
        Serial.print(comp_data->pressure);
        Serial.print(", ");
        Serial.println(comp_data->humidity);
#else
        printf("%ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

void BME_Sensors::PrintBmeSettings(struct bme280_dev *dev)

{
  Serial.print("Chip ID ");
  Serial.println(dev->chip_id, HEX);
  Serial.print("Device ID ");
  Serial.println(dev->dev_id, HEX);
  Serial.print("Bus Interface ");
  Serial.println(dev->intf);
  Serial.print("Settings" );
  Serial.println(dev->settings.standby_time);

  Serial.print("\n\n");

  Serial.print("T1: ");
  Serial.println(dev->calib_data.dig_T1);
  Serial.print("T2: ");
  Serial.println(dev->calib_data.dig_T2);
  Serial.print("T3: ");
  Serial.println(dev->calib_data.dig_T3);
  Serial.print("P1: ");
  Serial.println(dev->calib_data.dig_P1);
  Serial.print("P2: ");
  Serial.println(dev->calib_data.dig_P2);
  Serial.print("P3: ");
  Serial.println(dev->calib_data.dig_P3);
  Serial.print("P4 ");
  Serial.println(dev->calib_data.dig_P4);
  Serial.print("P5: ");
  Serial.println(dev->calib_data.dig_P5);
  Serial.print("P6 ");
  Serial.println(dev->calib_data.dig_P6);
  Serial.print("P7 ");
  Serial.println(dev->calib_data.dig_P7);
  Serial.print("P8 ");
  Serial.println(dev->calib_data.dig_P8);
  Serial.print("P9: ");
  Serial.println(dev->calib_data.dig_P9);
  Serial.print("H1: ");
  Serial.println(dev->calib_data.dig_H1);
  Serial.print("H2: ");
  Serial.println(dev->calib_data.dig_H2);
  Serial.print("H3: ");
  Serial.println(dev->calib_data.dig_H3);
  Serial.print("H4: ");
  Serial.println(dev->calib_data.dig_H4);
  Serial.print("H5: ");
  Serial.println(dev->calib_data.dig_H5);
  Serial.print("H6: ");
  Serial.println(dev->calib_data.dig_H6);
  Serial.print("t_fine: ");
  Serial.println(dev->calib_data.t_fine);


  Serial.print("\n\n");
  
}

void BME_Sensors::updateDailyTempHiandLow(){
if (bstart_check == 0) {
    DHhiTemp = avgTemp;
    DHloTemp = avgTemp;
    //bstart_check = 1;
  } else {
    //chech for new high temp
    if (avgTemp >= DHhiTemp) {
      DHhiTemp = avgTemp;
    }
    //check for new low temp
    if (avgTemp <= DHloTemp) {
      DHloTemp = avgTemp;
    }
  }
}

void BME_Sensors::updateDailyPressureHiandLow(){
if (bstart_check == 0) {
    DHhiPressure = Mb;
    DHloPressure = Mb;
    //bstart_check = 1;
  } else {
    //chech for new high Pressure
    if (Mb >= DHhiPressure) {
      DHhiPressure = Mb;
    }
    //check for new low Pressure
    if (Mb <= DHloPressure) {
      DHloPressure = Mb;
    }
  }
}

void BME_Sensors::updateDailyHumidityHiandLow(){
if (bstart_check == 0) {
    DHhiHumidity = humidity;
    DHloHumidity = humidity;
    bstart_check = 1;
  } else {
    //chech for new high humidity
    if (humidity >= DHhiHumidity) {
      DHhiHumidity = humidity;
    }
    //check for new low humidity
    if (humidity <= DHloHumidity) {
      DHloHumidity = humidity;
    }
  }
}

float BME_Sensors::dew_point_calc(float in_temp, float in_humid) {
  //float dp = (pow((in_humid/100), .125)) * (112 + (0.9 * in_temp)) + 0.1*in_temp - 112;
  float Es = 6.11 * pow(10, ((7.5 * in_temp) / (237.7 + in_temp)));
  float E = (in_humid * Es) / 100;
  float dp = (-430.22 + 237.7 * log(E)) / (-log(E) + 19.08);
  return dp;
}

float BME_Sensors::wetbulb_calc(float in_temp, float in_humid) {
  //float pow1 =  pow((50+8.313659),((1)/(2)));
  //float pow2 = pow(in_humid,((3)/(2)));
  float wb = in_temp * atan(0.151977 * pow((in_humid + 8.313659), 0.5)) ;
  float wb2 = (atan(in_temp + in_humid)) - (atan(in_humid - 1.676331));
  float wb3 = 0.00391838 * pow(in_humid, 1.5) * atan(0.023101 * in_humid) - 4.686035;
  return wb + wb2 + wb3;
}

float BME_Sensors::relitive_humidity_calc(float in_temp, float in_wbtemp, float P) {
  float Cp = 1.005;
  float Cpv = 4.186;
  float Lv = 2500;
  float Es = 6.11 * pow(10, ((7.5 * in_temp) / (237.7 + in_temp)));
  float Eswb = 6.11 * pow(10, ((7.5 * in_wbtemp) / (237.7 + in_wbtemp)));
  float W = ((in_temp - in_wbtemp) * (Cp) - Lv * (Eswb / P)) / (-(in_temp - in_wbtemp) * (Cpv) - Lv);
  float Ws = Es / P;
  return (W / Ws) * 100;
  //return (Es / Eswb) * 100;
}

int BME_Sensors::UpdateRedings(struct bme280_dev *dev)
{
  
  int attempts = 0;
  int newTemp;
  //->Serial1.print("geting the temperatures for ");
  //set.printAddress();
  //->Serial1.println();
    do {
      newTemp = stream_sensor_data_normal_mode(dev);
      //->Serial1.println(newTemp);
      attempts++;
    } while (!addHiTemp(celsius) and (attempts < 5));
    hiTemp = hiArray[0];

    attempts = 0;
    do {
      newTemp = stream_sensor_data_normal_mode(dev);
      attempts++;
    } while (!addLoTemp(celsius) and (attempts < 5));
    loTemp = loArray[0];
    avgTemp = (hiTemp + loTemp) / 2.0;
    updateDailyTempHiandLow();
    updateDailyPressureHiandLow();
    updateDailyHumidityHiandLow();
    
  //->Serial1.println(" in get temp function");

  return newTemp;

}

float BME_Sensors::readFloatAltitudeMeters( void )
{
  float heightOutput = 0;
      float T = celsius;
      double P = Pressure;
      double P0 = sealevel(1390,P,T);
  
  //heightOutput = ((float)-45846.2)*(pow(((float)readFloatPressure()/(float)101188), 0.190263) - (float)1);
      heightOutput = (((pow((P0/P),0.190263)) -1)*(T+273.15))/0.0065;
      Altitude = heightOutput;
  return heightOutput;
  
}

float BME_Sensors::readFloatAltitudeFeet( void )
{
  float heightOutput = 0;
  
  heightOutput = Altitude * 3.28084;
  return heightOutput;
  
}

double BME_Sensors::sealevel(double A, double P, float T)
// Given a pressure P (mb) taken at a specific altitude (meters),
// return the equivalent pressure (mb) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
  //return(P/pow(1-(A/44330.0),5.255));
     //double sealevelp;
     float AA = 0.0065*A;
     SeaLevelPressure = (P*pow((1.0 - (AA /(T + AA + 273.15))), - 5.257));
     return SeaLevelPressure;

}


double BME_Sensors::altitude(double P, double P0)
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
{
  //return(44330.0*(1-pow(P/P0,1/5.255)));
      return((((log(P/P0))*287.053)* ((fahrenheit+ 459.67) *5/9))/ -9.8);
}



} //end namespace
