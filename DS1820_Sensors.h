#include <algorithm>
#include <vector>
#include <DallasTemperature.h>
#include <math.h>

#define SAMPLE_SIZE 25 
//#define USING_STANDARD_DEVIATION  true

#ifdef USING_STANDARD_DEVIATION
  #define STANDARD_DEVIATION_FILTER 2.0  // we will toss any value that isnt within this number of standard deviations of the past BUFFER_SIZE readings
#else
  #define RANGE_BASED_FILTER 10.0  // total band 
#endif
#define ONE_WIRE_BUS 14
#define TEMPERATURE_PRECISION 12
const int LEDPIN1 = 16;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

 
class DS1820_Sensors  
{  
  private:
  
  bool crcCheck();
  byte calc_CRC_bit (byte shift_reg, byte data_bit);
  
  public:
  char sensorName[20];
  uint8_t deviceaddress[8];
  int  DeviceNum;
  //uint8_t hiSensor[8]; // high sensor
  //uint8_t loSensor[8];  // low sensor
  double hiTemp;
  double DHhiTemp;
  double loTemp;
  double DHloTemp;
  double temp;
  double celsius;
  double fahrenheit;
  double avgTemp;
  double energy;
  std::vector<double> hiArray;
  std::vector<double> loArray;
  byte CRC;
  byte calc_CRC;
  int dstart_check = 0;
  bool addHiTemp(double newVal);
  bool addLoTemp(double newVal);
  double getTemp();
  void setAvgTemp(double num);
  double getAvgTemp();
  DS1820_Sensors(void);
  //~DS1820_Sensors(void);
  void printAddress();
  void printTemperature();
  void printResolution();
  void setresolution();
  void printData();
  void setDeviceNum(int devicenum);
  void updateDailyHiandLow();
};


void DS1820_Sensors::setAvgTemp(double num)
{
  avgTemp = num;
}

double DS1820_Sensors::getAvgTemp()
{
 double _temp =  avgTemp;
 return _temp; 
}
 
  bool DS1820_Sensors::addHiTemp(double newVal)
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
  bool DS1820_Sensors::addLoTemp(double newVal)
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


DS1820_Sensors::DS1820_Sensors(void)
{
  hiTemp =0;
  loTemp =0;
  temp =0;
  avgTemp =0;
  energy =0;
  //sensors.begin();
  
}

//DS1820_Sensors::~DS1820_Sensors(void)
//{
//}


// function to print a device address
void DS1820_Sensors::printAddress()
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceaddress[i] < 16) Serial.print("0");
    //->Serial1.print(deviceaddress[i], HEX);
  }
}

// function to print the temperature for a device
void DS1820_Sensors::printTemperature()
{
  float tempC = temp ;
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
  temp = tempC;
}

// function to print a device's resolution
void DS1820_Sensors::printResolution()
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceaddress));
  Serial.println();
}

// main function to print information about a device
void DS1820_Sensors::printData()
{
  Serial.print("Device Address: ");
  printAddress();
  Serial.print(" ");
  printResolution();
  Serial.print(" ");
  printTemperature();
  Serial.println();
}

byte DS1820_Sensors::calc_CRC_bit (byte shift_reg, byte data_bit)
     {
        byte fb;
        fb = (shift_reg & 0x01) ^ data_bit ;
   /* exclusive or least sig bit of current shift reg with the data bit */

        shift_reg = shift_reg >> 1;     
     /* shift one place to the right */

        if (fb==1){
          shift_reg = shift_reg ^ 0x8C;
        }   
        return(shift_reg); 
     }
 
bool DS1820_Sensors::crcCheck()
{
  byte crc =0;
  byte data_bit = 0;
  byte shift_reg = 0;
  uint8_t scratchPad[9];
  
  //->Serial1.println(" in get crcCheck function");

  sensors.readScratchPad(deviceaddress,scratchPad);

  crc = scratchPad[8];
  CRC = scratchPad[8];
 /* 
  for (uint8_t i = 8; i ; i--)
  {
    data_bit = scratchPad[i];
    shift_reg = calc_CRC_bit (shift_reg,data_bit);
  }
  calc_CRC = shift_reg;
*/
//->Serial1.print(" crc check is ");
//->Serial1.print(scratchPad[8]);
//->Serial1.print(" and ");

int calc_CRC = OneWire::crc8(scratchPad, 8);
//->Serial1.println(calc_CRC);  

  if (calc_CRC == scratchPad[8]) {
    digitalWrite(LEDPIN1, 1);
    return true;
  }
  else {
    digitalWrite(LEDPIN1, 0);
    return false;
  }
  
}

double DS1820_Sensors::getTemp()
{
  static const int MAXRETRY = 3;
  double _temp;
  int i = 0;

  //->Serial1.println(" in get temp function");
  if (temp ==0){
    sensors.requestTemperaturesByAddress(deviceaddress);
    temp = 1;
  }else{
    temp = 1;
   do {
    sensors.requestTemperaturesByAddress(deviceaddress);
    
    _temp = sensors.getTempC(deviceaddress);
    //->Serial1.println(_temp);
    } while (!crcCheck() && MAXRETRY >= i++);
  }
  if (i < MAXRETRY)
  {
    celsius = _temp;
    fahrenheit = DallasTemperature::toFahrenheit(_temp);
    //->Serial1.println(_temp);
  }
  else
  {
    _temp = NAN;
    celsius = fahrenheit = NAN;
    Serial.println("Invalid reading");
  }
  temp = _temp;
 

 // _temp = sensors.getTempC(deviceaddress);
  //temp = _temp;
  //celsius = _temp;
  //fahrenheit = DallasTemperature::toFahrenheit(_temp);
  return _temp;
}

void DS1820_Sensors::setresolution()
{
  sensors.setResolution(deviceaddress, TEMPERATURE_PRECISION);
}

void DS1820_Sensors::setDeviceNum(int devicenum)
{
  DeviceNum = devicenum;
}

void DS1820_Sensors::updateDailyHiandLow(){
if (dstart_check == 0) {
    DHhiTemp = avgTemp;
    DHloTemp = avgTemp;
    dstart_check = 1;
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




