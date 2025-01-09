#pragma once
#include <Arduino.h>
#include <math.h>

class BatReading {
private:
  int _pin, _r1, _r2, _avg, _resolution;
  float _maxVoltage, _minVoltage, _coe;

public:
  float _voltage, _voltsPercentage;

  typedef struct {
    float voltage, voltsPercentage;
  } Bat;

  BatReading();
  ~BatReading();
  void init(int pin, int resolution, int r1, int r2, float maxVoltage, float minVoltage);
  void reading(int avg);
  void readMilliVolts(int avg);
  Bat read(int avg);
};

BatReading::BatReading(){};

BatReading::~BatReading(){};

void BatReading::init(int pin, int resolution, int r1, int r2, float maxVoltage, float minVoltage) {
  _pin = pin;
  _resolution = resolution;
  _r1 = r1;
  _r2 = r2;
  _maxVoltage = maxVoltage;
  _minVoltage = minVoltage;
  _coe = pow(2, _resolution);
  analogReadResolution(_resolution);
}

void BatReading::reading(int avg) {
  // 取均值
  _avg = avg;
  int adcValue, sum = 0;
  for (int count = 0; count < _avg; count++) {
    sum += analogRead(_pin);
  }
  // 电压计算
  adcValue = sum / _avg;
  float volts = (adcValue / _coe) * 3.6;
  _voltage = volts * (_r1 + _r2) / _r2;
  // 百分比计算
  if (_voltage >= _maxVoltage) {
    _voltsPercentage = 100.0;
  } else if (_voltage <= _minVoltage) {
    _voltsPercentage = 0.0;
  } else {
    _voltsPercentage = (_voltage - _minVoltage) / (_maxVoltage - _minVoltage) * 100.0;
  }
}

/*************************************************************************************************************/

// 注意结构体的作用域。使用 BatReading::Bat 来明确指定 Bat 的作用域。
// 而不能像声明其他函数那样 void BatReading::init(int pin, int resolution)，直接用Bat BatReading::read(int avg)来声明函数 read(int avg) 的类型。
BatReading::Bat BatReading::read(int avg) {
  // 取均值
  _avg = avg;
  Bat B;
  int adcValue, sum = 0;
  for (int count = 0; count < _avg; count++) {
    sum += analogRead(_pin);
  }
  // 电压计算
  adcValue = sum / _avg;
  float volts = (adcValue / _coe) * 3.6;
  B.voltage = volts * (_r1 + _r2) / _r2;
  // 百分比计算
  if (B.voltage >= _maxVoltage) {
    B.voltsPercentage = 100.0;
  } else if (B.voltage <= _minVoltage) {
    B.voltsPercentage = 0.0;
  } else {
    B.voltsPercentage = (B.voltage - _minVoltage) / (_maxVoltage - _minVoltage) * 100.0;
  }
  return B;
}

/*************************************************************************************************************/
