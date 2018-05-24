/**
 ******************************************************************************
 *
 *     AUTHOR:         Igor Kulunchakov
 *     FULLNAME:       CAR_Bluetooth_HC05_HardSerial.ino
 *     PROCESSOR:      ATmega8u32
 *     TOOLKIT:        Arduino, avr-gcc
 *     DESCRIPTION:    Удаленное управление машинкой дочи
 *						основано на проекте http://remotexy.com/ru/ через HC-05
 *						подключены серва руля, ходовые двигатели.
 *						педаль газа, переключение направления.
 *     Version:        0.01
 *
 ******************************************************************************
 */

#include <Filters.h>

///////////////////////////////////////////// 
//        RemoteXY include library         // 
///////////////////////////////////////////// 
/* RemoteXY select connection mode and include library */ 
#define REMOTEXY_MODE__HC05_HARDSERIAL 

#define    REMOTEXY__LEDRX    17
//#define    REMOTEXY__LEDTX    30

/* RemoteXY connection settings */ 
#define REMOTEXY_SERIAL Serial1
#define REMOTEXY_SERIAL_SPEED 9600
 
#include <RemoteXY.h> 

// конфигурация интерфейса   
#pragma pack(push, 1) 
uint8_t RemoteXY_CONF[] = 
  { 255,5,0,1,0,93,0,8,242,1,
	  2,0,4,2,25,11,3,26,31,31,
	  79,78,0,79,70,70,0,2,0,35,
	  2,25,11,1,26,31,31,208,160,208,
	  163,208,155,208,172,0,208,162,208,155,
	  208,164,0,5,32,4,42,56,56,233,
	  26,31,66,129,4,29,55,9,134,26,
	  2,0,4,15,56,10,1,26,31,31,
	  208,159,208,149,208,148,208,144,208,155,
  208,172,0,208,162,208,155,208,164,0 };

/*
  { 255,5,0,1,0,107,0,8,242,1,
	  2,0,4,2,25,11,3,26,31,31,
	  79,78,0,79,70,70,0,2,0,35,
	  2,25,11,1,26,31,31,208,160,208,
	  163,208,155,208,172,0,208,162,208,155,
	  208,164,0,5,32,4,42,56,56,233,
	  26,31,66,129,19,29,40,8,134,26,
	  2,0,4,15,56,10,1,26,31,31,
	  208,159,208,149,208,148,208,144,208,155,
	  208,172,0,208,162,208,155,208,164,0,
	  129,0,5,30,18,6,64,208,145,208,
	  144,208,162,0 };
  */ 
   
// структура определяет все переменные вашего интерфейса управления  
struct { 

    // input variable
  uint8_t mainPower; // =1 если переключатель включен и =0 если отключен 
  uint8_t rudder; // =1 если переключатель включен и =0 если отключен 
  int8_t joystick_1_x; // =-100..100 координата x положения джойстика 
  int8_t joystick_1_y; // =-100..100 координата y положения джойстика 
  uint8_t accellerator; // =1 если переключатель включен и =0 если отключен 
  
    // output variable
  int8_t level_1; // =0..100 положение уровня

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY; 
#pragma pack(pop) 

///////////////////////////////////////////// 
//           END RemoteXY include          // 
///////////////////////////////////////////// 

//#include <MedianFilter.h>


///////////////////////////////////////////// 
// BSP section

const int  PIN_RELAY_POWER  = 8;
const int  PIN_RELAY_RUDDER = 9;

const int  PIN_HC05_RESET   = 15;
const int  PIN_HC05_PIO11   = 7;

const int  LED_ARDUINO_1    = 17;
const int  LED_ARDUINO_2    = 30;

///////////////////////////////////////////// 
// Servo
#include <Servo.h>

Servo	rudder;

///////////////////////////////////////////////
// PWM Motor

//#define DEBUG_MOTOR	Serial
#include "pwm_motor.hpp"
PWMmotor			pwm_motor;
ManualAccControl	manual_pwm_motor;

uint8_t BattaryEmpty;

void setup()  
{ 
   
  pinMode (PIN_HC05_RESET, OUTPUT);
  pinMode (PIN_HC05_PIO11, OUTPUT);
   
  pinMode (PIN_RELAY_POWER, OUTPUT);
  pinMode (PIN_RELAY_RUDDER, OUTPUT);

  digitalWrite(PIN_RELAY_POWER, HIGH);
  digitalWrite(PIN_RELAY_RUDDER, HIGH);

  pinMode (LED_ARDUINO_2, OUTPUT);
  digitalWrite(LED_ARDUINO_2, HIGH);

  
  // reset BT
  delay(100);
  digitalWrite(PIN_HC05_RESET, LOW);
  delay(50);
  digitalWrite(PIN_HC05_PIO11, LOW);
  delay(50);
  digitalWrite(PIN_HC05_RESET, HIGH);
  
  
  // TODO you setup code 
#ifdef DEBUG_MOTOR
  Serial.begin(115200);
#endif

  analogReference(INTERNAL);
  
  pwm_motor.setup();
  manual_pwm_motor.setup();
  
  rudder.attach(6);
  rudder.write(0);

	BattaryEmpty = 0;

	RemoteXY_Init ();  
 } 

/*
 * Напряжение для LiPO 3S
 */
const float cCellUmin = 3.3;  // минимальное напряжение на банке
const float cCellUmax = 4.2;  // максимальное напряжение на банке
const int   cCellCount = 3;

const float cBattaryUmax = cCellUmax * cCellCount;
const float cBattaryUmin = cCellUmin * cCellCount;
const float cBattaryDiapazone = cBattaryUmax - cBattaryUmin;

const float cRezistorUp = 22000;
const float cRezistorDown = 5210;

FilterOnePole f_measure_power(LOWPASS, 0.5);

const unsigned long cBattaryScanInterval  = 300;
unsigned long BattaryScanPvev = 0;

int ConvertorU()
{
  if (millis() - BattaryScanPvev > cBattaryScanInterval)
  {
    BattaryScanPvev = millis();
  
    int u =   analogRead(A0);
  
    // конвертируем в показания датчика
    float AtoU = ((u * (float)2.56) / (float)1023);
    float u_battary = AtoU * (cRezistorDown + cRezistorUp / cRezistorDown) / 1000;
    
    int diapazone = (u_battary - cBattaryUmin)* 100.0 / cBattaryDiapazone;
    f_measure_power.input(diapazone);
  }
  return f_measure_power.output();
}

///////////////////////////////////////////////

unsigned long blink = 0;

uint8_t flag_bat_ok = 0;

void loop()  
{  
	RemoteXY_Handler ();
	if (BattaryEmpty < 100 && RemoteXY.connect_flag)
	{
	
		int battary_pos =  ConvertorU();
		if (battary_pos >= 0)
		{
			RemoteXY.level_1 = battary_pos;
			flag_bat_ok = 1;
		} else
		{
			// немедленная остановка
			RemoteXY.mainPower = 0;
			if (flag_bat_ok)
				BattaryEmpty++;
		}

		if (RemoteXY.mainPower)
		{
			digitalWrite(PIN_RELAY_POWER, LOW);
			if (RemoteXY.accellerator == 0)
			{
				pwm_motor.Update(RemoteXY.joystick_1_y);
			} 
			else
			{
				manual_pwm_motor.Update();
			}

			if (RemoteXY.rudder)
			{
				digitalWrite(PIN_RELAY_RUDDER, LOW);
				rudder.write(90);
			} else
			{
				digitalWrite(PIN_RELAY_RUDDER, HIGH);
				rudder.write(map(RemoteXY.joystick_1_x, -100, 100, 180, 0));
			}
	
		} else 
		{
			digitalWrite(PIN_RELAY_POWER, HIGH);
			pwm_motor.Update(0);
		}
	
	} // if (!BattaryEmpty)

		
	if (millis() - blink > 1000)
	{
		blink = millis();
		digitalWrite(LED_ARDUINO_2, !digitalRead(LED_ARDUINO_2));
	}

}
