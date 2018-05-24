/**
 ******************************************************************************
 *
 *     AUTHOR:         Igor Kulunchakov
 *     FULLNAME:       pwm_motor.h
 *     PROCESSOR:      ATmega8u32
 *     TOOLKIT:        Arduino, avr-gcc
 *     DESCRIPTION:    MC33886 - Brused Motor Lib. 
 *     Version:        0.01
 *
 ******************************************************************************
 */

#ifndef _PWM_MOTOR_HPP_
#define _PWM_MOTOR_HPP_

#include <Filters.h>
#include "Arduino.h"
#include <avr/io.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
	

/*
 * ========================================================================
 * PWM Brushed Motor Control
 */
class PWMmotor
{
public:
	enum class TStatus
	{
		STOP	= 0,
		NEUTRAL,
		FORWARD,
		REVERSE,
		BREAK
	};

	enum class TAppStat
	{
		NEUTRAL	= 0,
		PRE_ACCELERATION,
		ACCELERATION,
		BREAKING,
		PRE_REVERSING,
		REVERSING
	};

public:
	PWMmotor()
	{
		f_acceleration = new FilterOnePole(LOWPASS, 1.0, 0);
		_app_stat = TAppStat::NEUTRAL;
		_status	= TStatus::NEUTRAL;
		_time_app_chage = 0;
	}


	// GPIO 15 - IN1	- Direction
	static const uint8_t PIN_DIR	= 14;
	// GPIO 13 - IN2	- PWM
	static const uint8_t PIN_PWM	= 10;
	// GPIO 0  - EN		- Eneble
	static const uint8_t PIN_PWR		= 16;

	// pct - возле 0
	static const uint8_t NEUTRAL_PCT	= 5;

	static const int	MILLIS_BREAK_DELAY	= 300;
	
	static const int	MAX_PCT		= 100;
	static const int	MIN_PCT		= -100;
	static const int	EXPONENT	= 40;
	static const float	EXP_CONV	= exp(((float)(MAX_PCT) - 0.3) / (float)EXPONENT);

	void	setup()
	{
		pinMode(PIN_DIR, OUTPUT);
		digitalWrite(PIN_PWR, LOW);
		pinMode(PIN_PWR,  OUTPUT);

		// init timer 4? channal B
		TCCR4A = _BV(PWM4B);
		TCCR4B = _BV(CS40) | _BV(CS41) | _BV(CS42);
		TCCR4D = _BV(WGM40);
		
		Break();
	}

private:
	void	setPWM(uint16_t val)
	{
			sbi(DDRB, PINB6);
			//cbi(PORTB, PINB6);
			
			if (val == 0)
			{
				cbi(TCCR4A, COM4B1);
				cbi(PORTB, PINB6);
			}
			else if (val >= 255)
			{
				cbi(TCCR4A, COM4B1);
				sbi(PORTB, PINB6);
			}
			else 
			{
				// connect pwm to pin on timer 4, channel B
				sbi(TCCR4A, COM4B1);
				OCR4B = val; // set pwm duty
				
			}
	}

	void	Forward(int pwmSpeed)
	{
		_status = TStatus::FORWARD;
		digitalWrite(PIN_PWR,  HIGH);
		digitalWrite(PIN_DIR, HIGH);
		//analogWrite (PIN_PWM, 255 - pwmSpeed);
		setPWM(255 - pwmSpeed);

	}

	void	Reverse(int pwmSpeed)
	{
		_status = TStatus::REVERSE;
		digitalWrite(PIN_PWR, HIGH);
		digitalWrite(PIN_DIR, LOW );
		//analogWrite (PIN_PWM, pwmSpeed);
		setPWM(pwmSpeed);
	}
	
	void	Neutral()
	{
		// Free running - motor stop
		setPWM(0);
		digitalWrite(PIN_PWR, LOW);
		digitalWrite(PIN_DIR, LOW);

		_status = TStatus::NEUTRAL;
	}

	void	Break()
	{
		// FAST Break
		setPWM(0);
		digitalWrite(PIN_PWR, HIGH);
		digitalWrite(PIN_DIR, LOW);

		_status = TStatus::STOP;
	}
	
private:	
	int	ExponentConvert(int pct)
	{
		int _pct = abs(pct);
		int result = (int)((float)_pct * ((float)exp((float)_pct / (float)EXPONENT) / EXP_CONV));
		if (result > MAX_PCT) result = MAX_PCT;
		return result;
	}

public:
	void	Update(int pct)
	{
		TAppStat  input_stat;
		
		if (pct == 0)
		{
		  input_stat = TAppStat::NEUTRAL;
		} else if (pct > 0) 
		{
		  input_stat = TAppStat::ACCELERATION;
		} else
		{
		  input_stat = TAppStat::REVERSING;
		}


    switch(input_stat)
    {
    case TAppStat::NEUTRAL:
        switch(_app_stat)
        {
        case TAppStat::ACCELERATION:
		case TAppStat::REVERSING:
			Neutral();
		case TAppStat::NEUTRAL:
			f_acceleration->setToNewValue(0);
        case TAppStat::PRE_REVERSING:
		case TAppStat::PRE_ACCELERATION:
			_app_stat = TAppStat::NEUTRAL;
		break;
		
		}	
		
    break;
    
    case TAppStat::ACCELERATION:
        switch(_app_stat)
        {
		case TAppStat::NEUTRAL:
		case TAppStat::PRE_REVERSING:
        case TAppStat::ACCELERATION:
			_app_stat = TAppStat::ACCELERATION;
								
			f_acceleration->input(map(ExponentConvert(pct), 0, 100, 0 ,255));
			Forward(f_acceleration->output());
			
			#ifdef DEBUG_MOTOR
				DEBUG_MOTOR.print("ACC:ACC ");
				DEBUG_MOTOR.println(f_acceleration->output());
			#endif
		break;
		case TAppStat::BREAKING:
			if (millis() - _time_app_chage > MILLIS_BREAK_DELAY)
			{
				// остановились
			#ifdef DEBUG_MOTOR
				DEBUG_MOTOR.println("ACC:STOP");
			#endif
				_app_stat = TAppStat::PRE_ACCELERATION;
			}
		break;
		case TAppStat::REVERSING:
			_time_app_chage = millis();
			_app_stat = TAppStat::BREAKING;
			Break();
			f_acceleration->setToNewValue(0);
			#ifdef DEBUG_MOTOR
				DEBUG_MOTOR.println("ACC:REV");
			#endif
		break;

        case TAppStat::PRE_ACCELERATION:
		break;
        }
    break;

    case TAppStat::REVERSING:
        switch(_app_stat)
        {
        case TAppStat::ACCELERATION:
			// отрабатываем торможение
			_time_app_chage = millis();
			_app_stat = TAppStat::BREAKING;
			Break();
			f_acceleration->setToNewValue(0);
			#ifdef DEBUG_MOTOR
				DEBUG_MOTOR.println("REV:BRK");
			#endif
		break;
		case TAppStat::BREAKING:
			if (millis() - _time_app_chage > MILLIS_BREAK_DELAY)
			{
				// остановились
			#ifdef DEBUG_MOTOR
				DEBUG_MOTOR.println("REV:STOP");
			#endif
				_app_stat = TAppStat::PRE_REVERSING;
			}
		break;
		case TAppStat::NEUTRAL:
		case TAppStat::REVERSING:
			_app_stat = TAppStat::REVERSING;
			
			f_acceleration->input(map(ExponentConvert(pct), 0, 100, 0 ,255));
			Reverse(f_acceleration->output());
			
			#ifdef DEBUG_MOTOR
				DEBUG_MOTOR.print("REV:REV ");
				DEBUG_MOTOR.println(f_acceleration->output());
			#endif
		break;
        case TAppStat::PRE_ACCELERATION:
		case TAppStat::PRE_REVERSING:
		break;
        }
    break;
    }
  
	}

private:
	TStatus		_status;
	TAppStat	_app_stat;

	uint32_t	_time_app_chage;

protected:
	FilterOnePole 	*f_acceleration;
 
};

/*
 * управление педалью
 */
class ManualAccControl: public PWMmotor
{
public:
	static const uint8_t	SCAN_ACCELETATION_PEDAL_MS = 5;
	
	static const uint8_t	PIN_DI_FORWARD	= 4;
	static const uint8_t	PIN_DI_REVERSE	= 5;
	
public:
	using PWMmotor::PWMmotor;
	
	void setup()
	{
		pinMode(PIN_DI_FORWARD, INPUT_PULLUP);
		pinMode(PIN_DI_REVERSE, INPUT_PULLUP);
		
		f_acceleration->setFilter(LOWPASS, 0.1, 0);
	}
	
	void Update() 
	{	
		// сканируем состояние датчиков
		forward.SetStatusRead(PIN_DI_FORWARD);
		reverse.SetStatusRead(PIN_DI_REVERSE);
		
		// автомат на состояние управления
		int pct = 0;
		if (reverse.status)
		{
			pct = MIN_PCT;
		}
		
		if (forward.status)
		{
			pct = MAX_PCT;
		}
		//Serial.println(pct);
		
		PWMmotor::Update(pct);
		
	}

private:
	struct TDigitalIn
	{
		uint32_t		_time_scan_save;
		uint8_t			status;
		
		TDigitalIn(): status(0), _time_scan_save(0)	{};
			
		void SetStatusRead(uint8_t port)
		{
			if (millis() - _time_scan_save > SCAN_ACCELETATION_PEDAL_MS)
			{
				_time_scan_save = millis();
			}
			else
			{
				return;
			}
			
			if (digitalRead(port) == LOW) // нажатие педали 0 на порту
			{
				status <<= 2;
				status |= 0x3;
			} 
			else
			{
				status >>= 2;
				status &= 0x3F;
			}
		}
	};

private:
	TDigitalIn		forward;
	TDigitalIn		reverse;
	

};

#endif /* FLY_LED_LED_IO_HPP_ */
