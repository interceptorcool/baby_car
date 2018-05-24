# baby_car
controlling the child's car from the phone

# library
1. Filters-master.zip
2. RemoteXY.zip

# Files
main Arduino files
	CAR_Bluetooth_HC05_HardSerial.ino

PWM Brused motor control
	pwm_motor.hpp


# classes  PWMmotor
	used only ATmega32u4 - for use all chandge setPWM

		 PWMmotor.setup() - init ports
		 PWMmotor.Update(pct)
			pct = -100 ... 100 
			Auto:
			    pct = 0 - neutral

				pct = 0 -- change to --> pct > 0 => forward
				pct > 0 -- change to --> pct < 0 => break

				pct = 0 -- change to --> pct < 0 => reverse
				pct < 0 -- change to --> pct > 0 => break

