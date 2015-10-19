#ifndef ME_ENCODER_MOTOR_H
#define ME_ENCODER_MOTOR_H
#include <Arduino.h>

#define PULSE_PER_C 12 

class MeEncoderMotor{
public:
	MeEncoderMotor();
	void begin();
	void moveTo(uint8_t motor, long angle);
	void moveSpeed(uint8_t motor, int rpm);
	void setHold(uint8_t motor,uint8_t hold);
	void setPID(uint8_t motor, float p,float i,float d,float s);
	void setMaxPower(uint8_t motor,int8_t maxPower);
	long getAngle(uint8_t motor);
  int8_t getPower(uint8_t motor);
	void getPID(uint8_t motor,float * p,float * i,float * d,float * s);
	int getSpeed(uint8_t motor);
        void resetMotor(uint8_t motor);
        float getRatio(uint8_t motor);
        void setRatio(uint8_t motor, float r);
        int getPulse(uint8_t motor);
        void setPulse(uint8_t motor, int p);
        void setMode(uint8_t motor, uint8_t mode);
        void setPWM(uint8_t motor, int pwm);
private:
//        size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size);
        void sendCmd();
	uint8_t address;
	char cmdBuf[18];
//        float ratio[2];
};

#endif

