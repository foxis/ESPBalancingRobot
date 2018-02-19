/*  ESPBalancingRobot
 *
 *  Copyright (C) 2018  foxis (Andrius Mikonis <andrius.mikonis@gmail.com>)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <Wire.h>
#include <MadgwickAHRS.h>
#include <PID_v1.h>
#define SENSORS_MPU6050_ATTACHED
#include <Sensors.h>
#include <math.h>
#include "DiffDrive.h"

#define MPU_addr 0x68
#define MOTOR_A0  14
#define MOTOR_B0  12
#define MOTOR_A1  0
#define MOTOR_B1  2
#define IMU_SDA   3
#define IMU_SCL   1

//#define USE_LOGGING 1

#define STABILIZING_TIMEOUT 2000000
#define INCLINATION_THRESHOLD .6

#ifdef USE_LOGGING
#define LOG_SIZE 4
typedef struct log_struct
{
	unsigned long now;
	float targetInclination;
	float currentInclination;
	float fwdControl;
	float targetSpeed;
	float currentSpeed;
} log_t;
#endif

class Robot
{
  Madgwick filter;
  DiffDrive ddrive;
  PID pidInclination;
  PID pidHeading;
  PID pidSpeed;

  long prevTimeStamp;
  long duration;
  long fps;
	unsigned long stabilizingStart = 0;

  // PID input/output/target
  double currentInclination = 0, fwdControl, targetInclination = -.0475;
  double currentHeading = 0, headingControl, targetHeading = 0;
  double currentSpeed = 0, targetSpeed = 0;
	bool stabilizing = true;

#ifdef USE_LOGGING
	volatile bool dumping = false;
	volatile bool logging = false;
	size_t log_tail = 0, log_head = 0, log_size = 0;
	log_t log[LOG_SIZE];
#endif

public:
  Robot(long fps, double wheelBase)
	//:pidInclination(&currentInclination, &fwdControl, &targetInclination, 8.0, 0.0, 0.09, DIRECT),
    :pidInclination(&currentInclination, &fwdControl, &targetInclination, 5.0, 19.0, 0.09, DIRECT),
    pidHeading(&currentHeading, &headingControl, &targetHeading, 0.1, 0.0, 0.001, DIRECT),
    pidSpeed(&currentSpeed, &targetInclination, &targetSpeed, .4, .6, 0.01, REVERSE),
    ddrive(MOTOR_A0, MOTOR_B0, MOTOR_A1, MOTOR_B1, wheelBase)
  {
    this->fps = fps;
    duration = 1000000L / fps;
    pidSpeed.SetOutputLimits(-.5, .05);
    pidHeading.SetOutputLimits(-1, 1);
    pidInclination.SetOutputLimits(-1, 1);
    pidInclination.SetSampleTime(duration);
    pidSpeed.SetSampleTime(duration);
    pidHeading.SetSampleTime(duration);
    pidInclination.SetMode(AUTOMATIC);
    pidSpeed.SetMode(AUTOMATIC);
    pidHeading.SetMode(AUTOMATIC);
  }

  void begin()
  {
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);
    filter.begin(fps);

    Sensors::initialize();

    ddrive.init();
    prevTimeStamp = micros();
  }

  void loop(unsigned long now)
  {
    if ( now - prevTimeStamp > duration ) {
			#ifdef USE_LOGGING
						logging = true;
						if (log_size < LOG_SIZE && !dumping)
						{
							log[log_head].now = now;
							log[log_head].currentInclination = (float)currentInclination;
							log[log_head].targetInclination = (float)targetInclination;
							log[log_head].fwdControl = (float)fwdControl;
							log[log_head].currentSpeed = (float)currentSpeed;
							log[log_head].targetSpeed = (float)targetSpeed;
							log_head = (log_head + 1) % LOG_SIZE;
							log_size ++;
						}
						logging = false;
			#endif

			DetermineOrientation();

			if (stabilizing || fabs(currentInclination) > INCLINATION_THRESHOLD) {
				if (now - stabilizingStart > STABILIZING_TIMEOUT && stabilizingStart != 0 && fabs(currentInclination) < INCLINATION_THRESHOLD)
					stabilizing = false;
				else {
					ddrive.setDiffSpeed(0, 0);
					fwdControl = 0;
					targetInclination = -.0475;
					pidInclination.Reset();
					pidHeading.Reset();
					pidSpeed.Reset();
					if (fabs(currentInclination) > INCLINATION_THRESHOLD)
					{
						stabilizingStart = now;
						stabilizing = true;
					}
				}
			}
			DetermineSpeed();

      pidSpeed.Compute(now);
      pidHeading.Compute(now);
      pidInclination.Compute(now);
			if (!stabilizing)
      	ApplyControls();
			prevTimeStamp = now;
    }
  }

  //
  // Control methods
  //
  void setSpeed(double speed)
  {
    targetSpeed = speed;
  }
	void setHeading(double heading)
  {
    targetHeading = heading;
  }
	void setInclination(double incl)
  {
    targetInclination = incl;
  }

  //
  // control parameters
  //
  void setInclinationPID(double P, double I, double D)
  {
    pidInclination.SetTunings(P, I, D);
  }
  void setSpeedPID(double P, double I, double D)
  {
    pidSpeed.SetTunings(P, I, D);
  }
  void setHeadingPID(double P, double I, double D)
  {
    pidHeading.SetTunings(P, I, D);
  }

  //
  // Diagnostics methods
  //
  double getRollRadians()
  {
    return filter.getRollRadians();
  }
  double getYawRadians()
  {
    return filter.getYawRadians();
  }
  double getPitchRadians()
  {
    return filter.getPitchRadians();
  }

  String formatString(float current, float target)
  {
    char tmp[64];
    sprintf(tmp, "%.03f - %.03f", current, target);
    return String((char *)tmp);
  }
  String getInclination()
  {
    return formatString(currentInclination, targetInclination);
  }

  String getHeading()
  {
    return formatString(currentHeading, targetHeading);
  }

  String getSpeed()
  {
    return formatString(currentSpeed, targetSpeed);
  }

#ifdef USE_LOGGING
	size_t dump_log(log_t* data, size_t size)
	{
		if (logging) return 0;

		dumping = true;
		size_t i = 0;
		size_t real_size = min(log_size, size);

		for (i = 0; i < real_size; i++)
		{
			memcpy(data + i, log + (log_tail + i) % LOG_SIZE, sizeof(log_t));
		}
		log_tail = (log_tail + real_size) % LOG_SIZE;
		log_size -= real_size;

		dumping = false;
		return real_size;
	}
#endif

private:
  double softcap(double x)
  {
    //double ex = exp(5 * x);
    //return 2.0 * ex / (1.0 + ex) - 1;
		if (x > 1.0)
			return 1.0;
		else if (x < -1.0)
			return -1.0;
		else
			return x;
  }

  void ApplyControls()
  {
		double motor = softcap(fwdControl);
		double eps = .01;
		if (motor > eps)
			motor += .17;
		else if (motor < -eps)
			motor -= .17;
    ddrive.setDiffSpeed(motor, softcap(headingControl));
  }

  void DetermineSpeed()
  {
		double alpha = .01;
		double drive_speed = fwdControl; //ddrive.getSpeed();
    currentSpeed = currentSpeed * (1.0 - alpha) + drive_speed * alpha;
  }

  int DetermineOrientation()
  {
    Accelerometer *acc = Sensors::getAccelerometer();
    Gyroscope *gyro = Sensors::getGyroscope();

    Vector3 a, r;

    if (acc)
      a = acc->getAcceleration();
    if (gyro)
      r = gyro->getRotation();

    filter.updateIMU(r.x, r.y, r.z, a.x, a.y, a.z);

		//double alpha = .5;
		//double roll = getRollRadians();
    //currentInclination = (1 - alpha) * currentInclination + roll * alpha;
    currentInclination = getRollRadians();

    return 0;
  }
};
