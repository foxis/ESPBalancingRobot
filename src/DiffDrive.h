#include "Arduino.h"


class DiffDrive
{
  int _AA, _AB, _BB, _BA;
  int _motorConst;
  double _wheelBase;

  double _vTarget;
  double _wTarget;

  public:
    DiffDrive(int AA, int AB, int BA, int BB, double wheelBase)
      :DiffDrive(AA, AB, BA, BB, PWMRANGE + 1, wheelBase)
    {

    }

    DiffDrive(int AA, int AB, int BA, int BB, int motorConst, double wheelBase)
    {
      _AA = AA;
      _BB = BB;
      _AB = AB;
      _BA = BA;
      _motorConst = motorConst;
      _wheelBase = wheelBase;

      _vTarget = 0;
      _wTarget = 0;
    }

    void init()
    {
      pinMode(_AA, OUTPUT);
      pinMode(_AB, OUTPUT);
      pinMode(_BA, OUTPUT);
      pinMode(_BB, OUTPUT);
      setSpeed(0, 0);
    }

    /// power - differential drive train power
    /// w - rotational power (normalized wL)
    /// all control parameters range -1.0 ... +1.0
    void setDiffSpeed(double power, double w)
    {
      _vTarget = power;
      _wTarget = w;
      double a = (2.0 * power + w) / 2.0;
      double b = (2.0 * power - w) / 2.0;
      setSpeed(a, b);
    }

    double getSpeed()
    {
      return _vTarget;
    }
    double getRotation()
    {
      return _wTarget;
    }

private:
    void setSpeed(double a, double b)
    {
      setLeftSpeed(a);
      setRightSpeed(b);
    }

    void setMotorSpeed(int A, int B, double a)
    {
      uint16_t motor = (uint16_t)abs(a * _motorConst);
      analogWrite(A, a > 0 ? motor : 0);
      analogWrite(B, a > 0 ? 0: motor);
    }

    void setLeftSpeed(double a)
    {
      setMotorSpeed(_AA, _AB, a);
    }
    void setRightSpeed(double b)
    {
      setMotorSpeed(_BA, _BB, -b);
    }
};
