/*MIT License

Copyright (c) 2024 JD edu. http://jdedu.kr author: conner.jeong@gmail.com
     
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
     
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
     
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN TH
SOFTWARE.*/

#ifndef TB6612_MOTOR
#define TB6612_MOTOr
// install Encoder library 
// https://github.com/PaulStoffregen/Encoder

#include<Encoder.h>

class SPDMotor {
  public:
  SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 );
  /// Set the PWM speed and direction pins.
  /// pwm = 0, stop (no active control)
  /// pwm = 1 to 255, proportion of CCW rotation
  /// pwm = -1 to -255, proportion of CW rotation
  void speed( int pwm );
  /// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
  void hardStop();
  /// Get the current speed.
  int getSpeed();
  /// Get the current rotation position from the encoder.
  long getEncoderPosition();
  private:
    Encoder *_encoder;
    bool _encoderReversed;
    int _motorPWM, _motorDir1, _motorDir2;
    // Current speed setting.
    int _speed;
};

#endif