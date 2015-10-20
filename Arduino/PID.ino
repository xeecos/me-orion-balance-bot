void PID_revalue(void)
{
  /*if (FLAG & MOVING)
  {
    PID_speed.P = 0.025;
    PID_speed.I = 0.0012;
  }
  else
  {*/
    //PID_speed.P = 0.022;
    //PID_speed.I = 0.001;
    PID_speed.P = 0.048;
    PID_speed.I = 0.0008;
  //}
}

void PID_angle_compute(void)
{
  double dt = micros() - PID_angle.Timer ;
  if (dt > 5000)
  {
    double error = compAngleY - PID_angle.Setpoint ;
    PID_angle.Integral += error * dt * 0.001;
    PID_angle.Integral = constrain(PID_angle.Integral, -255, 255);

    PID_angle.Output = PID_angle.P * error + PID_angle.I * PID_angle.Integral - PID_angle.D * gyroY;

    double pwm_left = PID_angle.Output + PID_turn.Output ;
    double pwm_right = PID_angle.Output - PID_turn.Output ;
    int spd = 250;
    pwm_left = constrain(pwm_left, -spd, spd);
    pwm_right = constrain(pwm_right, -spd, spd);

#ifdef MOTOR_ENABLE
    runMotor(0, pwm_left);
    runMotor(1, pwm_right);
#endif

    PID_angle.Timer = micros();
  }
}

void PID_speed_compute(void)
{
  double dt = micros() - PID_speed.Timer ;
  if (dt > 50000)
  {
    double speed_now = encoder.getSpeed(1) - encoder.getSpeed(0);
    double error = speed_now - PID_speed.Setpoint ;
    PID_speed.Integral += error;

    PID_speed.Output = PID_speed.P * speed_now + PID_speed.I * PID_speed.Integral ;
    PID_speed.Output = constrain(PID_speed.Output , -35, 35);
    PID_angle.Setpoint =  RELAX_ANGLE + PID_speed.Output;

    PID_speed.Timer = micros();
    Serial.println(PID_speed.Output);
  }
}

void PID_turn_compute(void)
{
  double dt = micros() - PID_turn.Timer ;
  if (dt > 50000)
  {
    double error = 0;
    if (LED_3)
      error = 1;
    else if (LED_4)
      error = 3;
    else if (LED_2)
      error = -1;
    else if (LED_1)
      error = -3;
    //Serial.println(error);

    double delta = error - PID_turn.last_error;
    PID_turn.last_error = error;

    PID_turn.Output = PID_turn.P * error + PID_turn.D * delta ;

    PID_turn.Timer = micros();
  }
}
