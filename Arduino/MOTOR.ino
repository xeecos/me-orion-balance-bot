void runMotor(char type, int speed)
{
  if (type)
  {
    encoder.setPWM(0, -speed);
  }
  else
  {
    encoder.setPWM(1, speed);
  }
}

