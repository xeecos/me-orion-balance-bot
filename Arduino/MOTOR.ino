void MOTOR(char TYPE, int SPEED)
{
  if (TYPE)
  {
    encoder.setPWM(0, -SPEED);
  }
  else
  {
    encoder.setPWM(1, SPEED);
  }
}

