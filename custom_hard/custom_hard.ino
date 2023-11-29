int nbr_joints = 8;
int analogPin = A0;

float val = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);

}

void loop() {
  // put your main code here, to run repeatedly:
  val = analogRead(analogPin);
  val = mapfloat(val, 0.0, 500.0, 0.0, 3.14);
  if(Serial.available())
  {
    
    char c = Serial.read();
    if (c == 'r')
    {
      float j[] = {3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14};
      j[0] = val;
      j[1] = val - 0.5;
      Serial.write((char*)j, nbr_joints * sizeof(float));
    }
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
