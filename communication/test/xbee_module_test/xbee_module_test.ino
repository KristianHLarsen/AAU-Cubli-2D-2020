
void setup()
{
  Serial1.begin(115200);
  Serial.begin(115200);
}

void loop()
{
  delay(1000);
  Serial1.write('f');
  if (Serial.available())
  { 
    Serial1.write(Serial.read());
      }
  if (Serial1.available())
  { 
    Serial.write(Serial1.read());
      }
}

    
