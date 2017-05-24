

int ENA = 10;
int IN1 = 4;
int IN2 = 5;
int ENB = 11;
int IN3 = 6;
int IN4 = 7;

void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop()
{
  // Turn off motor A & B
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  // Delay
  delay(1000);
  
  // Turn on motor A
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Turn on motor B
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  for (int i = 0; i < 200; i++)
  {
    analogWrite(ENA, i);
    analogWrite(ENB, i);
    delay(50);
  }
  
  // Delay
  delay(1000);
  
  // Turn off motor A & B
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  // Delay
  delay(1000);
  
  // Turn on motor A reversed
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  // Turn on motor B reversed
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  for (int i = 0; i < 200; i++)
  {
    analogWrite(ENA, i);
    analogWrite(ENB, i);
    delay(50);
  }
  
  // Delay
  delay(1000);
  
}

