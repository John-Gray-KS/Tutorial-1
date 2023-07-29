// Tail light definitions
#define Left_Tail_Light 40
#define Right_Tail_Light 41
unsigned long previousTime;
unsigned long blinkTime;

void tail_Lights_On(){
  pinMode(Left_Tail_Light, OUTPUT);
  pinMode(Right_Tail_Light, OUTPUT);
  digitalWrite(Left_Tail_Light, HIGH);
  digitalWrite(Right_Tail_Light, HIGH);
}

unsigned long left_tail_light_blink(unsigned long previousTime){
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= 500) {
    digitalWrite(Left_Tail_Light, HIGH);
    digitalWrite(Right_Tail_Light, LOW);
    previousTime = currentTime;
  }
else {
  digitalWrite(Left_Tail_Light, LOW);
  digitalWrite(Right_Tail_Light, LOW);
  }
  return previousTime;
}

unsigned long right_tail_light_blink(unsigned long previousTime){
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= 500) {
    digitalWrite(Left_Tail_Light, LOW);
    digitalWrite(Right_Tail_Light, HIGH);
    previousTime = currentTime;
  }
else {
  digitalWrite(Left_Tail_Light, LOW);
  digitalWrite(Right_Tail_Light, LOW);
  }
  return previousTime;
}

unsigned long both_tail_light_blink(unsigned long previousTime){
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= 500) {
    digitalWrite(Left_Tail_Light, HIGH);
    digitalWrite(Right_Tail_Light, HIGH);
    previousTime = currentTime;
  }
else {
  digitalWrite(Left_Tail_Light, LOW);
  digitalWrite(Right_Tail_Light, LOW);
  }
  return previousTime;
}