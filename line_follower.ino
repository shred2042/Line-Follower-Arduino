int MOTOR1_PIN1 = 3;
int MOTOR1_PIN2 = 5;
int MOTOR2_PIN1 = 6;
int MOTOR2_PIN2 = 9;

int sensor[8] = {4, A5, A4, A3, A2, A1, A0, 2};//{ 2,A0,A1,A2,A3,A4,A5,4 }; 
int weights[8] = {-20, -10, -5, -1, 1, 5, 10, 20};
int sensorReading[8] = { 0 }; 
float activeSensor = 0; 
float totalSensor = 0;
//float avgSensor = 
float avgSensor = 0; // Average sensor reading

float Kp = 3;   // Max deviation = 10 - 0 = 10 ||  255/10 = 25
float Ki = 0.00015;
float Kd = 2;

float error = 0;
float previousError = 0;
float totalError = 0;

float power = 0;

int PWM_Left;
int PWM_Right;

int MAX_SPEED = 50;

void setup() {
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  for(int i=0; i<8; i++) {
    pinMode(sensor[i], INPUT);
  }
  Serial.begin(9600);
}

void loop() {
    /*go(255,-100);
    delay(1000);
    go(0,0);
    delay(1000);*/
    
   lineFollow();
   
}

void lineFollow(void) {
   PID_program();
   go(PWM_Left,PWM_Right);//viteza negativa
}
void PID_program()
{ 
    readSensor();
    
    previousError = error; // save previous error for differential 
    error = avgSensor; // Count how much robot deviate from center
    totalError += error; 
    Serial.print("error=");
    Serial.println(error);
    power = (Kp*error) + (Kd*(error-previousError));// + (Ki*totalError);
    //scos integrala
    if( power>MAX_SPEED ) { power = MAX_SPEED; }
    if( power<-MAX_SPEED ) { power = -MAX_SPEED; }//aici putem lasa power in asa fel incat una sa fie negativa
    Serial.print("POWER=");
    Serial.println(power);
    if(power<0) // Turn left
    {
      PWM_Right = MAX_SPEED;// + abs(int(power));
      PWM_Left = MAX_SPEED - abs(int(power));
    }
    
    else // Turn right
    {
      PWM_Right = MAX_SPEED - int(power);
      PWM_Left = MAX_SPEED;// + abs(int(power));
    }
    Serial.print("PWM_Right=");
    Serial.print(PWM_Right);
    Serial.println(" ");
    Serial.print("PWM_Left=");
    Serial.print(PWM_Left);
    Serial.println(" ");
    
    //delay(200);
}


void go(int speedLeft, int speedRight) {
  if (speedLeft > 0) {
    analogWrite(MOTOR1_PIN1, speedLeft);
    analogWrite(MOTOR1_PIN2, 0);
  } 
  else {
    analogWrite(MOTOR1_PIN1, 0);
    analogWrite(MOTOR1_PIN2, -speedLeft);
  }
 
  if (speedRight > 0) {
    analogWrite(MOTOR2_PIN1, speedRight);
    analogWrite(MOTOR2_PIN2, 0);
  }else {
    analogWrite(MOTOR2_PIN1, 0);
    analogWrite(MOTOR2_PIN2, -speedRight);
  }
}
void readSensor() {
  for(int i=1; i<7; i++) 
    {
       if(analogRead(sensor[i])>200){
         sensorReading[i]=1;
       }
    }
    
    sensorReading[0] = digitalRead(sensor[0]);
    sensorReading[7] = digitalRead(sensor[7]);
    /*Serial.print("senzori: ");
    for(int i=0; i<8; i++) 
    {
      Serial.print(sensorReading[i]);
      Serial.print(" ");
    }
    Serial.println(" ");
    */
    for(int i=0; i<8; i++) 
    {
      if(sensorReading[i]==1) { activeSensor+=1; }
      totalSensor += sensorReading[i] * weights[i];
    }  
    avgSensor = totalSensor/activeSensor;
    /*Serial.print("avgSensor=");
    Serial.println(avgSensor);
    */
    activeSensor = 0; totalSensor = 0;
    for(int i=0; i<8; i++) 
    {
      sensorReading[i]=0;
    }
}
