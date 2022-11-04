#define sensorNum 8          //always even



#define linegapSpeed 255
int maxSpeed = 150; //180  //180 //200
int sSpeed = 150;

int blackLimit[sensorNum];
int digitalReading[sensorNum];

const int motorPin1=10,motorPin2=9;        //right motor
const int motorPin3=6,motorPin4=5;       //left motor
const int initIR=0;

float error, prevError=0;
float mappedValue, targetValue = 7;      


int frontDistance = 1000;
int frontTrig = 2;
int frontEcho = 3;

float safety=0.35;
int bDelay = 25;
float kp=40;   //26.7G //26        //38 //37  //39                //45 IF DOESN'T WORK
float kd=60;  //57.3  //50G //50    //67             //70  //42 //87

float agg_kp = 26.7;
float agg_kd = 57.3;

int rotationSpeed=100;
int gCount = 0;
int motorResponse;
float correction;

int leftSpeed,rightSpeed;

float lastAct;
int time=5;

int allBlack=0;
int leftBlack=0;
int rightBlack=0;

int blackSurfaceKey = 0;

int frontIR= 12;                      //change
int leftIR=2;                          //change
int rightIR=3;                         //change

int sonarTimer=0;
int sonarActive=0;

int led = 13;
int prev, curr, diff;
int danceDelay = 500;
int gapDelay = 1500; 
int obsKey = 0; 

int left=0,right=0; 
int preLeft=0, preRight=0;

int contiBlack = 0;
int stopTrigger = 3 ; //2
int startTrigger = 3;
int extremeCounter = 0;
int extremeTrigger = 3;
int preExtremeCounter = 0;
int preExtremeTrigger = 3;


int aggressiveCounter = 0;
int aggressiveTrigger = 250 ;

void setup()
{
  //initialize IR pins
  for(int i = initIR; i < sensorNum; i++)          //A0 is leftmost Ir
  {
    pinMode(A0 + i, INPUT);
  }

  pinMode(frontIR,INPUT);
  pinMode(rightIR,INPUT);
  pinMode(leftIR,INPUT);

  pinMode(led, OUTPUT);

  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);  


  //initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  delay(1000);
  Serial.begin(9600);
  
  Serial.println("Calibrating");
  calibration();
  
  
}





void loop()
{

//led code
if(!left && !right)
 digitalWrite(led, LOW);
else
  digitalWrite(led, HIGH);


if(aggressiveCounter > aggressiveTrigger)
{
  kp = agg_kp;
  kd = agg_kd;
  maxSpeed = 180;
  bDelay = 75;
}


//obstacle code

if(obsKey == 1)
{
  if(digitalRead(frontIR) == 0)
  {
    motor(-sSpeed, -sSpeed);
    delay(300);
    leftCurveRun();
    maxSpeed = 180;
    sSpeed= 150;
    obsKey = 0;
  }


//  obstacleSonar();
    

}


sensorMapping();
Serial.print(" ");
if(stopTrigger < contiBlack) 
{ 
  while(allBlack == 1) {brake(); startTrigger = 0; sensorMapping();}
   //if(mappedValue == 100) {delay(100); sensorMapping(); if(mappedValue == 100) brake();}
}

for(int i = 0; i < sensorNum; i++)
{
  //Serial.print(analogRead(A0+i));
  /*if(analogRead(A0+i)<blackLimit[i])
     Serial.print("1");
  else
     Serial.print("0");   
  Serial.print(" ");*/ 
}

  Serial.print("left: ");
  Serial.print(left);
  Serial.print(" right: ");
  Serial.print(right);
  Serial.println();
 /*
  while(!digitalRead(frontIRpin))    // if reading 0 object is close
  {
  	brake();
  }
 */ 
 
//inverse run
if(blackSurfaceKey==1)
{
  motor(maxSpeed,maxSpeed);
  delay(200);
  brake();
  sensorMapping();
  while( gCount > 3  )
  {
    blackSensorMapping();
    pid();
    motor(leftSpeed,rightSpeed);
    sensorMapping();
    
  }
  brake();
  delay(200);

  obsKey = 1;
  maxSpeed = 130;
  sSpeed = 120;
}







 
  if(mappedValue!=100 || (contiBlack > stopTrigger && allBlack == 1))     // add conditions                    
    {
      pid();      
      motor(leftSpeed,rightSpeed);
      
    }
  else                                 // all white
    {
        brake();
        //delay(10);
        //delayMicroseconds();
        //dronzer stop
        motor(-100,-100);
        delay(bDelay);
        
        if((left==1)&&(right==0))
        {
          ACRotate(rotationSpeed);   
          while(mappedValue==100) sensorMapping();
          while(digitalReading[initIR+2]!=1)
          {
            sensorMapping();
          pid();
          motor(leftSpeed,rightSpeed);
          }      
           left=0;                                     //changed
        }
        else if((right==1)&&(left==0))
        {
          CRotate(rotationSpeed);   
          while(mappedValue==100) sensorMapping();
          while(digitalReading[sensorNum-3]!=1)
          {
            sensorMapping();
          pid();
          motor(leftSpeed,rightSpeed);
          }
          right=0;                                     //changed
        }
        else
        {
          if((preLeft==1)&&(preRight==0))
        {
          digitalWrite(led, HIGH);
          ACRotate(rotationSpeed);   
          while(mappedValue==100) sensorMapping();
          while(digitalReading[initIR+2]!=1)
          {
            sensorMapping();
          pid();
          motor(leftSpeed,rightSpeed);
          }      
           left=0;                                     //changed
        }
        else if((preRight==1)&&(preLeft==0))
        {
          digitalWrite(led, HIGH);
          CRotate(rotationSpeed);   
          while(mappedValue==100) sensorMapping();
          while(digitalReading[sensorNum-3]!=1)
          {
            sensorMapping();
          pid();
          motor(leftSpeed,rightSpeed);
          }
          right=0;                                     //changed
        }

          else
          {
          forwardDance();
          }
        }

      
    }
      Serial.print("left");
      Serial.print(left);
      Serial.print(" Right: ");    
      Serial.print(right);

Serial.println();
}


void sensorMapping()
{
int sum=0,coun=0;
gCount = 0;




 
 for (int i = initIR; i <sensorNum; i++)
  { 
    if (analogRead(A0+i) < blackLimit[i])              //A0 is leftmost IR 
     { 
      if(i > 0 && i < 7)     
      {  
        sum += i*2;
        coun++;
      }
      gCount++;
      digitalReading[i]=1;
    }
    else digitalReading[i]=0;
  }
 

   //if((left==1)&&(right==1)) insideLoop=1;             //loop detection
   
   if(coun!=0){                   
  mappedValue = sum / coun;
   }
   else mappedValue=100;         //all White detection 


 
    //reset corner IRs
  if(extremeCounter > extremeTrigger)
  {
    left = 0;
    right = 0;
  }
  
  //reset secondary corner IRs
  if(preExtremeCounter > preExtremeTrigger)
   {
     preLeft = 0;
     preRight = 0;
   }
  if(digitalReading[0]||digitalReading[sensorNum-1])  //if left or right gives black saves their value
   {
     left=digitalReading[0];
     right=digitalReading[sensorNum-1];
     extremeCounter = 0; //reset counter
   }
   
   else 
   {
     if(mappedValue != 100)
     extremeCounter++;
   }
   
   if(digitalReading[1] || digitalReading[sensorNum-2])
   {
     preLeft = digitalReading[1];
     preRight = digitalReading[sensorNum-2];
     preExtremeCounter = 0;
   }
   
   else
   {
     if(mappedValue != 100)
     preExtremeCounter++;
   }

   if(coun==sensorNum - 2) 
   {
    contiBlack++;
   
   allBlack=1;
   
 
   }
   else if(startTrigger == 0 && mappedValue != 7 && coun != sensorNum)
   {
    allBlack = 1;
   }

    else
    {
    allBlack=0; 
    contiBlack = 0;  
    }

  //inverse detection
  if ( digitalReading[0]==1 && digitalReading[1]==1 && digitalReading[6]==1 && digitalReading[7]==1 && (digitalReading[3]==0 || digitalReading[4]==0) )
  {
    blackSurfaceKey = 1;
  }
  else
  {
    blackSurfaceKey = 0;
  }

   
}

void blackSensorMapping()
{
  int sum=0,pidDetect=0, stopDetect=0;   
   for (int i = 0; i <sensorNum; i++)
  { 
    
    if (analogRead(A0+i) > blackLimit[i])           
     { 

      if(i>1 && i<6) 
      {sum += i*2; pidDetect++;}

     } 
   }  

   if(pidDetect!=0){  
    mappedValue = sum / pidDetect;
   }
   else mappedValue=100;  
}


void pid()
{
  
  error=targetValue-mappedValue;
  correction=(kp*error)+(kd*(error-prevError));
  prevError=error;
  motorResponse=(int)correction;
 
 if(motorResponse>maxSpeed) motorResponse=maxSpeed;
 
if(motorResponse<-maxSpeed) motorResponse=-maxSpeed;

   if(motorResponse>0)
  {
    rightSpeed=maxSpeed;
    leftSpeed=maxSpeed-motorResponse;
  }
  else if(motorResponse<0)
  {
    rightSpeed=maxSpeed+ motorResponse;
    leftSpeed=maxSpeed;
  }
  //non aggressive straight
  else
  {
    rightSpeed= sSpeed;
    leftSpeed= sSpeed;
  }

}


void motor(int left, int right)
{
  
  if(right>0)
  {
  analogWrite(motorPin1,right);
  analogWrite(motorPin2,0);
  }
  else
  {
    analogWrite(motorPin1,0);
    analogWrite(motorPin2,-right);
  }

  if(left>0)
  {
  analogWrite(motorPin3,left);
  analogWrite(motorPin4,0);
  }
  else
  {
   analogWrite(motorPin3,0);
   analogWrite(motorPin4,-left); 
  }

 }

void CRotate(int rotateSpeed)
{
  analogWrite(motorPin1,0);
  analogWrite(motorPin2, rotateSpeed);
  analogWrite(motorPin3, rotateSpeed);
  analogWrite(motorPin4,0);

}

void ACRotate(int rotateSpeed)
{
  analogWrite(motorPin1,rotateSpeed);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4,rotateSpeed);

}

void brake(void)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
}



void forwardDance(void)
{

int loopCounter = (int) (danceDelay / diff);
int gapSearch = (int) (gapDelay/diff);

  for(int i = gapSearch; i > 0; i--)
  {
    motor(150,150);
    
    if(mappedValue != 100)
    {
      pid();
      motor(leftSpeed, rightSpeed);
      return;
    }
     
  }
  
  
  
  for(int i = loopCounter; i > 0; i--)
  {

  CRotate(rotationSpeed);
  sensorMapping();

   if(mappedValue != 100)
   {
    pid();
    motor(leftSpeed, rightSpeed);
    return;
   }

  }

  while(1)
  {
  ACRotate(rotationSpeed);
  sensorMapping();

   if(mappedValue != 100)
   {
    pid();
    motor(leftSpeed, rightSpeed);
    return;
   }

  }
  
}





long mstocm(long microseconds)
{
 
  return (microseconds*346.3)/2/10000;
}

// variables used triggered time oldTime interruptVariable distance

long trigger(int trigPin,int echoPin)
{
  unsigned long oldTime;
  unsigned long time;
  unsigned long timeLimit=2000;

  digitalWrite(echoPin,LOW);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  
  while(digitalRead(echoPin)==LOW); 
  
  oldTime=micros();
  while(digitalRead(echoPin)==HIGH&&(micros()-oldTime<timeLimit));
  time=micros()-oldTime;

  
  long distance =  mstocm(time);

  Serial.print(" trigger Time: ");
  //Serial.println(distance);
  
  return distance;
}

void triggerSonars()
{
  Serial.println("trigger start");
  int sonarDelay=10;  //use 50 if 50 works fine
  


  
  if(millis()-sonarTimer<sonarDelay)
   delay(sonarDelay-millis()-sonarTimer);

  frontDistance=trigger(frontTrig,frontEcho);

  
  sonarTimer=millis();
  Serial.println("trigger end");  
}


//inverse code

void inverseRun(void)
{
      if(digitalReading[0] == 1 && (digitalReading[3] == 0 || digitalReading[4] == 0) && digitalReading[7] == 1)
    {
      do
      {
        motor(sSpeed, sSpeed);
        sensorMapping();
      } while(gCount < 4);

      pid();
      motor(leftSpeed, rightSpeed);
      
    }
}

void inverseRun2(void)
{
      if(digitalReading[0] == 1 && digitalReading[1] == 1 &&  (digitalReading[3] == 0 || digitalReading[4] == 0) && digitalReading[6] == 1 && digitalReading[7] == 1)
    {
      do
      {
        motor(sSpeed, sSpeed);
        sensorMapping();
      } while(gCount < 4);

      pid();
      motor(leftSpeed, rightSpeed);
      
    }
}


void inverseRun2Key(void)
{
      if(digitalReading[0] == 1 && digitalReading[1] == 1 &&  (digitalReading[3] == 0 || digitalReading[4] == 0) && digitalReading[6] == 1 && digitalReading[7] == 1)
    {
      do
      {
        motor(sSpeed, sSpeed);
        sensorMapping();
      } while(gCount < 4);

      pid();
      motor(leftSpeed, rightSpeed);
      
    }

    obsKey = 1;
}

//curve run

void leftCurveRun()
{
  motor(-100,-100);
  delay(200);
  ACRotate(rotationSpeed);
  delay(130);
  motor(200,130);
  delay(500);
  sensorMapping();
  while(mappedValue==100)
  {
    motor(200,130);
    sensorMapping();
  }
}

void rightCurveRun()
{
  motor(-100,-100);
  delay(200);
  CRotate(rotationSpeed);
  delay(130);
  motor(130,200);
  delay(500);
  sensorMapping();
  while(mappedValue==100)
  {
    motor(130,200);
    sensorMapping();
  }
}


void obstacleSonar(void)
{
 if(obsKey==1)
{
  
  triggerSonars();
  
  //if (frontDistance<20 && frontDistance!=0)
  //{
  //  Serial.println("Leftsonar got reading");
  //  brake();
  //  delay(500);
  //  curveRun();
  //  triggerSonars();
  //
  //}
    
  if (frontDistance<20 && frontDistance >5)
  {
    sonarActive = 1;
    brake();
    delay(50);
    for(int i=0;i<3;i++)
    {
      delay(10);
      triggerSonars();
      if ( frontDistance>20 )
      {
        sonarActive=0;
      }
      
    }
  
    if(sonarActive==1)
    {
    motor(-sSpeed, -sSpeed);
    delay(300);
    leftCurveRun();
    maxSpeed = 180;
    sSpeed= 150;
    obsKey = 0;
    }
    else
    {
      sensorMapping();
      pid();
      motor(leftSpeed,rightSpeed);
    }
     
  }
} 
}


 //auto calibration
void calibration()
{
  CRotate(50);
  float upSum = 0,lowSum = 0;
  int sensorArray[sensorNum][2];

  for(int i = initIR; i < sensorNum; i++)
    {
      sensorArray[i][0] = analogRead(A0+i);
      sensorArray[i][1] = analogRead(A0+i);
    }
 

  int loopCounter = (int)(time * 1000 / 2.5);  
  while(loopCounter)
  {
    for(int i = initIR; i < sensorNum; i++)
    {
      if(analogRead(A0+i)<sensorArray[i][0]) sensorArray[i][0]=analogRead(A0+i);
      if(analogRead(A0+i)>sensorArray[i][1]) sensorArray[i][1]=analogRead(A0+i);
    }
  loopCounter--;

  }

 for(int i=initIR; i < sensorNum; i++)
  blackLimit[i] = (int)(sensorArray[i][0] + safety * (sensorArray[i][1] - sensorArray[i][0]));
  
  prev = millis();
  sensorMapping ();
  curr= millis();
  diff = curr - prev;

  brake();
  delay(1000);

}
