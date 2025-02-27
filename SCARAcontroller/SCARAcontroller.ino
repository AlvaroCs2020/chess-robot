#include <math.h>
#include <Servo.h>
struct Angles {
    float theta1;
    float theta2;
};
const float LENGTH1 = 150.0;
const float LENGTH2 = 200.0;
void inverseKinematics(float x, float y, float Z, float angles[], float length0,float length1)
{
  float jointAngle2 = atan(Z/x) * (180/PI);
  
  //Rotate point to xy plane
  
  float newX = sqrt(Z*Z + x*x);
  float newY = y;

  float length2 = sqrt(newX*newX + newY*newY);
  
  float cosAngle0 = ((length2 * length2) + (length0 * length0) - (length1 * length1)) / (2 * length2 * length0);

  float angle0 = acos(cosAngle0) * (180/PI);

  float cosAngle1 = ((length1 * length1) + (length0 * length0) - (length2 * length2)) / (2 * length1 * length0);

  float angle1 = acos(cosAngle1) * (180/PI);
  float atang = atan(newY/newX) * (180/PI);
  float jointAngle0 = atang+angle0 ;// Angle shoulder
  float jointAngle1 = 180.0   - angle1;// Angle elbow
  
  if(newX<0)
    jointAngle0= abs(180+jointAngle0);
  
  //Out of range check
  
  if(length0+length1 <length2)
  {
    jointAngle0 = atang;
    jointAngle1 = 0;
  }
  //Return
  
  angles[0] = jointAngle0;
  angles[1] = jointAngle1+ 90;

}

class RobotController {
    private:
    int pinDirFst;
    int pinDirScn;
    int pinDirThr;
    int pinStepFst;
    int pinStepScn;
    int pinStepThr;
    float angleFst;
    float angleScn;
    Servo degreeServo;
    Servo gripperServo;
    public:
    RobotController(int pDirFst, int pDirScn, int pDirThr)
    {
      pinDirFst = pDirFst;
      pinDirScn = pDirScn;
      pinDirThr = pDirThr;
      pinStepFst = pDirFst + 1;
      pinStepScn = pDirScn + 1;
      pinStepThr = pDirThr + 1;
      pinMode(pinDirFst, OUTPUT);
      pinMode(pinDirScn, OUTPUT);
      pinMode(pinDirThr, OUTPUT);
      pinMode(pinStepFst, OUTPUT);
      pinMode(pinStepScn, OUTPUT);
      pinMode(pinStepThr, OUTPUT);
      digitalWrite(pinDirFst, LOW);
      digitalWrite(pinStepFst, LOW);
      digitalWrite(pinDirScn, LOW);
      digitalWrite(pinStepScn, LOW);
      angleFst = 90.0;
      angleScn = 90.0;
      //Servo Handling
      //degreeServo.attach(9,600,2400);
      degreeServo.write(0);
      Serial.begin(9600);
      Serial.println(String(pinDirFst));
    }
    void setHomeAngles()
    {
      this->angleFst = 90.0;
      this->angleScn = 90.0;
    }
    void moveFst(float angle, int dir)
    {
      float reduction = 0.6;
      int steps = angle/(0.9*reduction);
      Serial.print("El motor 1 se movera: ");
      Serial.println(steps);
      digitalWrite(pinDirFst, dir);		// giro en un sentido
      for(int i = 0; i < steps; i++){   	// 200 pasos para motor de 0.9 grados de angulo de paso
        digitalWrite(pinStepFst, HIGH);     	// nivel alto
        delay(10);			  	// por 10 mseg
        digitalWrite(pinStepFst, LOW);      	// nivel bajo
        delay(10);			  	// por 10 mseg
      }
      int sign = dir == 0? 1: -1;
      this->angleFst = this->angleFst +steps*(0.9*reduction)*sign;
      
    }
    void moveScn(float angle, int dir)
    {
      float reduction = 0.6;
      int steps = angle/(0.9*reduction);

      Serial.print("El motor 2 se movera: ");
      Serial.println(steps);
      digitalWrite(pinDirScn, dir);		// giro en un sentido
      for(int i = 0; i < steps; i++){   	// 200 pasos para motor de 0.9 grados de angulo de paso
        digitalWrite(pinStepScn, HIGH);     	// nivel alto
        delay(10);			  	// por 10 mseg
        digitalWrite(pinStepScn, LOW);      	// nivel bajo
        delay(10);			  	// por 10 mseg
      }     
      int sign = dir == 0? -1: 1;
      this->angleScn = this->angleScn +steps*(0.9*reduction)*sign;
    }
    void goTo(float thetaFst, float thetaScn)
    {
      Serial.println("sss");
      float delta = thetaFst - this->angleFst;
      int dir = thetaFst > this->angleFst ? 0: 1;
      this->moveFst(abs(delta), dir);
      
      delta = thetaScn - this->angleScn;
      dir = thetaScn > this->angleScn ? 1: 0;
      this->moveScn(abs(delta), dir);
    }  
    void writeDegreeServo(int angle)
    {
      degreeServo.write(angle);
    }
};
RobotController robotController(4,6,8);
char receivedChar;
bool manual = 0;
//int initalPoint
boolean newData = false;
Servo servito;
Angles angles;
void setup() 
{
  pinMode(2,INPUT);
  servito.attach(9);
  servito.write(90);
  Serial.begin(9600);
  Serial.setTimeout(10);
}

void loop() {
  handleSerial();
  if(manual) manualMode();
}
void setManualMode()
{
  manual = !manual;
  //angles = computeAngles(0, 350);
}
int desiredPoint[2] = {0,350}; //Extendido
int currentPoint[2] = {0,350};
void updatePosition()
{
  float currentAngles[2];
  float desiredAngles[2]; 
  inverseKinematics(currentPoint[0], currentPoint[1], 0, currentAngles, 200, 150);
  inverseKinematics(desiredPoint[0], desiredPoint[1], 0, desiredAngles, 200, 150);
  float deltaAngle1 = desiredAngles[0] - currentAngles[0];
  float deltaAngle2 = desiredAngles[1] - currentAngles[1];
  int fstCuadrant = desiredPoint[0] > 0? 1:-1;
  int sense1 = (0>=deltaAngle1*fstCuadrant?1:0);
  int sense2 = (0>=deltaAngle2*fstCuadrant?1:0); 
  robotController.moveFst(abs(deltaAngle2),sense2);
  robotController.moveScn(abs(deltaAngle1),sense1);
  currentPoint[0] = desiredPoint[0];
  currentPoint[1] = desiredPoint[1];
  Serial.println("Angulos : "+String(deltaAngle1) + " : " + String(deltaAngle2));
}
int i = 0;
void manualMode()
{
  int potX = analogRead(A0);
  int potY = analogRead(A1);
  if (potX > 800) 
  {
    desiredPoint[0]--;Serial.println("Punto : "+String(desiredPoint[0]) + " : " + String(desiredPoint[1]));
  }
  else if (potX < 200) 
  {
    desiredPoint[0]++;Serial.println("Punto : "+String(desiredPoint[0]) + " : " + String(desiredPoint[1]));
  }
  else if (potY > 800) 
  {
    desiredPoint[1]--;Serial.println("Punto : "+String(desiredPoint[0]) + " : " + String(desiredPoint[1]));
  }
  else if (potY < 200) 
  {
    desiredPoint[1]++;Serial.println("Punto : "+String(desiredPoint[0]) + " : " + String(desiredPoint[1]));
  }
  delay(80);

  //String position = String(desiredPoint[])
  //Serial.println("")
  
  if(digitalRead(2) == LOW)
  {
    updatePosition();
  }

  //float angulos[2];
  //inverseKinematics(desiredPoint[0], desiredPoint[1], 0, angulos, 200, 150);
  //Angles deltaAngles = computeAngles(desiredPoint[0], desiredPoint[1]);
  //Serial.println("Punto : "+String(desiredPoint[0]) + " : " + String(desiredPoint[1]));
  //Serial.println("Angulos : "+String(angulos[0]) + " : " + String(angulos[1]));


}

void handleSerial()
{
  // send data only when you receive data:
  if (Serial.available() > 0) {

      //el mensaje va a tener el formato "+{codigo};{angulo}-" "{1;30;1}"
      String command =  Serial.readStringUntil(';');
      if(command.begin()[0] != '{') return;
      int x = command.substring(1).toInt();
      command =  Serial.readStringUntil(';');
      float y = command.toFloat();
      int Y = command.toInt();
      command =  Serial.readStringUntil('}');
      int Z = command.toInt();
      float z = command.toFloat();
      //(if(!(Z == 1 || Z == 0|| Z == 2) ) return; //data check
      if(Z == 1 && x == 1 && Y == 1 ) 
      {
        robotController.setHomeAngles();
        return;
      }
      //if(Z == 2 && x == 2 && Y == 2 ) // Enter ManualMode
      //{
      //  Serial.println("x Recibi el mensaje lokura, aca esta la respuesta");
      //  return;
      //}
      //if(Z != 1) return;
      //if(Z != 1 && Z != 0) return;
      Serial.println(String(x) + " :  " + String(y)+ " :  " + String(z));
      switch(x){
        case 1:
          robotController.moveFst(y,Z);
          break;
        case 2:
          robotController.moveScn(y,Z);
          break;
        case 3: 
          robotController.goTo(y, z);
          break;
        case 4://1 abrir 0 cerrar
          int sense = 0;
          if(Z==0)sense = 180;
          servito.write(sense);
          delay(Y);
          servito.write(90);
          break;
      }
      //Serial.println(String(x) + " : " + String(y) + " : " + String(Z));
      //mapCoordinatesToAngles(x,y,Z);
  }
}
