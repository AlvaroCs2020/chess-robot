#include <math.h>
#include <Servo.h>
struct Angles {
    float theta1;
    float theta2;
};

#define L1 200.0  // Longitud del primer link (en mm)
#define L2 150.0  // Longitud del segundo link (en mm)

// Función para resolver la cinemática inversa
bool InverseKinematics(float x, float y, float &theta1, float &theta2) {
    float d = sqrt(x * x + y * y);
    
    if (d > (L1 + L2) || d < fabs(L1 - L2)) {
        Serial.println("Punto fuera del alcance.");
        return false;
    }

    float cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (cosTheta2 < -1 || cosTheta2 > 1) {
        Serial.println("Error matemático en acos.");
        return false;
    }

    theta2 = acos(cosTheta2) * 180.0 / M_PI; // Convertir a grados

    float k1 = L1 + L2 * cosTheta2;
    float k2 = L2 * sin(acos(cosTheta2));

    theta1 = atan2(y, x) - atan2(k2, k1);
    theta1 = theta1 * 180.0 / M_PI; // Convertir a grados
    // Ajuste para mantener dentro del rango [0, 180]
    /*
    if (theta1 < 0 || theta1 > 180 || theta2 < 0 || theta2 > 180) {
        Serial.println("Ángulos fuera de rango.");
        return false;
    }
    */
    return true;
}
const float LENGTH1 = 150.0;
const float LENGTH2 = 200.0;
void inverseKinematics(float x, float y, float angles[], float length0,float length1)
{
  float newX = x;
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
      //degreeServo.attach(10);
      //degreeServo.write(90);
      Serial.begin(9600);
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
      //Serial.print("El motor 1 se movera: ");
      //Serial.println(steps);
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

      //Serial.print("El motor 2 se movera: ");
      //Serial.println(steps);
      digitalWrite(pinDirScn, dir);		// giro en un sentido
      for(int i = 0; i < steps; i++){   	// 200 pasos para motor de 0.9 grados de angulo de paso
        digitalWrite(pinStepScn, HIGH);     	// nivel alto
        delay(20);			  	// por 10 mseg
        digitalWrite(pinStepScn, LOW);      	// nivel bajo
        delay(20);			  	// por 10 mseg
      }     
      int sign = dir == 0? -1: 1;
      this->angleScn = this->angleScn +steps*(0.9*reduction)*sign;
    }

    void moveThr(float angle, int dir)
    {
      int steps = angle;

      Serial.print("El motor 3 se movera: ");
      Serial.println(steps);
      digitalWrite(pinDirThr, dir);		
      for(int i = 0; i < steps; i++){   	
        digitalWrite(pinStepThr, HIGH);     	// nivel alto
        delay(1);			  	
        digitalWrite(pinStepThr, LOW);      	// nivel bajo
        delay(1);			  	
      }     
      
    }
    void goTo(float thetaFst, float thetaScn)
    {
      thetaFst = constrain(thetaFst, 0, 180);
      thetaScn = constrain(thetaScn, 0, 180);
      float delta = thetaFst - this->angleFst;
      int dir = thetaFst > this->angleFst ? 0: 1;
      this->moveFst(abs(delta), dir);
      
      delta = thetaScn - this->angleScn;
      dir = thetaScn > this->angleScn ? 1: 0;
      this->moveScn(abs(delta), dir);
    }  
    void routine()
    {
      Serial.println("Arrancamos la rutina");
      this->setHomeAngles();
      this->goTo(30, 110); //move A
      this->moveThr(1000, 0); //bajo
      this->writeDegreeServo(0, 170);//cierro servo
      this->moveThr(1000, 1); //subo
      this->goTo(10, 90); //move B
      this->moveThr(1000, 0); //bajo
      this->writeDegreeServo(1, 170);//abro servo
      this->moveThr(1000, 1); //subo
      delay(1000);
      this->moveThr(1000, 0); //bajo
      this->writeDegreeServo(0, 170);//cierro servo
      this->moveThr(1000, 1); //subo
      this->goTo(30, 110); //move A
      this->moveThr(1000, 0); //bajo
      this->writeDegreeServo(1, 170);//abro servo
      delay(1000);
      this->moveThr(1000, 1); //subo
    }
    void initServo()
    {
      this->degreeServo.attach(10);
      this->degreeServo.write(90);
    }
    void writeDegreeServo(int Z, int Y)//1 abrir 0 cerrar
    {
      //this->degreeServo.attach(10);
      Serial.println("servo");
      
      int sense = 0;
      if(Z==0)sense = 180;
      this->degreeServo.write(sense);
      delay(Y);
      this->degreeServo.write(90);
    }

};
RobotController robotController(4,6,8);
char receivedChar;
bool manual = 1;
//int initalPoint
boolean newData = false;
Servo servito;
Angles angles;
int desiredPoint[2] = {0,350}; //Extendido
int currentPoint[2] = {0,350};
void setup() 
{
  pinMode(2,INPUT);
  //servito.attach(10);
  //servito.write(90);
  Serial.begin(9600);
  Serial.setTimeout(10);
  robotController.initServo();
  //robotController.goTo(180, 90);
}

void loop() {
  handleSerial();
  if(manual) manualMode();
  /*
  robotController.goTo(0, 90);
  delay(1500);
  robotController.goTo(28.5, 86.1);
  delay(1500);
  robotController.goTo(56, 77.7);
  delay(1500);
  */
}
void setManualMode()
{
  manual = !manual;
  //angles = computeAngles(0, 350);
}

void updatePosition()
{
  float currentAngles[2];
  float desiredAngles[2]; 
  inverseKinematics(currentPoint[0], currentPoint[1],currentAngles, 200, 150);
  inverseKinematics(desiredPoint[0], desiredPoint[1], desiredAngles, 200, 150);
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
  desiredPoint[0] =constrain(desiredPoint[0], -200, 200);
  desiredPoint[1] =constrain(desiredPoint[1], 0, 350);
  int potX = analogRead(A0);
  int potY = analogRead(A1);
  if (potX > 800) 
  {
    desiredPoint[0]-= 2;Serial.println("Punto : "+String(desiredPoint[0]) + " : " + String(desiredPoint[1]));return;
  }
  else if (potX < 200) 
  {
    desiredPoint[0]+= 2;Serial.println("Punto : "+String(desiredPoint[0]) + " : " + String(desiredPoint[1]));return;
  }
  else if (potY > 800) 
  {
    desiredPoint[1]-= 2;Serial.println("Punto : "+String(desiredPoint[0]) + " : " + String(desiredPoint[1]));return;
  }
  else if (potY < 200) 
  {
    desiredPoint[1]+= 2;Serial.println("Punto : "+String(desiredPoint[0]) + " : " + String(desiredPoint[1]));return;
  }
  delay(50);
  float theta1, theta2;
  InverseKinematics(desiredPoint[1], desiredPoint[0], theta1, theta2);
  float delta1, delta2;

  delta1 = 90 -theta1;
  delta2 = 90-theta2-theta1;
  robotController.goTo(delta2, delta1);
  //Serial.println("Punto : "+String(desiredPoint[0]) + " : " + String(desiredPoint[1]));
  //Serial.println("Angulos : "+String(angulos[0]) + " : " + String(angulos[1]));
  //robotController.goTo(angulos[0], angulos[1]);
  ////String position = String(desiredPoint[])
  //Serial.println("")
  
  if(digitalRead(2) == LOW)
  {
    float theta1, theta2;
    InverseKinematics(desiredPoint[1], desiredPoint[0], theta1, theta2);
    float delta1, delta2;

    delta1 = 90 -theta1;
    delta2 = 90-theta2-theta1;
    robotController.goTo(delta2, delta1);
    Serial.println("Angulos SCN : "+String(theta2) + " :  FST" + String(theta1));
    //Serial.println("Punto : "+String(desiredPoint[0]) + " : " + String(desiredPoint[1]));
    delay(100);
  }

  //
  //Angles deltaAngles = computeAngles(desiredPoint[0], desiredPoint[1]);
  //Serial.println("Punto : "+String(desiredPoint[0]) + " : " + String(desiredPoint[1]));
  //Serial.println("Angulos : "+String(angulos[0]) + " : " + String(angulos[1]));


}

void handleSerial() //funcion desvirtuada
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
      if(Z == 9 && x == 9 && Y == 9 ) 
      {
        robotController.routine();
        return;
      }


      if (Z == 3 && x == 7 && Y == 7 ) //esta malisimo esto
      {
        desiredPoint[0]-= 10;return;
      }
      else if (Z == 2 && x == 7 && Y == 7 ) 
      {
        desiredPoint[0]+= 102;return;
      }
      else if (Z == 4 && x == 7 && Y == 7 ) 
      {
        desiredPoint[1]-= 10;return;
      }
      else if (Z == 1 && x == 7 && Y == 7 ) 
      {
        desiredPoint[1]+= 10;return;
      }
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
        case 4:
          robotController.moveThr(y,Z);
          
          break;
        case 5://1 abrir 0 cerrar
          robotController.writeDegreeServo(Z, Y);
          //int sense = 0;
          //if(Z==0)sense = 180;
          //servito.write(sense);
          //delay(Y);
          //servito.write(90);
          break;
      }
      //Serial.println(String(x) + " : " + String(y) + " : " + String(Z));
      //mapCoordinatesToAngles(x,y,Z);
  }
}
