

class RobotController {
    private:
    int pinDirFst;
    int pinDirScn;
    int pinDirThr;
    int pinStepFst;
    int pinStepScn;
    int pinStepThr;
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
      Serial.begin(9600);
      Serial.println(String(pinDirFst));
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
    
    }
};
RobotController robotController(4,6,8);
char receivedChar;
boolean newData = false;
void setup() 
{
  Serial.begin(9600);
  Serial.setTimeout(10);
}
void loop() {
    handleSerial();

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
      float y = command.toInt();
      command =  Serial.readStringUntil('}');
      int z = command.toInt();
      if(!(z == 1 || z == 0) ) return;
      Serial.println(String(x) + " :  " + String(y)+ " :  " + String(z));
      //if(z != 1) return;
      //if(z != 1 && z != 0) return;
      switch(x){
        case 1:
          robotController.moveFst(y,z);
          break;
        case 2:
          robotController.moveScn(y,z);
          break;
        case 3:
          //robotController.moveThrd(y);
          break;
      }
      //Serial.println(String(x) + " : " + String(y) + " : " + String(z));
      //mapCoordinatesToAngles(x,y,z);
  }
}
