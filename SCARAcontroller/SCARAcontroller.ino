float reduction = 40.0/12.0;

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
    }
    void print();
};

void RobotController :: print()
{
  int a = 1;
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
