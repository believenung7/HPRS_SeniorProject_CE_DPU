#include <SoftwareSerial.h>
#include <PS2X_lib.h>
SoftwareSerial MegaUno(6, 7); // RX, TX
char text;

#define PS2_DAT        8
#define PS2_CMD        9
#define PS2_SEL        10
#define PS2_CLK        11

#define sw_control       2

PS2X ps2x;
int check = 0;
char Control;

int flagAutoGyro = 0;

int val11;
float val2;
int a = 0;
int SWcontrol = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  MegaUno.begin(9600);
  //MegaUno.println("Ready to control");
  //  MegaUno.begin(9600);
  pinMode(sw_control, INPUT_PULLUP);
  Serial.println("OK to use");
  MegaUno.println("GRM#");
  MegaUno.println("SP0#");
  while (true)
  {
    int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
    if (error == 0)
    {
      Serial.println("joy ON");
      break;
    }
    delay(500);
  }
}

void loop()
{
  SWcontrol = digitalRead(sw_control);
  if (SWcontrol == HIGH)
  {
    //serial();
    //Serial.println("1");//Android
  }
  else
  {
    JoyControl();
    //Serial.println("0");//Joy
  }
}



void CheckVoltage()
{
  float temp;
  val11 = analogRead(1);
  temp = val11 / 4.092;

  val11 = (int)temp; //

  val2 = ((val11 % 100) / 10.0);
  //bluetooth.println(val2);
  Serial.println(val2);

  delay(1000);
}

void serial()
{
  //Read from MegaUno and write to usb serial

  if (MegaUno.available() )
  {
    char toSend = (char)MegaUno.read();
    Serial.print(toSend);
  }

  //Read from usb serial to MegaUno
  if (Serial.available())
  {
    char toSend = (char)Serial.read();
    //  char toSend = (char)Control;
    MegaUno.print(toSend);
    //Serial.print(toSend);
  }
}

void JoyControl ()
{
  //  while (true)
  //  {
  //    int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  //    if (error == 0)
  //    {
  //      Serial.println("joy ON");
  //      break;
  //    }
  //  }
  ps2x.read_gamepad(false, false);     // อ่านข้อมูลจาก PS2 Controller

  if (ps2x.Button(PSB_TRIANGLE))  // ถ้าปุ่มสามเหลี่ยมถูกกด
  {
    MegaUno.println("CU#");        // แสดงข้อความว่า Triangle
    //Serial.println("CU#");
  }
  else if (ps2x.Button(PSB_CROSS))     // ถ้าปุ่มกากบาทถูกกด
  {
    MegaUno.println("CD#");           // แสดงข้อความว่า Cross
    //Serial.println("CD#");
  }
  else if (ps2x.Button(PSB_CIRCLE))         // ถ้าปุ่มวงกลมถูกกด
  {
    MegaUno.println("GRA#");          // แสดงข้อความว่า Circle
    //Serial.println("GRA#");
    //Control = 'CC';
    flagAutoGyro = 1;
  }
  else if (ps2x.Button(PSB_SQUARE))    // ถ้าปุ่มสี่เหลี่ยมถูกกด
  {
    MegaUno.println("GRM#");          // แสดงข้อความว่า Square
    //Serial.println("GRM#");
    flagAutoGyro = 0;
  }

  else if (ps2x.Button(PSB_L1))        // ถ้าปุ่ม L1 ถูกกด
  {
    MegaUno.println("SP0#");              // แสดงข้อความว่า L1
    //Serial.println("SP0#");
  }
  else if (ps2x.Button(PSB_L2))        // ถ้าปุ่ม L2 ถูกกด
  {
    MegaUno.println("SP1#");              // แสดงข้อความว่า L2
    //Serial.println("SP1#");
  }
  else if (ps2x.Button(PSB_L3))        // ถ้าปุ่ม L3 ถูกกด
  {
    //Serial.println("L3#");
  }
  else if (ps2x.Button(PSB_R1))        // ถ้าปุ่ม R1 ถูกกด
  {
    MegaUno.println("SP2#");              // แสดงข้อความว่า R1
    //Serial.println("SP2#");
  }
  else if (ps2x.Button(PSB_R2))        // ถ้าปุ่ม R2 ถูกกด
  {
    MegaUno.println("SP3#");             // แสดงข้อความว่า R2
    //Serial.println("SP3#");
  }
  else if (ps2x.Button(PSB_R3))        // ถ้าปุ่ม R3 ถูกกด
  {
    //Serial.println("R3");              // แสดงข้อความว่า R3
  }
  else if (ps2x.Button(PSB_START))     // ถ้าปุ่ม Start ถูกกด
  {
    //Serial.println("Start");           // แสดงข้อความว่า Start
  }
  else if (ps2x.Button(PSB_SELECT))    // ถ้าปุ่ม Select ถูกกด
  {
    //Serial.println("Select");          // แสดงข้อความว่า Select
  }
  else if (ps2x.Button(PSB_PAD_UP))    // ถ้าปุ่ม Up ถูกกด
  {
    MegaUno.println("CF#");              // แสดงข้อความว่า Up
    //Serial.println("CF#");
  }
  else if (ps2x.Button(PSB_PAD_DOWN))  // ถ้าปุ่ม Down ถูกกด
  {
    MegaUno.println("CB#");            // แสดงข้อความว่า Down
    //Serial.println("CB#");
  }
  else if (ps2x.Button(PSB_PAD_LEFT))  // ถ้าปุ่ม Left ถูกกด
  {
    MegaUno.println("CL#");            // แสดงข้อความว่า Left
    //Serial.println("CL#");
  }
  else if (ps2x.Button(PSB_PAD_RIGHT)) // ถ้าปุ่ม Right ถูกกด
  {
    MegaUno.println("CR#");           // แสดงข้อความว่า Right
    //Serial.println("CR#");
  }
  else
  {
    if (flagAutoGyro == 0) // Maunal
    {
      MegaUno.print("CP#");
      //Serial.println("CP#");
      delay(5);
      MegaUno.println("CS#");
      //Serial.println("CS#");
    }
    else
    {
      MegaUno.println("CS#");
      //Serial.println("CS#");
    }
    //MegaUno.println();
  }
  delay(50);
}

