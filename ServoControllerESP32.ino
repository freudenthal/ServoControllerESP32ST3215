#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <CommandParser.h>
#include <Adafruit_NeoPixel.h>
#include <SCServo.h>
#include <math.h>
#include <PrintCharArray.h>
#include <TaskManagerIO.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels, 32 as default.
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define BRIGHTNESS  255
#define CTRL_ST_SERVO
#define ServoInitACC      0
#define ServoMaxSpeed     1500
#define MaxSpeed_X        1500
#define ServoInitSpeed    1500
// === SC Servo ===
#define CTRL_SC_SERVO
#define SMS_STS_ID SCSCL_ID

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// the IIC used to control OLED screen.
// GPIO 21 - S_SDA, GPIO 22 - S_SCL, as default.
#define S_SCL 22
#define S_SDA 21

// the GPIO used to control RGB LEDs.
// GPIO 23, as default.
#define RGB_LED   23
#define NUMPIXELS 10

typedef CommandParser<16, 4, 32, 128, 255> TextCommandParser;

// Servo variables
static const uint8_t ServoMaxID = 15;
static const uint8_t ServoDeadZone = 1;
static const uint8_t ServoMaxRoundsToTarget = 4;
static const uint16_t ServoCheckMillis = 100;
bool ServoVerbose = false;
uint8_t listID[ServoMaxID];
uint8_t searchNum = 0;
bool searchedStatus = false;
bool searchFinished = false;
bool searchCmd      = false;
uint16_t ServoSpeed = 500;
uint8_t ServoAcceleration = 0;
int16_t ServoTargetPositions[ServoMaxID];
bool ServoIsMoving[ServoMaxID];
bool ServoAttemptsToTarget[ServoMaxID];
bool ServoAnyMoving = false;
int ServoCheckIsMovingTaskID = 0;
int RAINBOW_STATUS = 0;

// Serial communication variables
static const uint16_t LineBufferCount = 256;
char LineBuffer[LineBufferCount];
uint8_t LineBufferReadLength = 0;
char LastChar = '\0';
PrintCharArray CommandResponse(TextCommandParser::MAX_RESPONSE_SIZE);

TextCommandParser Parser;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel matrix = Adafruit_NeoPixel(NUMPIXELS, RGB_LED, NEO_GRB + NEO_KHZ800);
SMS_STS st;


void servoInit()
{
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  for (uint8_t ServoIndex=0; ServoIndex < ServoMaxID; ServoIndex++)
  {
    ServoTargetPositions[ServoIndex] = 0;
    ServoIsMoving[ServoIndex] = false;
    ServoAttemptsToTarget[ServoIndex] = 0;
  }
}

void setMiddle(byte InputID)
{
  st.CalibrationOfs(InputID);
}

void setMode(byte InputID)
{
  st.unLockEprom(InputID);
  st.writeWord(InputID, SMS_STS_MIN_ANGLE_LIMIT_L, 0);
  st.writeWord(InputID, SMS_STS_MAX_ANGLE_LIMIT_L, 4095);
  st.writeWord(InputID, SMS_STS_OFS_L, 0);
  st.writeByte(InputID, SMS_STS_MODE, 0);
  st.writeByte(InputID, SMS_STS_CW_DEAD, ServoDeadZone);
  st.writeByte(InputID, SMS_STS_CCW_DEAD, ServoDeadZone);
  st.LockEprom(InputID);
}

void setID(byte ID_select, byte ID_set)
{
  if(ID_set >= ServoMaxID)
  {
    Serial.println(F("[ERROR](Servo ID limited. Please increase ServoMaxID.)"));
    return;
  }
  st.unLockEprom(ID_select);
  st.writeByte(ID_select, SMS_STS_ID, ID_set);
  st.LockEprom(ID_set);
}

void servoStop(byte servoID)
{
  st.EnableTorque(servoID, 0);
  delay(10);
  st.EnableTorque(servoID, 1);
}

void servoTorque(byte servoID, u8 enableCMD)
{
  st.EnableTorque(servoID, enableCMD);
}

void InitRGB()
{
  matrix.setBrightness(BRIGHTNESS);
  matrix.begin();
  matrix.show();
}

void colorWipe(uint32_t c, uint8_t wait) 
{
  for(uint16_t i=0; i<matrix.numPixels(); i++)
  {
    matrix.setPixelColor(i, c);
    matrix.show();
    delay(wait);
  }
}


void RGBALLoff()
{
  colorWipe(matrix.Color(0, 0, 0), 0);
}


void setSingleLED(uint16_t LEDnum, uint32_t c)
{
  matrix.setPixelColor(LEDnum, c);
  matrix.show();
}

void RGBoff()
{
  setSingleLED(0, matrix.Color(0, 0, 0));
  setSingleLED(1, matrix.Color(0, 0, 0));
}

void RGBcolor(byte Rinput, byte Ginput, byte Binput)
{
  setSingleLED(0, matrix.Color(Rinput, Ginput, Binput));
  setSingleLED(1, matrix.Color(Rinput, Ginput, Binput));
}


void ctrlAllLED(int totalNum, int inputR, int inputG, int inputB)
{
  for(int i = 0; i<totalNum; i++){
    setSingleLED(i, matrix.Color(inputR, inputG, inputB));
    delay(1);
  }
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
  if(WheelPos < 85) {
    return matrix.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return matrix.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return matrix.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


void rainbow(uint8_t wait)
{
  uint16_t i, j;
  for(j=0; j<256; j++) {
    for(i=0; i<matrix.numPixels(); i++) {
      matrix.setPixelColor(i, Wheel((i*1+j) & 255));
    }
    matrix.show();
    if(!RAINBOW_STATUS){RGBALLoff();break;}
    delay(wait);
  }
}

void InitScreen()
{
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
}

void screenUpdate()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  // Row 1
  display.print(F("Hello!"));
  display.println("");
  // Row 2
  display.print(F("Alive!"));
  display.println("");
  // Row 3
  display.print(F("Line 3!"));
  display.println("");
  // Row 4
  display.print(F("L:ine 4!"));
  display.println("");
  display.display();
}

void boardDevInit()
{
    Wire.begin(S_SDA, S_SCL);
    InitScreen();
    InitRGB();
}

void pingAll(bool searchCommand)
{
  if(searchCommand)
  {
    RGBcolor(0, 255, 64);
    searchNum = 0;
    searchedStatus = true;
    searchFinished = false;
    int PingStatus;
    for(int i = 0; i <= ServoMaxID; i++)
    {
      PingStatus = st.Ping(i);
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println(F("Searching Servos..."));
      display.print(F("Max ID "));
      display.print(ServoMaxID);
      display.print(F("-Ping:"));
      display.println(i);
      display.print(F("Detected:"));
      for(int i = 0; i < searchNum; i++)
      {
        display.print(listID[i]);display.print(" ");
      }
      display.display();
      if(PingStatus!=-1)
      {
        listID[searchNum] = i;
        searchNum++;
      }
    }
    searchedStatus = false;
    searchFinished = true;
    searchCmd      = false;
    RGBoff();
  }
}

void PrintPingAll()
{
  CommandResponse.clear();
  CommandResponse.print(F("[PingAll]("));
  for(int i = 0; i< searchNum; i++)
  {
    CommandResponse.print(listID[i]);
    CommandResponse.print(",");
  }
  CommandResponse.print(")");
}

void CommandIdentify(TextCommandParser::Argument *args, char *response)
{
  CommandResponse.clear();
  CommandResponse.print(F("SerialBusSTServoController"));
}

void CommandPingAll(TextCommandParser::Argument *args, char *response)
{
  taskManager.execute([] { pingAll(true); PrintPingAll();});
}

void CheckMovingServos()
{
  int16_t MinimalSlackToStartMove = 6 + ServoDeadZone;
  bool IsMoving = false;
  int16_t TargetPosition = 0;
  int16_t PresentPosition = 0;
  bool PrintResponse = false;
  if (ServoAnyMoving)
  {
    for (uint8_t ServoIndex=0; ServoIndex < ServoMaxID; ServoIndex++)
    {
      if (ServoIsMoving[ServoIndex])
      {
        IsMoving = st.readByte(ServoIndex, SMS_STS_MOVING) == 1;
        ServoIsMoving[ServoIndex] = IsMoving;
        if (!IsMoving)
        {
          TargetPosition = ServoTargetPositions[ServoIndex];
          PresentPosition = st.readWord(ServoIndex, SMS_STS_PRESENT_POSITION_L);
          CommandResponse.print(F("[SetPositionCount]("));
          CommandResponse.print(ServoIndex);
          CommandResponse.print(",");
          CommandResponse.print(TargetPosition);
          CommandResponse.print(",");
          CommandResponse.print(PresentPosition);
          CommandResponse.println(")");
          Serial.write(CommandResponse.getBuffer(),CommandResponse.size());
        }
      }
    }
  }
  bool NowAnyMoving = false;
  for (uint8_t ServoIndex=0; ServoIndex < ServoMaxID; ServoIndex++)
  {
    if (ServoIsMoving[ServoIndex])
    {
      NowAnyMoving = true;
      break;
    }
  }
  if (NowAnyMoving)
  {
    taskManager.schedule(onceMillis(ServoCheckMillis),CheckMovingServos);
  }
}

bool RequestMove(uint8_t ServoID, int16_t TargetPosition)
{
  if (ServoID >= ServoMaxID)
  {
    Serial.print(F("[ERROR](Servo ID mmust be less than ServoMaxID)"));
    return false;
  }
  //int16_t MinimalSlack = 3 + ServoDeadZone;
  int16_t MinimalSlackToStartMove = 6 + ServoDeadZone;
  int16_t GoalPosition = st.readWord(ServoID, SMS_STS_GOAL_POSITION_L);
  int16_t PresentPosition = st.readWord(ServoID, SMS_STS_PRESENT_POSITION_L);
  bool IsMoving = st.readByte(ServoID, SMS_STS_MOVING) == 1;
  if ( (PresentPosition != TargetPosition) && ( abs(TargetPosition - PresentPosition) > MinimalSlackToStartMove ) )
  {
    st.WritePosEx(ServoID,TargetPosition,ServoSpeed,ServoAcceleration);
    ServoTargetPositions[ServoID] = TargetPosition;
    ServoIsMoving[ServoID] = true;
    ServoAttemptsToTarget[ServoID] = 1;
    ServoAnyMoving = true;
    if (ServoVerbose)
    {
      Serial.print(F("<SERVOVERB>(Moving servo "));
      Serial.print(ServoID);
      Serial.print(F(" to "));
      Serial.print(TargetPosition);
      Serial.print(F(" from "));
      Serial.print(PresentPosition);
      Serial.println(F(".)"));
    }
    taskManager.schedule(onceMillis(ServoCheckMillis),CheckMovingServos);
    return true;
  }
  else
  {
    if (ServoVerbose)
    {
      Serial.print(F("<SERVOVERB>(No move needed for "));
      Serial.print(ServoID);
      Serial.print(F(" moving to "));
      Serial.print(TargetPosition);
      Serial.print(F(" from "));
      Serial.print(PresentPosition);
      Serial.println(F(".)"));
    }
    return false;
  }
}

void CommandSetPositionCount(TextCommandParser::Argument *args, char *response)
{
  uint8_t ParsedIndex = (uint8_t)(constrain((uint8_t)args[0].asUInt64,(uint8_t)0,(uint8_t)ServoMaxID));
  int16_t ParsedPosition = (int16_t)(constrain((int16_t)args[1].asUInt64,(int16_t)0,(int16_t)4095));
  bool Status = RequestMove(ParsedIndex, ParsedPosition);
  CommandResponse.clear();
  if (!Status)
  {
    CommandResponse.print(F("[SetPositionCount]("));
    CommandResponse.print(ParsedIndex);
    CommandResponse.print(",");
    CommandResponse.print(ParsedPosition);
    CommandResponse.print(")");
  }
}

void CommandGetPositionCount(TextCommandParser::Argument *args, char *response)
{
  uint8_t ParsedIndex = (uint8_t)(constrain((uint8_t)args[0].asUInt64,(uint8_t)0,(uint8_t)255));
  int32_t ReadPos = st.ReadPos(ParsedIndex);
  CommandResponse.clear();
  CommandResponse.print(F("[GetPositionCount]("));
  CommandResponse.print(ParsedIndex);
  CommandResponse.print(",");
  CommandResponse.print(ReadPos);
  CommandResponse.print(")");
}

void CommandSetSpeed(TextCommandParser::Argument *args, char *response)
{
  uint8_t ParsedSpeed = (uint8_t)(constrain((uint8_t)args[0].asUInt64,(uint8_t)0,(uint8_t)2000));
  CommandResponse.clear();
  CommandResponse.print(F("[SetSpeed]("));
  CommandResponse.print(ParsedSpeed);
  CommandResponse.print(")");
  ServoSpeed = ParsedSpeed;
}

void CommandGetSpeed(TextCommandParser::Argument *args, char *response)
{
  CommandResponse.clear();
  CommandResponse.print(F("[GetSpeed]("));
  CommandResponse.print(ServoSpeed);
  CommandResponse.print(")");
}

void CommandGetServoData(TextCommandParser::Argument *args, char *response)
{
  uint8_t ParsedIndex = (uint8_t)(constrain((uint8_t)args[0].asUInt64,(uint8_t)0,(uint8_t)255));
  int32_t CalibrationOfs = st.CalibrationOfs(ParsedIndex);
  int32_t Pos = st.ReadPos(ParsedIndex);
  int32_t Speed = st.ReadSpeed(ParsedIndex);
  int32_t Load = st.ReadLoad(ParsedIndex);
  int32_t Voltage = st.ReadVoltage(ParsedIndex);
  int32_t Temper = st.ReadTemper(ParsedIndex);
  int32_t Move = st.ReadMove(ParsedIndex);
  int32_t Current = st.ReadCurrent(ParsedIndex);
  int32_t Mode = st.ReadMode(ParsedIndex);
  CommandResponse.clear();
  CommandResponse.print(F("[GetServoData]("));
  CommandResponse.print(ParsedIndex);
  CommandResponse.print(","); CommandResponse.print(CalibrationOfs);
  CommandResponse.print(","); CommandResponse.print(Pos);
  CommandResponse.print(","); CommandResponse.print(Speed);
  CommandResponse.print(","); CommandResponse.print(Load);
  CommandResponse.print(","); CommandResponse.print(Voltage);
  CommandResponse.print(","); CommandResponse.print(Temper);
  CommandResponse.print(","); CommandResponse.print(Move);
  CommandResponse.print(","); CommandResponse.print(Current);
  CommandResponse.print(","); CommandResponse.print(Mode);
  CommandResponse.print(")");
}

void CommandGetRegisters(TextCommandParser::Argument *args, char *response)
{
  uint8_t ParsedIndex = (uint8_t)(constrain((uint8_t)args[0].asUInt64,(uint8_t)0,(uint8_t)255));
  
  int16_t Model = st.readWord(ParsedIndex, SMS_STS_MODEL_L);
  uint8_t ID = st.readByte(ParsedIndex, SMS_STS_ID);
  uint8_t BaudRate = st.readByte(ParsedIndex, SMS_STS_BAUD_RATE);
  int16_t MinAngle = st.readWord(ParsedIndex, SMS_STS_MIN_ANGLE_LIMIT_L);
  int16_t MaxAngle = st.readWord(ParsedIndex, SMS_STS_MAX_ANGLE_LIMIT_L);
  uint8_t DeadCW = st.readByte(ParsedIndex, SMS_STS_CW_DEAD);
  uint8_t DeadCCW = st.readByte(ParsedIndex, SMS_STS_CCW_DEAD);
  int16_t Offset = st.readWord(ParsedIndex, SMS_STS_OFS_L);
  uint8_t Mode = st.readByte(ParsedIndex, SMS_STS_MODE);
  uint8_t Torque = st.readByte(ParsedIndex, SMS_STS_TORQUE_ENABLE);
  uint8_t Acceleration = st.readByte(ParsedIndex, SMS_STS_ACC);
  int16_t GoalPosition = st.readWord(ParsedIndex, SMS_STS_GOAL_POSITION_L);
  int16_t GoalTime = st.readWord(ParsedIndex, SMS_STS_GOAL_TIME_L);
  int16_t GoalSpeed = st.readWord(ParsedIndex, SMS_STS_GOAL_SPEED_L);
  int16_t TorqueLimit = st.readWord(ParsedIndex, SMS_STS_TORQUE_LIMIT_L);
  uint8_t Lock = st.readByte(ParsedIndex, SMS_STS_LOCK);
  int16_t PresentPosition = st.readWord(ParsedIndex, SMS_STS_PRESENT_POSITION_L);
  int16_t PresentSpeed = st.readWord(ParsedIndex, SMS_STS_PRESENT_SPEED_L);
  int16_t PresentLoad = st.readWord(ParsedIndex, SMS_STS_PRESENT_LOAD_L);
  uint8_t Voltage = st.readByte(ParsedIndex, SMS_STS_PRESENT_VOLTAGE);
  uint8_t Temperature = st.readByte(ParsedIndex, SMS_STS_PRESENT_TEMPERATURE);
  uint8_t Moving = st.readByte(ParsedIndex, SMS_STS_MOVING);
  int16_t Current = st.readWord(ParsedIndex, SMS_STS_PRESENT_CURRENT_L);

  CommandResponse.clear();
  CommandResponse.print(F("[Model]("));CommandResponse.print(Model);CommandResponse.println(")");
  CommandResponse.print(F("[ID]("));CommandResponse.print(ID);CommandResponse.println(")");
  CommandResponse.print(F("[BaudRate]("));CommandResponse.print(BaudRate);CommandResponse.println(")");
  Serial.write(CommandResponse.getBuffer(),CommandResponse.size());

  CommandResponse.clear();
  CommandResponse.print(F("[MinAngle]("));CommandResponse.print(MinAngle);CommandResponse.println(")");
  CommandResponse.print(F("[MaxAngle]("));CommandResponse.print(MaxAngle);CommandResponse.println(")");
  CommandResponse.print(F("[DeadCW]("));CommandResponse.print(DeadCW);CommandResponse.println(")");
  CommandResponse.print(F("[DeadCCW]("));CommandResponse.print(DeadCCW);CommandResponse.println(")");
  Serial.write(CommandResponse.getBuffer(),CommandResponse.size());

  CommandResponse.clear();
  CommandResponse.print(F("[Offset]("));CommandResponse.print(Offset);CommandResponse.println(")");
  CommandResponse.print(F("[Mode]("));CommandResponse.print(Mode);CommandResponse.println(")");
  CommandResponse.print(F("[Torque]("));CommandResponse.print(Torque);CommandResponse.println(")");
  CommandResponse.print(F("[Acceleration]("));CommandResponse.print(Acceleration);CommandResponse.println(")");
  Serial.write(CommandResponse.getBuffer(),CommandResponse.size());

  CommandResponse.clear();
  CommandResponse.print(F("[GoalPosition]("));CommandResponse.print(GoalPosition);CommandResponse.println(")");
  CommandResponse.print(F("[GoalTime]("));CommandResponse.print(GoalTime);CommandResponse.println(")");
  CommandResponse.print(F("[GoalSpeed]("));CommandResponse.print(GoalSpeed);CommandResponse.println(")");
  Serial.write(CommandResponse.getBuffer(),CommandResponse.size());

  CommandResponse.clear();
  CommandResponse.print(F("[TorqueLimit]("));CommandResponse.print(TorqueLimit);CommandResponse.println(")");
  CommandResponse.print(F("[Lock]("));CommandResponse.print(Lock);CommandResponse.println(")");
  Serial.write(CommandResponse.getBuffer(),CommandResponse.size());

  CommandResponse.clear();
  CommandResponse.print(F("[PresentPosition]("));CommandResponse.print(PresentPosition);CommandResponse.println(")");
  CommandResponse.print(F("[PresentSpeed]("));CommandResponse.print(PresentSpeed);CommandResponse.println(")");
  CommandResponse.print(F("[PresentLoad]("));CommandResponse.print(PresentLoad);CommandResponse.println(")");
  Serial.write(CommandResponse.getBuffer(),CommandResponse.size());

  CommandResponse.clear();
  CommandResponse.print(F("[Voltage]("));CommandResponse.print(Voltage);CommandResponse.println(")");
  CommandResponse.print(F("[Temperature]("));CommandResponse.print(Temperature);CommandResponse.println(")");
  CommandResponse.print(F("[Moving]("));CommandResponse.print(Moving);CommandResponse.println(")");
  Serial.write(CommandResponse.getBuffer(),CommandResponse.size());

  CommandResponse.clear();
  CommandResponse.print(F("[Current]("));CommandResponse.print(Current);CommandResponse.println(")");
  Serial.write(CommandResponse.getBuffer(),CommandResponse.size());

  CommandResponse.clear();
}

void CommandSetServoMode(TextCommandParser::Argument *args, char *response)
{
  uint8_t ParsedIndex = (uint8_t)(constrain((uint8_t)args[0].asUInt64,(uint8_t)0,(uint8_t)255));
  CommandResponse.clear();
  CommandResponse.print(F("[SetServoMode]("));
  CommandResponse.print(ParsedIndex);
  CommandResponse.print(")");
  setMode(ParsedIndex);
}

void ClearLineBuffer()
{
  memset(LineBuffer, 0, sizeof(LineBuffer));
  LineBufferReadLength = 0;
  LastChar = '\0';
}

void BuildParser()
{
  ClearLineBuffer();
  Parser.registerCommand("ID", "", &CommandIdentify);
  Parser.registerCommand("PingAll", "", &CommandPingAll);
  Parser.registerCommand("SetPositionCount", "uu", &CommandSetPositionCount);
  Parser.registerCommand("GetPositionCount", "u", &CommandGetPositionCount);
  Parser.registerCommand("GetServoData", "u", &CommandGetServoData);
  Parser.registerCommand("GetRegisters", "u", &CommandGetRegisters);
  Parser.registerCommand("SetSpeed", "u", &CommandSetSpeed);
  Parser.registerCommand("GetSpeed", "", &CommandGetSpeed);
  Parser.registerCommand("SetServoMode", "u", &CommandSetServoMode);
}

void ProcessLineBuffer()
{
  LineBuffer[LineBufferReadLength] = '\0';
  Parser.processCommand(LineBuffer, CommandResponse.getBuffer());
  if (CommandResponse.size() > 0)
  {
    CommandResponse.println();
    Serial.write(CommandResponse.getBuffer(),CommandResponse.size());
  }
}

void ParseSerial()
{
  while (Serial.available() > 0)
  {
    char NewChar = Serial.read();
    bool NewCharIsLineEnd = (NewChar == '\n') || (NewChar == ';') || (NewChar == '\r');
    bool NewCharIsBlankOrEnd = ( isspace(NewChar) || NewCharIsLineEnd );
    bool NewCharIsIgnored = (NewChar == '\t') || (NewChar == '\v') || (NewChar == '\f');
    bool NewCharIsDoubleSpace = isspace(NewChar) && isspace(LastChar);
    bool IsLeadingWhiteSpace = (LineBufferReadLength == 0) && NewCharIsBlankOrEnd;
    if ( IsLeadingWhiteSpace || NewCharIsIgnored || NewCharIsDoubleSpace )
    {
      //Do nothing
    }
    else if ( NewCharIsLineEnd )
    {
      if (LineBufferReadLength > 1)
      {
        ProcessLineBuffer();
      }
      ClearLineBuffer();
      break;
    }
    else
    {
      LastChar = NewChar;
      LineBuffer[LineBufferReadLength] = NewChar;
      LineBufferReadLength++;
      if (LineBufferReadLength >= LineBufferCount)
      {
        ClearLineBuffer();
        Serial.println("Input buffer overflow. Input cleared.");
        break;
      }
    }
  }
}

void SetupTasks()
{
  taskManager.schedule(repeatMillis(100),ParseSerial);
}

void setup()
{
  Serial.begin(115200);
  boardDevInit();
  BuildParser();
  RGBcolor(0, 64, 255);
  servoInit();
  delay(1000);
  pingAll(true);
  SetupTasks();
  RGBoff();
}


void loop()
{
  taskManager.runLoop();
}