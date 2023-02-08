#include <twi.h>

const byte OWN_TARGET_ADDRESS = 0x23;
const byte OWN_TARGET_GENERAL_CALL = 0;

const byte OTHER_TARGET_ADDRESS = 0x24;

void setup()
{
  Serial.begin(115200);

  Serial.println(F("\nsetup() begin\n"));

  pinMode(LED_BUILTIN, OUTPUT);

  static Queue<byte, BYTE_BUFFER_LENGTH> rx_buffer;
  static Queue<byte, BYTE_BUFFER_LENGTH> tx_buffer;
  twi.OwnTargetInit(OWN_TARGET_ADDRESS, OWN_TARGET_GENERAL_CALL, &rx_buffer, &tx_buffer);

  twi.Enable(400000);

  if (bitRead(MCUSR, WDRF))
  {
    Serial.println(F("System was reset due to watchdog timeout."));
    char buffer[20];
    sprintf(buffer, "WDTCSR 0x%02x", WDTCSR);
    Serial.println(buffer);
    Serial.println();
  }

  Serial.println(F("\nsetup() end\n"));
}

void loop()
{
  BlinkLed();
  ControllerWrite();
  ControllerRead();
  OwnTargetWrite();
  OwnTargetRead();
}

void BlinkLed()
{
  const uint16_t INTERVAL = 500;
  uint16_t timestamp = millis();
  static uint16_t previous = 0;
  if (timestamp - previous >= INTERVAL)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    previous += INTERVAL;
  }
}

void ControllerWrite()
{
  const int16_t INTERVAL = 5000;
  int16_t timestamp = millis();
  static int16_t previous = 0;
  if (timestamp - previous >= INTERVAL)
  {
    static int i = 1;
    char buffer[64];
    sprintf(buffer, "Hello from 0x%X controller transmitter, %d.\n\0", OWN_TARGET_ADDRESS, i);
    if (twi.Write(OTHER_TARGET_ADDRESS, (byte*)buffer, strlen(buffer) + 1, TWI::Modes::Wait) == TWI::Results::Success)
    {
      i++;
    }
    previous += INTERVAL;
  }
}

void ControllerRead()
{
  const int16_t INTERVAL = 5000;
  int16_t timestamp = millis();
  static int16_t previous = 500;
  if (timestamp - previous >= INTERVAL)
  {
    char buffer[64];
    TWI::Results r = twi.Read(OTHER_TARGET_ADDRESS, (byte*)buffer, 64, TWI::Modes::Wait);
    if (r == TWI::Results::Success)
    {
      Serial.print(F("0x"));
      Serial.print(OWN_TARGET_ADDRESS, 16);
      Serial.print(F(" ControllerRead: "));
      Serial.print(buffer);
    }
    previous += INTERVAL;
  }
}

void OwnTargetWrite()
{
  const int16_t INTERVAL = 5000;
  int16_t timestamp = millis();
  static int16_t previous = 1000;
  if (timestamp - previous >= INTERVAL)
  {
    static int i = 1;
    char buffer[64];
    sprintf(buffer, "Hello from 0x%X target transmitter, %d.\n", OWN_TARGET_ADDRESS, i);
    byte length = strlen(buffer);
    byte freeSpace = twi.OwnTargetWriteable();
    if (freeSpace > length)
    {
      i++;
      for (byte b = 0; b < length; b++)
      {
        twi.OwnTargetWrite(buffer[b]);
      }
      twi.OwnTargetWrite(0);
    }
    previous += INTERVAL;
  }
}

void OwnTargetRead()
{
  byte b;
  if (twi.OwnTargetRead(b))
  {
    static char buffer[65];
    static int i = 0;
    buffer[i++] = b;
    i %= 64;
    if (b == 0)
    {
      i = 0;
      Serial.print(F("0x"));
      Serial.print(OWN_TARGET_ADDRESS, 16);
      Serial.print(F(" OwnTargetRead: "));
      Serial.print(buffer);
    }
  }
}

void TwiPrintState()
{
  TWI::States state = twi.GetState();
  switch (state)
  {
    case TWI::States::Busy:
      Serial.println(F("Busy."));
      break;
    case TWI::States::Ready:
      Serial.println(F("Ready."));
      break;
  }
}

void TwiPrintResult()
{
  TWI::Results result = twi.GetResult();
  uint32_t timestamp = millis();
  Serial.print(timestamp);
  Serial.print(F(": "));
  switch (result)
  {
  case TWI::Results::Unknown:
    Serial.println(F("Unknown."));
    break;
  case TWI::Results::Pending:
    Serial.println(F("Pending."));
    break;
  case TWI::Results::FailedToStart:
    Serial.println(F("FailedToStart."));
    break;
  case TWI::Results::Started:
    Serial.println(F("Started."));
    break;
  case TWI::Results::ArbitrationLost:
    Serial.println(F("Arbitration lost."));
    break;
  case TWI::Results::Timeout:
    Serial.println(F("Timeout."));
    break;
  case TWI::Results::Success:
    Serial.println(F("Success."));
    break;
  default:
    Serial.println(F("Default unknown."));
    break;
  }
}
