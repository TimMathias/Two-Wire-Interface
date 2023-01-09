#include "twi.h"

const byte MPU_ADDRESS = 0x68;  // I2C address of the MPU-6050. Either 0x68 or 0x69.
const byte ADO_PIN = 12;        // LOW = 0x68 (default); HIGH = 0x69.

const byte SCUPPER_SCL_PIN = 2;
const byte SCUPPER_SDA_PIN = 3;

const byte LOGIC_TRIGGER_PIN = 7;

struct AccelTempGyroData
{
  const uint32_t reg = 0x3B;
  const uint8_t reg_size = 1;
  volatile byte buffer[14];
  int16_t AccelX() const { return buffer[ 0] << 8 | buffer[ 1]; }
  int16_t AccelY() const { return buffer[ 2] << 8 | buffer[ 3]; }
  int16_t AccelZ() const { return buffer[ 4] << 8 | buffer[ 5]; }
  int16_t Temp()   const { return buffer[ 6] << 8 | buffer[ 7]; }
  int16_t GyroX()  const { return buffer[ 8] << 8 | buffer[ 9]; }
  int16_t GyroY()  const { return buffer[10] << 8 | buffer[11]; }
  int16_t GyroZ()  const { return buffer[12] << 8 | buffer[13]; }
  float   TempD()  const { return Temp() / 340.0 + 36.53; }      // Equation for temperature in °C from datasheet.
  float   TempF()  const { return TempD() * 9.0 / 5.0 + 32.0; }  // Equation for temperature in °F.
};

AccelTempGyroData data;

void setup()
{
  Serial.begin(115200);

  Serial.println("\nsetup()\n");

  pinMode(LED_BUILTIN, OUTPUT);

  twi.Enable();

  if (bitRead(MCUSR, WDRF))
  {
    Serial.println("System was reset due to watchdog timeout.");
    Serial.println(WDTCSR);
    Serial.println();
  }

  pinMode(SCUPPER_SCL_PIN, INPUT_PULLUP);
  pinMode(SCUPPER_SDA_PIN, INPUT_PULLUP);

  pinMode(LOGIC_TRIGGER_PIN, OUTPUT);
  //digitalWrite(LOGIC_TRIGGER_PIN, HIGH);  // Capture data on the Wokwi Logic Analyser.
  digitalWrite(LOGIC_TRIGGER_PIN, LOW);  // Don't capture data on the Wokwi Logic Analyser.

  pinMode(ADO_PIN, OUTPUT);
  digitalWrite(ADO_PIN, MPU_ADDRESS & 1);

  //
  // Perform full reset as per MPU-6000/MPU-6050 Register Map and Descriptions, Section 4.28, pages 40 to 41.
  //

  // PWR_MGMT_1 register: DEVICE_RESET = 1
  Serial.println("Resetting MPU...");
  twi.Write(MPU_ADDRESS, 0x6B, 1, 0b10000000, TWI::Modes::Wait);
  delay(100);  // Wait for reset to complete.
  Serial.println("Done.");

  // SIGNAL_PATH_RESET register: GYRO_RESET = 1, ACCEL_RESET = 1, TEMP_RESET = 1.
  Serial.println("Resetting MPU gyro, accel and temp...");
  twi.Write(MPU_ADDRESS, 0x68, 1, 0b00000111, TWI::Modes::Wait);
  delay(100);  // Wait for reset to complete.
  Serial.println("Done.");

  // Disable SLEEP mode because the reset re-enables it. Section 3, PWR_MGMT_1 register, page 8.
  // PWR_MGMT_1 register: SLEEP = 0
  Serial.println("Resetting MPU sleep mode...");
  twi.Write(MPU_ADDRESS, 0x6B, 1, 0b00000000, TWI::Modes::Wait);
  Serial.println("Done.");
}

void loop()
{
  //
  // TASK 1: Alternate between non-blocking and blocking burst-reads.
  //

/**/

  // TASK 1A: Non-blocking burst-read of 14 bytes. (Uses the TWI ISR.)

  static bool receiving = false;
  if (!receiving && twi.Read(MPU_ADDRESS, data.reg, data.reg_size, data.buffer, sizeof(data.buffer)) == TWI::Results::Started)
  {
    receiving = true;
    Serial.println("\nBurst-read of 14 bytes (using non-blocking ISR).");
  }
  if (receiving && twi.IsReady())
  {
    receiving = false;
    MPU6050PrintResults();
  }

/**/

  // TASK 1B: Blocking burst-read of 14 bytes. (Uses a while loop by specifying the Wait flag.)

  if (twi.Read(MPU_ADDRESS, data.reg, data.reg_size, data.buffer, sizeof(data.buffer), TWI::Modes::Wait) == TWI::Results::Success)
  {
    Serial.println("\nBurst-read of 14 bytes (using blocking while loop).");
    MPU6050PrintResults();
  }

/**/

  //
  // TASK 2: Blink the inbuilt LED.
  //

  uint16_t timestamp = millis();
  const uint16_t LED_BLINK_INTERVAL = 200;
  static uint16_t led_blink_previous_timestamp = timestamp;
  static bool led_state = false;
  if (timestamp - led_blink_previous_timestamp >= LED_BLINK_INTERVAL)
  {
    led_state = !led_state;
    digitalWrite(LED_BUILTIN, led_state);
    led_blink_previous_timestamp += LED_BLINK_INTERVAL;
  }
}

void MPU6050PrintResults()
{
  TWI::Results result = twi.GetResult();
  switch (result)
  {
    case TWI::Results::Success:
      Serial.println("Success");
      Serial.print("AccelX = "); Serial.print(data.AccelX());
      Serial.print(" | AccelY = "); Serial.print(data.AccelY());
      Serial.print(" | AccelZ = "); Serial.print(data.AccelZ());
      Serial.print(" | Temp = "); Serial.print(data.TempD()); Serial.print("°C "); Serial.print(data.TempF()); Serial.print("°F");
      Serial.print(" | GyroX = "); Serial.print(data.GyroX());
      Serial.print(" | GyroY = "); Serial.print(data.GyroY());
      Serial.print(" | GyroZ = "); Serial.println(data.GyroZ());
      Serial.println();
      break;
    case TWI::Results::Timeout:
      Serial.println("Timeout.");
      break;
    default:
      Serial.println("Comms error.");
      break;
  }
}
