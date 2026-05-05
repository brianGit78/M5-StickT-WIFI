#include "Arduino.h"
#include "Lepton.h"
#include "Wire.h"
#include "SPI.h"
#define rotationHorizont 0
#define rotationVert 0
static byte leptonFrame[164];
unsigned short  smallBuffer[160*120];
uint16_t raw_max = 0, raw_min = 0xFFFF;
//uint16_t frame_buffer[4][60][164];
uint16_t aux_temp = 0, fpa_temp = 0, max_x, max_y, min_x, min_y;

static SPIClass* lepton_spi = new SPIClass(HSPI);

static void ESP_DelayUS(uint64_t us)
{
  uint64_t m = ESP_GetUS();
  uint64_t e = (m + us);
  if (us)
  {
    //todo: this sucks if ref tick is 100Hz
    if (us > (3000 * portTICK_PERIOD_MS))
    {
      vTaskDelay((us - (2000 * portTICK_PERIOD_MS)) / (portTICK_PERIOD_MS * 1000));
      m = ESP_GetUS();
    }
    if (m > e)
    { //overflow
      while (ESP_GetUS() > e)
        ;
    }
    while (ESP_GetUS() < e)
      ;
  }
}

static volatile bool vsync_triggered = 0;
static volatile uint64_t vsync_time = 0;

static void IRAM_ATTR onVsyncISR()
{
  vsync_triggered = 1;
  //vsync_time = (uint64_t)esp_timer_get_time();
}

Lepton::Lepton(int sdaPin, int sclPin, int ssPin, int syncPin) : _sdaPin(sdaPin), _sclPin(sclPin), _ssPin(ssPin), _syncPin(syncPin) {
}
void Lepton::begin() {
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  delay(100);
  digitalWrite(RESET_PIN, LOW);
  delay(300);
  digitalWrite(RESET_PIN, HIGH);
  delay(1000); // Lepton spec: 950ms min after RESET release before first I2C command

  //Wire.begin(_sdaPin, _sclPin);

  pinMode(_syncPin, INPUT); //vsync
  attachInterrupt(_syncPin, onVsyncISR, RISING);

  pinMode(_ssPin, OUTPUT);
  digitalWrite(_ssPin, HIGH);

  lepton_spi->begin(2, 25, -1, -1); // CLK=2, MISO=25; MOSI unused (Lepton is read-only SPI), CS managed manually
}

uint16_t Lepton::readRegister(uint16_t reg) {
  setRegister(reg);
  Wire1.requestFrom(DEVICE_ID, (uint8_t)2);
  return readWord();
}

void Lepton::writeRegister(uint16_t reg, uint16_t value) {
  startTransmission(reg);
  transmitWord(value);
  endTransmission();
}

uint16_t Lepton::doGetCommand(uint16_t commandIdBase, uint16_t *data)
{
  writeRegister(REG_COMMAND_ID, commandIdBase | TYPE_GET);
  waitIdle();
  return readData(data);
}

void Lepton::doSetCommand(uint16_t commandIdBase, uint16_t *data, uint16_t dataLen)
{
  writeData(data, dataLen);
  writeRegister(REG_COMMAND_ID, commandIdBase | TYPE_SET);
  waitIdle();
}

uint16_t Lepton::doRunCommand(uint16_t commandIdBase, uint16_t *data, uint16_t dataLen)
{
  writeData(data, dataLen);
  writeRegister(REG_COMMAND_ID, commandIdBase | TYPE_RUN);
  waitIdle();
  return readData(data);
}

/* Get one line package from the Lepton */
int Lepton::getPackage(byte line, byte seg)
{

  lepton_spi->transferBytes(NULL,leptonFrame,164); 
   
  if((leptonFrame[0] & 0x0F) == 0x0F)
    return 1;
    
  //Check if the line number matches the expected line
  if (leptonFrame[1] != line)
   return 2;

  //For the Lepton3.x, check if the segment number matches
  if (line == 20)
  {
    byte segment = (leptonFrame[0] >> 4);
    if (segment == 0)
      return 3;
    if (segment != seg)
      return 4;
  }

  return 0;
}

/* Store one package of 80 columns into RAM */
bool Lepton::savePackage(byte line, byte segment)
{
  // Reject invalid segment — (segment-1) as byte underflows to 255 when segment=0,
  // producing addr_base ~44352 and writing past the end of smallBuffer into heap.
  if (segment < 1 || segment > 4 || line >= 60) return 0;

  uint16_t x_base = (line & 0x01) ? 80 : 0;
  uint16_t y_base = ((segment - 1) * 30) + (line >> 1);
  uint16_t addr_base = y_base * 160 + x_base;
  //Go through the video pixels for one video line
  for (int column = 0; column < 80; column++)
  {
    //Make a 16-bit rawvalue from the lepton frame
    uint16_t result = (uint16_t)(leptonFrame[(column << 1) + 4] << 8 | leptonFrame[(column << 1) + 5]);

    if (result > raw_max)
    {
      raw_max = result;
      max_x = x_base + column;
      max_y = y_base;
    }
    if (result < raw_min)
    {
      raw_min = result;
      min_x = x_base + column;
      min_y = y_base;
    }

    //Invalid value, return
    if (result == 0)
    {
      return 0;
    }
    else
    {
      smallBuffer[addr_base + column] = result;
    }

  }
  
  //Everything worked
  return 1;
}

// Returns true if VSYNC fired, false on timeout.
static bool WaitForVsync(uint32_t timeoutMs = 2000)
{
  vsync_triggered = 0;
  uint32_t deadline = millis() + timeoutMs;
  while (!vsync_triggered) {
    if (millis() >= deadline) return false;
    // Yield so the FreeRTOS IDLE task can run and reset the Task Watchdog Timer.
    // Without this, 4 segments × 500ms busy-wait + 2× I2C waitIdle = ~7s, which
    // exceeds the 5s TWDT window and resets the device mid-stream.
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  return true;
}

/* Get one frame of raw values from the lepton */
void Lepton::getRawValues()
{
  byte line, error, segment;
  raw_max = 0;
  raw_min = 0xFFFF;
  max_x = 0;
  max_y = 0;
  min_x = 0;
  min_y = 0;
  syncFrame();

  static uint32_t lastVsyncWarnMs = 0;
  uint8_t vsyncMisses = 0;

  for (segment = 1; segment <= 4; segment++)
  {
    if (!WaitForVsync(500)) {
      // VSYNC didn't fire within 500ms — Lepton may still be booting
      uint32_t now = millis();
      if (now - lastVsyncWarnMs > 2000) {
        Serial.printf("[Lepton] VSYNC timeout seg=%d vsync_triggered=%d\n",
                      segment, (int)vsync_triggered);
        lastVsyncWarnMs = now;
      }
      if (++vsyncMisses >= 4) { reset(); break; }
      segment--; // retry this segment
      continue;
    }
    vsyncMisses = 0;

    error = 0;
    uint16_t innerLoops = 0;
    do
    {
      for (line = 0; line < 60; line++)
      {
        if (error == 255)
        {
          segment = 0; // outer for-loop will increment to 1
          error = 0;
          line = 60;   // satisfy while(line!=60) so do-while exits cleanly;
                       // without this, do-while continues with segment=0 and
                       // savePackage(line,0) underflows (segment-1)=255,
                       // computing addr_base=44352 and writing thermal data
                       // 50 KB past smallBuffer into FreeRTOS heap (TCBs).
          reset();
          break;
        }

        int retVal = getPackage(line, segment);

        if (retVal == 0)
        {
          if (savePackage(line, segment)) continue;
        }

        error++;
        ESP_DelayUS(900);
        break;
      }
      if (++innerLoops > 400) { // ~360ms cap (400 * 900µs)
        reset();
        segment = 0;
        break;
      }
    } while (line != 60);
  }

  doGetCommand(CMD_SYS_FPA_TEMPERATURE_KELVIN, &fpa_temp);
  doGetCommand(CMD_SYS_AUX_TEMPERATURE_KELVIN, &aux_temp);
  end();
}

void Lepton::reset()
{
  end();
  delay(186);
  syncFrame();
}
uint16_t Lepton::syncFrame() {
  
   lepton_spi->beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
   digitalWrite(_ssPin, LOW);
   //delay(0.02);
   ESP_DelayUS(20);
   
    static int count;
    if (count < 1) delay(1000);
    count++;
    if(count>=5)
    count = 5;
    return 0;
}

void Lepton::end() {
  digitalWrite(_ssPin, HIGH);
  lepton_spi->endTransaction();
}

int Lepton::readFrame(uint16_t* data) {
  for (byte segment = 1; segment <= 4; segment++){
  uint16_t row = 0;
  uint16_t id = waitNextFrame();
  while ((id & 0xfff) == row) {
    (void)readFrameWord();   // CRC word: must be consumed to advance the frame, value unused
    for (int col = 0; col < 80; col++) {
      data[(segment - 1) * 4800 + row * 80 + col] = readFrameWord();
    }

    if ((row == 20)){
    //byte seg = (id >> 12);
    //if (seg == 0)
     // return 1;
    ///if (segment != seg)
     // return 2;
    }
    //Serial.printf("row = %d, segment = %d,  id = %d,id_row = %d\n", row, segment, id >> 12, id & 0xfff);
    row++;  
    if (row < 60) {
      id = readFrameWord();
    } else {
      //return 1;
      break;
    }
    //Serial.printf("................readFrame ended with row %4x != id %4x\n", row, id);
  }

  
   //for(;row<60;row++)
  //for (int col = 0; col < 80; col++) {
    //  data[(segment - 1) * 4800 + row * 80 + col] = 0x1f40;
    //}
  //Serial.printf("readFrame ended with row %4x != id %4x\n", row, id);
  }
  return 0;
}

/*
uint16_t Lepton::wait_160x120_Seg(void)
{
  //uint16_t Lepton::wait_160X120_NextFrame() {
  uint16_t id = readFrameWord();
  Serial.printf("id =  %d\n", id);
  while ((id & 0x0f00) == 0x0f00) {
    for (int i = 0; i < 161; i++) {
      readFrameWord();
    }
    id = readFrameWord();
    //Serial.printf("id =  %d\n", id);
  }
  return id;    
}
}
*/
int Lepton::read_160x120_Frame(uint16_t* data) {
  uint16_t row = 0;
  uint16_t id = waitNextFrame();
  while ((id & 0xfff) == row) {
    (void)readFrameWord();   // CRC word: must be consumed to advance the frame, value unused
    for (int col = 0; col < 80; col++) {
      data[row * 80 + col] = readFrameWord();
    }

   // Serial.printf(" %d %d %d\n",  (id & 0x7000)>>12 ,(id & 0xfff) , row);
    row++;
    if (row < 60) {
      id = readFrameWord();
    } else {
      return 1;
    }
    //Serial.printf("................readFrame ended with row %4x != id %4x\n", row, id);
  }
  //Serial.printf("readFrame ended with row %4x != id %4x\n", row, id);
  return 0;
}



void Lepton::readFrameRaw(uint16_t* data) {
  data[0] = waitNextFrame();
  for (int i = 1; i < 82 * 60; i++) {
    data[i] = readFrameWord();
  }
}

void Lepton::startTransmission(uint16_t reg) {
  Wire1.beginTransmission(DEVICE_ID);
  transmitWord(reg);
}

void Lepton::transmitWord(uint16_t value) {
  Wire1.write(value >> 8 & 0xff);
  Wire1.write(value & 0xff);    
}

void Lepton::endTransmission() {
  uint8_t error = Wire1.endTransmission();
  if (error != 0) {
    Serial.print("error=");
    Serial.println(error);
  }    
}

uint16_t Lepton::readWord() {
  uint16_t value = Wire1.read() << 8;
  value |= Wire1.read();
  return value;
}

void Lepton::setRegister(uint16_t reg) {
  startTransmission(reg);
  endTransmission();
}

void Lepton::waitIdle() {
  uint32_t deadline = millis() + 2000;
  while ((readRegister(REG_STATUS) & STATUS_BIT_BUSY) && (millis() < deadline)) {
  }
  if (millis() >= deadline) {
    Serial.println("[Lepton] waitIdle timeout — camera may not be responding on I2C");
  }
}

uint16_t Lepton::readData(uint16_t* data) {
  uint16_t dataLen = readRegister(REG_DATA_LEN) / 2; // The data sheet says the data length register is in 16-bit words, but it actually seems to be in bytes
  setRegister(REG_DATA_BASE);
  Wire1.requestFrom(DEVICE_ID, (uint8_t)(dataLen * 2));
  for (int i = 0; i < dataLen; i++) {
    data[i] = readWord();
  }
  // TODO Check CRC
  return dataLen;
}

void Lepton::writeData(uint16_t* data, uint16_t dataLen) {
  startTransmission(REG_DATA_LEN);
  transmitWord(dataLen);
  for (int i = 0; i < dataLen; i++) {
    transmitWord(data[i]);
  }
  endTransmission();
}

uint16_t Lepton::readFrameWord() {
  uint16_t data = lepton_spi->transfer(0x00) << 8;
  data |= lepton_spi->transfer(0x00);
  return data;
}

uint16_t Lepton::waitNextFrame() {
  uint16_t id = readFrameWord();
  //Serial.printf("id =  %d\n", id);
  while ((id & 0x0f00) == 0x0f00) {
    for (int i = 0; i < 81; i++) {
      readFrameWord();
    }
    id = readFrameWord();
    //Serial.printf(" %d %d %d\n",  id,(id & 0xfff) , (id & 0x0f00));
     //Serial.printf("ID =  %X \n",  id);
  }
  //Serial.printf("idwhile =  %d\n", id);
  return id;    
}
uint16_t Lepton::wait_160X120_NextFrame() {
  /*
  uint16_t seg = readFrameWord();
  Serial.printf("seg =  %d\n", seg);
  //while ((id & 0x0e00) == 0x0e00) {
    
    for (int i = 0; i < 81; i++) {
    id =  readFrameWord();
    }
    id = readFrameWord();
    //Serial.printf("id =  %d\n", id);
  */
  uint16_t seg = 0;
  for(int i = 0; i < 82 * 60; i++)
  {
       seg = readFrameWord();
       Serial.printf("seg =  %d ", (seg & 0x7000)>>12);Serial.printf("id =  %d\n", seg & 0x0fff);
  }
  return seg;
}


void Lepton::dumpHex(uint16_t *data, int dataLen) {
  for (int i = 0; i < dataLen; i++) {
    Serial.printf("%4x ", data[i]);
  }
  Serial.println();
}
