#include <SPI.h>

#define PIN_BTN 2
#define PIN_CE 8
#define PIN_IRQ 9
#define PIN_SS 10
#define PIN_MOSI 11
#define PIN_MISO 12
#define PIN_SCK 13

#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_OBSERVE_TX  0x08
#define REG_RPD         0x09
#define REG_RX_ADDR_P0  0x0A
#define REG_RX_ADDR_P1  0x0B
#define REG_RX_ADDR_P2  0x0C
#define REG_RX_ADDR_P3  0x0D
#define REG_RX_ADDR_P4  0x0E
#define REG_RX_ADDR_P5  0x0F
#define REG_TX_ADDR     0x10
#define REG_RX_PW_P0    0x11
#define REG_RX_PW_P1    0x12
#define REG_RX_PW_P2    0x13
#define REG_RX_PW_P3    0x14
#define REG_RX_PW_P4    0x15
#define REG_RX_PW_P5    0x16
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD       0x1C
#define REG_FEATURE     0x1D

void dump_regs(uint8_t pin);

void readReg(uint8_t csPin, uint8_t addr, uint8_t length, uint8_t* data)
{
  SPI.begin();
  digitalWrite(csPin, LOW);
  SPI.transfer(addr & 0x1F);
  while(length > 0)
  {
    *data = SPI.transfer(0xFF);
    ++data;
    --length;
  }
  SPI.end();
  digitalWrite(csPin, HIGH);
}

void writeReg(uint8_t csPin, uint8_t addr, uint8_t data)
{
  writeReg(csPin, addr, 1, &data);
}

void writeReg(uint8_t csPin, uint8_t addr, uint8_t length, uint8_t* data)
{
  SPI.begin();
  digitalWrite(csPin, LOW);
  SPI.transfer(0x20 | (addr & 0x1F));
  while(length > 0)
  {
    SPI.transfer(*data);
    ++data;
    --length;
  }
  SPI.end();
  digitalWrite(csPin, HIGH);
}

void transmit(uint8_t csPin, uint8_t cePin, uint8_t length, uint8_t* data)
{
  SPI.begin();
  digitalWrite(csPin, LOW);
  SPI.transfer(0xA0);
  while(length > 0)
  {
    SPI.transfer(*data);
    ++data;
    --length;
  }
  SPI.end();
  digitalWrite(csPin, HIGH);
  
  digitalWrite(cePin, HIGH);
  // This covers both Thce and Tpece2csn
  delayMicroseconds(10);
  digitalWrite(cePin, LOW);
}

void receive(uint8_t csPin, uint8_t cePin, uint8_t length, uint8_t* data)
{
  digitalWrite(cePin, LOW);
  
  SPI.begin();
  digitalWrite(csPin, LOW);
  SPI.transfer(0x61);
  while(length > 0)
  {
    *data = SPI.transfer(0xFF);
    ++data;
    --length;
  }
  SPI.end();
  digitalWrite(csPin, HIGH);
  
  digitalWrite(cePin, HIGH);
  // Cover Tpece2csn just in case
  delayMicroseconds(4);
}

void flushTX(uint8_t csPin)
{
  SPI.begin();
  digitalWrite(csPin, LOW);
  SPI.transfer(0xE1);
  digitalWrite(csPin, HIGH);
}

void flushRX(uint8_t csPin)
{
  SPI.begin();
  digitalWrite(csPin, LOW);
  SPI.transfer(0xE2);
  digitalWrite(csPin, HIGH);
}

void initTX(uint8_t csPin)
{
  writeReg(csPin, REG_CONFIG, 0x0E); // EN_CRC, CRCO, PWR_UP
  writeReg(csPin, REG_EN_AA, 0x01); // Enable auto ack on pipe 0
  writeReg(csPin, REG_EN_RXADDR, 0x01); // Receive on pipe 0
  writeReg(csPin, REG_SETUP_AW, 0x03); // 5 bit address
  // NOTE that ARD >= 500 us retransmit delay is critical if we are using 250 kbps, because anything
  // less isn't enough time to actually send the ACK in.
  writeReg(csPin, REG_SETUP_RETR, 0x1F); // 500 us retransmit delay, up to 15 retransmits
  writeReg(csPin, REG_RF_CH, 0x02); // Channel 2
  writeReg(csPin, REG_RF_SETUP, 0x26); // 250 kbps 0 dBm
  
  // NOTE: You have to set up to RECEIVE ON THE ADDRESS YOU TRANSMIT TO
  // to support auto ack!
  uint8_t txAddr[] = {0x01, 0x78, 0x56, 0x34, 0x12};
  
  writeReg(csPin, REG_TX_ADDR, 5, txAddr);
  writeReg(csPin, REG_RX_ADDR_P0, 5, txAddr);
  
  writeReg(csPin, REG_RX_PW_P0, 1);
  writeReg(csPin, REG_DYNPD, 0);
  writeReg(csPin, REG_FEATURE, 0);
}

void dumpStatus(uint8_t statusReg)
{
  Serial.print("Status: ");
  if(statusReg & (1<<6))
    Serial.print("RX_DR; ");
  if(statusReg & (1<<5))
    Serial.print("TX_DS; ");
  if(statusReg & (1<<4))
    Serial.print("MAX_RT; ");
  if(statusReg & (1<<0))
    Serial.print("TX_FULL; ");
  uint8_t rxPipeNo = (statusReg >> 1) & 0x7;
  if(rxPipeNo == 0x7)
    Serial.print("RX FIFO empty");
  else
  {
    Serial.print("RX FIFO ");
    Serial.print(rxPipeNo, DEC);
    Serial.print(" contains data");
  }
  Serial.println();
}

static char helloWorld[32] = "Hello World!";

static uint8_t lastButtonVal = 0;

void setup()
{
  digitalWrite(PIN_CE, LOW);
  digitalWrite(PIN_SS, HIGH);
  digitalWrite(PIN_MOSI, LOW);
  digitalWrite(PIN_SCK, LOW);
  
  // pullup  
  pinMode(PIN_CE, OUTPUT);
  pinMode(PIN_SS, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);
  
  pinMode(PIN_IRQ, INPUT); 
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_BTN, INPUT);
  digitalWrite(PIN_BTN, HIGH);
  
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  
  Serial.begin(115200);
  
  // wait for module to boot up (generously)
  delay(250);
  
  Serial.println("Initializing module");
  initTX(PIN_SS);
  flushTX(PIN_SS);
  dump_regs(PIN_SS);
  Serial.println("Module init completed");
  
  // wait for power on (generously)
  delay(250);
}

void loop()
{  
  while(1)
  {
    uint8_t buttonVal = (PIND & _BV(2)) ? 0xFF : 0x00;
    if(buttonVal != lastButtonVal)
    {
        //Serial.println("button change");
      lastButtonVal = buttonVal;      
      helloWorld[0] = buttonVal;
      transmit(PIN_SS, PIN_CE, 1, (uint8_t*)helloWorld);
      
      while(digitalRead(PIN_IRQ));
    
      uint8_t statusRegister;
      readReg(PIN_SS, REG_STATUS, 1, &statusRegister);
      if(statusRegister & (1<<5))
      {
        /*Serial.println("IRQ reason: Data was sent!");*/
        // TODO: maybe unnecessary
        flushTX(PIN_SS);
        writeReg(PIN_SS, REG_STATUS, 0x70);
      }
      else if(statusRegister & (1<<4))
      {
        Serial.println("Max retransmissions hit!");
        /*uint8_t txObs;
        readReg(PIN_SS, REG_OBSERVE_TX, 1, &txObs);
        Serial.print("  Packets lost: "); Serial.println(txObs >> 4, DEC);
        Serial.print("  Retransmits: "); Serial.println(txObs & 0xF, DEC);*/
        flushTX(PIN_SS);
        writeReg(PIN_SS, REG_STATUS, 0x70);
      }
    }    
  }
  /*
  Serial.println("Transmitting...");
  transmit(PIN_SS, PIN_CE, sizeof(helloWorld), (uint8_t*)helloWorld);
  
  Serial.println("Waiting for IRQ...");
  while(digitalRead(PIN_IRQ));
  
  uint8_t statusRegister;
  readReg(PIN_SS, REG_STATUS, 1, &statusRegister);
  if(statusRegister & (1<<5))
  {
    Serial.println("IRQ reason: Data was sent!");
    flushTX(PIN_SS);
    writeReg(PIN_SS, REG_STATUS, 0x70);
  }
  else if(statusRegister & (1<<4))
  {
    Serial.println("IRQ reason: Max retransmissions hit!");
    uint8_t txObs;
    readReg(PIN_SS, REG_OBSERVE_TX, 1, &txObs);
    Serial.print("  Packets lost: "); Serial.println(txObs >> 4, DEC);
    Serial.print("  Retransmits: "); Serial.println(txObs & 0xF, DEC);
    flushTX(PIN_SS);
    writeReg(PIN_SS, REG_STATUS, 0x70);
  }

  delay(500);*/
}

  
void dump_regs(uint8_t pin)
{
  uint8_t val;
    
  // BEGIN CONFIG
  readReg(pin, 0x00, 1, &val);
  
  if((val & 0x70) != 0x70)
  {
    Serial.print("IRQ enabled for ");
    
    bool first = true;
    
    if(!(val & (1<<6)))
    {
      if(!first)
        Serial.print(", ");
      else
        first = false;
      Serial.print("RX_DR");
    }
    
    if(!(val & (1<<5)))
    {
      if(!first)
        Serial.print(", ");
      else
        first = false;
      Serial.print("TX_DS");
    }
    
    if(!(val & (1<<4)))
    {
      if(!first)
        Serial.print(", ");
      else
        first = false;
      Serial.print("MAX_RT");
    }
    
    Serial.print("; ");
  }
  else
  {
    Serial.print("IRQs all disabled; ");
  }
  
  if(val & (1<<3))
    Serial.print("CRC Enabled; ");
  else
    Serial.print("CRC Disabled; ");
    
  if(val & (1<<2))
    Serial.print("2 byte CRC; ");
  else
    Serial.print("1 byte CRC; ");
    
  if(val & (1<<1))
    Serial.print("Power Up; ");
  else
    Serial.print("Power Down; ");
    
  if(val & (1<<0))
    Serial.print("Receiver");
  else
    Serial.print("Transmitter");
    
  Serial.println();
  // END CONFIG
  
  // BEGIN SETUP_AW  
  readReg(pin, 0x03, 1, &val);
  Serial.print("Address width: ");
  const char* aw;
  switch(val & 0x3)
  {    
    case 0: aw = "!!!Illegal!!!"; break;
    case 1: aw = "3 bytes"; break;
    case 2: aw = "4 bytes"; break;
    case 3: aw = "5 bytes"; break;
  }
  Serial.println(aw);
  // END SETUP_AW register
  
  // BEGIN SETUP_RETR register  
  readReg(pin, 0x04, 1, &val);
  Serial.print("Auto Retransmit Interval: ");
  uint16_t interval = (val & 0xF0) >> 4;
  interval += 1;
  interval *= 250;
  Serial.print(interval, DEC);
  Serial.print(" us; Auto Retransmit Count: ");
  Serial.println((val & 0x0F));
  // END SETUP_RETR register
  
  // BEGIN RF_CH register  
  readReg(pin, 0x05, 1, &val);
  Serial.print("RF Channel: ");
  int ch = val & 0x7F;
  Serial.print(ch);
  Serial.print(" (2.");  
  Serial.print(ch+400, DEC);
  // TODO: Calculate GHz value
  Serial.println(" GHz)");
  // END RF_CH register
  
  // BEGIN RF_SETUP register  
  readReg(pin, 0x06, 1, &val);
  Serial.print("Continuous Carrier TX ");
  Serial.print(val & (1<<7) ? "ON" : "OFF");
  Serial.print("; Force PLL Lock ");
  Serial.print(val & (1<<4) ? "ON" : "OFF");
  Serial.print("; ");
  uint8_t rf_dr_low = (val>>5)&0x1;
  uint8_t rf_dr_high = (val>>3)&0x1;
  switch((rf_dr_low<<1) | rf_dr_high) {
    case 0: Serial.print("1 Mbps"); break;
    case 1: Serial.print("2 Mbps"); break;
    case 2: Serial.print("250 kbps"); break;
    case 3: Serial.print("**RESERVED RATE**"); break;
  }
  Serial.print("; Transmit Power ");
  uint8_t rf_pwr = (val>>1)&0x3;
  switch(rf_pwr) {
    case 0: Serial.print("-18 dBm");
    case 1: Serial.print("-12 dBm");
    case 2: Serial.print("-6 dBm");
    case 3: Serial.print("0 dBm");
  }
  Serial.println();
  // END RF_SETUP register
  
  // Begin STATUS register
  readReg(pin, 0x07, 1, &val);
  Serial.print("");
  Serial.print((val & (1<<6)) ? "Data Ready; " : "No Data Ready; ");
  Serial.print((val & (1<<5)) ? "Data Sent; " : "Data Not Sent; ");
  Serial.print((val & (1<<4)) ? "Max Retransmits Hit; " : "Max Retransmits Not Hit; ");
  uint8_t plno = (val>>1) & 0x7;
  switch(plno)
  {
    case 0: case 1: case 2: case 3: case 4: case 5:
      Serial.print("RX FIFO has data from pipe ");
      Serial.print(plno, DEC);
      Serial.print("; ");
      break;
    case 6:
      Serial.print("INVALID RX_P_NO!!! ");
      break;
    case 7:
      Serial.print("RX FIFO empty; ");
      break;
  }
  Serial.println((val & 0x1) ? "TX FIFO full" : "TX FIFO not full");
  // End STATUS register
  
  // Begin OBSERVE_TX
  readReg(pin, 0x08, 1, &val);
  Serial.print("Packets lost: ");
  Serial.print((val >> 4) & 0xF, DEC);
  Serial.print("; Retransmitted: ");
  Serial.println(val & 0xF, DEC);  
  // End OBSERVE_TX
  
  // Begin RPD
  readReg(pin, 0x09, 1, &val);
  Serial.println(val & 0x1 ? "Carrier detected" : "No carrier detected");
  // End RPD
  
  // Display pipe info
  uint8_t autoAck;
  uint8_t enableAddr;
  uint8_t dynamicPayloadLen;
  uint8_t addr[5];
  readReg(pin, 0x01, 1, &autoAck);
  readReg(pin, 0x02, 1, &enableAddr);
  readReg(pin, 0x1C, 1, &dynamicPayloadLen);
  
  // EN_AA, EN_RXADDR, RX_ADDR_Pn, RX_PW_Pn, DYNPD
  Serial.println("RX Pipes:");
  for(uint8_t i=0; i<6; ++i)
  {
    uint8_t len = 5;
    if(i >= 2)
      len = 1;
    readReg(pin, 0x0A + i, len, addr);
    Serial.print(i,DEC);
    Serial.print(": ");
    print_addr(addr);
    Serial.print(autoAck & (1<<i) ? " AA" : "   ");
    Serial.print(enableAddr & (1<<i) ? " RX" : "   ");
    Serial.print(dynamicPayloadLen & (1<<i) ? " DPL" : "    ");
    
    uint8_t pw;
    readReg(pin, 0x11 + i, 1, &pw);
    
    Serial.print("PW=");
    Serial.print(pw,DEC);
    Serial.println(" bytes");
  }
  
  // Begin TX_ADDR
  readReg(pin, 0x10, 5, addr);
  Serial.print("TX Addr: ");
  print_addr(addr);
  Serial.println();
  // End TX_ADDR
  
  // Begin FIFO_STATUS
  readReg(pin, 0x17, 1, &val);
  Serial.print((val & (1<<6)) ? "Reusing TX payload; " : "Not reusing TX payload; ");
  Serial.print((val & (1<<5)) ? "TX FIFO full; " : "TX FIFO not full; ");
  Serial.print((val & (1<<4)) ? "TX FIFO empty; " : "TX FIFO nonempty; ");
  Serial.print((val & (1<<1)) ? "RX FIFO full; " : "RX FIFO not full; ");
  Serial.println((val & (1<<0)) ? "RX FIFO empty" : "RX FIFO nonempty");
  // End FIFO_STATUS
  
  // Begin FEATURE
  readReg(pin, 0x1D, 1, &val);
  Serial.print((val & (1<<2)) ? "Dynamic Payload Length Enabled; " : "Dynamic Payload Length Disabled; ");  
  Serial.print((val & (1<<1)) ? "ACK Payload Enabled; " : "ACK Payload Disabled; ");
  Serial.println((val & (1<<0)) ? "Dynamic ACK enabled" : "Dynamic ACK Disabled");
  // End FEATURE
  
  Serial.println();
}

void print_addr(uint8_t* addr)
{
  Serial.print("0x");
  for(int i = 4; i >= 0; --i) {
    if(addr[i] < 10)
      Serial.print("0");
    Serial.print(addr[i], HEX);
  }  
}

