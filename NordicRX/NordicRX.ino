#include <SPI.h>
#include <avr/pgmspace.h>

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

void print_p(const char* pstr)
{
  while(pgm_read_byte(pstr)) {
    Serial.write(pgm_read_byte(pstr));
    ++pstr;
  }
}
  
void println_p(const char* pstr)
{
  print_p(pstr);
  print_p(PSTR("\r\n"));
}

#define PRINT_P(__str) print_p(PSTR(__str))
#define PRINTLN_P(__str) println_p(PSTR(__str))

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

void initRX(uint8_t csPin)
{
  writeReg(csPin, REG_CONFIG, 0x0F); // EN_CRC, CRCO, PWR_UP, PRIM_RX
  writeReg(csPin, REG_EN_AA, 0x01); // Enable auto ack on pipe 0
  writeReg(csPin, REG_EN_RXADDR, 0x01); // Receive on pipe 0
  writeReg(csPin, REG_SETUP_AW, 0x03); // 5 bit address
  writeReg(csPin, REG_SETUP_RETR, 0x1F); // 500 us retransmit delay, up to 15 retransmits
  writeReg(csPin, REG_RF_CH, 0x02); // Channel 2
  writeReg(csPin, REG_RF_SETUP, 0x26); // 250 kbps 0 dBm
  
  uint8_t rxAddr[] = {0x01, 0x78, 0x56, 0x34, 0x12};
  writeReg(csPin, REG_RX_ADDR_P0, 5, rxAddr);
  
  writeReg(csPin, REG_RX_PW_P0, 32);
  writeReg(csPin, REG_DYNPD, 0);
  writeReg(csPin, REG_FEATURE, 0);
}

void dumpStatus(uint8_t statusReg)
{
  PRINT_P("Status: ");
  if(statusReg & (1<<6))
    PRINT_P("RX_DR; ");
  if(statusReg & (1<<5))
    PRINT_P("TX_DS; ");
  if(statusReg & (1<<4))
    PRINT_P("MAX_RT; ");
  if(statusReg & (1<<0))
    PRINT_P("TX_FULL; ");
  uint8_t rxPipeNo = (statusReg >> 1) & 0x7;
  if(rxPipeNo == 0x7)
    PRINT_P("RX FIFO empty");
  else
  {
    PRINT_P("RX FIFO ");
    Serial.print(rxPipeNo, DEC);
    PRINT_P(" contains data");
  }
  Serial.println();
}

static char helloWorld[32] = "Hello World!";

void setup()
{
  digitalWrite(PIN_CE, LOW);
  digitalWrite(PIN_SS, HIGH);
  digitalWrite(PIN_MOSI, LOW);
  digitalWrite(PIN_SCK, LOW);
  
  pinMode(PIN_CE, OUTPUT);
  pinMode(PIN_SS, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_IRQ, INPUT); 
  pinMode(PIN_MISO, INPUT);
  
  digitalWrite(PIN_CE, LOW);
  digitalWrite(PIN_SS, HIGH);
  digitalWrite(PIN_MOSI, LOW);
  digitalWrite(PIN_SCK, LOW);
  
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  
  Serial.begin(115200);
  
  // wait for module to boot up (generously)
  delay(250);
  
  PRINTLN_P("Initializing module");
  initRX(PIN_SS);
  flushRX(PIN_SS);
  dump_regs(PIN_SS);
  writeReg(PIN_SS, REG_STATUS, 0x70);
  PRINTLN_P("Module init completed");
    
  // wait for power on (generously)
  delay(250);
  
  digitalWrite(PIN_CE, HIGH);
}

void loop()
{
  // TODO: Should we drop the CE pin in here somewhere?
  if(digitalRead(PIN_IRQ) != HIGH)
  {    
    Serial.println("IRQ asserted");
    //uint8_t statusRegister;
    uint8_t fifoStatus;
    //readReg(CS_1, REG_STATUS, 1, &statusRegister);
    readReg(PIN_SS, REG_FIFO_STATUS, 1, &fifoStatus);
    
    bool read = false;
    
    // While FIFO is not empty
    while(!(fifoStatus & 0x01))
    {
      read = true;
      char buf[32];
      receive(PIN_SS, PIN_CE, sizeof(buf), (uint8_t*)buf);
      Serial.println("Received data: ");
      for(int i=0; i<32; ++i)
        Serial.write(buf[i]);
      Serial.println();
      
      readReg(PIN_SS, REG_FIFO_STATUS, 1, &fifoStatus);
    }
    
    // clear status register to clear the assert
    writeReg(PIN_SS, REG_STATUS, 0x70);
    
    if(read) 
    {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
    }
  }
}
  
  
void dump_regs(uint8_t pin)
{
  uint8_t val;
    
  // BEGIN CONFIG
  readReg(pin, 0x00, 1, &val);
  
  if((val & 0x70) != 0x70)
  {
    PRINT_P("IRQ enabled for ");
    
    bool first = true;
    
    if(!(val & (1<<6)))
    {
      if(!first)
        PRINT_P(", ");
      else
        first = false;
      PRINT_P("RX_DR");
    }
    
    if(!(val & (1<<5)))
    {
      if(!first)
        PRINT_P(", ");
      else
        first = false;
      PRINT_P("TX_DS");
    }
    
    if(!(val & (1<<4)))
    {
      if(!first)
        PRINT_P(", ");
      else
        first = false;
      PRINT_P("MAX_RT");
    }
    
    PRINT_P("; ");
  }
  else
  {
    PRINT_P("IRQs all disabled; ");
  }
  
  if(val & (1<<3))
    PRINT_P("CRC Enabled; ");
  else
    PRINT_P("CRC Disabled; ");
    
  if(val & (1<<2))
    PRINT_P("2 byte CRC; ");
  else
    PRINT_P("1 byte CRC; ");
    
  if(val & (1<<1))
    PRINT_P("Power Up; ");
  else
    PRINT_P("Power Down; ");
    
  if(val & (1<<0))
    PRINT_P("Receiver");
  else
    PRINT_P("Transmitter");
    
  Serial.println();
  // END CONFIG
  
  // BEGIN SETUP_AW  
  readReg(pin, 0x03, 1, &val);
  PRINT_P("Address width: ");
  const char* aw;
  switch(val & 0x3)
  {    
    case 0: aw = PSTR("!!!Illegal!!!"); break;
    case 1: aw = PSTR("3 bytes"); break;
    case 2: aw = PSTR("4 bytes"); break;
    case 3: aw = PSTR("5 bytes"); break;
  }
  println_p(aw);
  // END SETUP_AW register
  
  // BEGIN SETUP_RETR register  
  readReg(pin, 0x04, 1, &val);
  PRINT_P("Auto Retransmit Interval: ");
  uint16_t interval = (val & 0xF0) >> 4;
  interval += 1;
  interval *= 250;
  Serial.print(interval, DEC);
  PRINT_P(" us; Auto Retransmit Count: ");
  Serial.println((val & 0x0F));
  // END SETUP_RETR register
  
  // BEGIN RF_CH register  
  readReg(pin, 0x05, 1, &val);
  PRINT_P("RF Channel: ");
  int ch = val & 0x7F;
  Serial.print(ch);
  PRINT_P(" (2.");  
  Serial.print(ch+400, DEC);
  // TODO: Calculate GHz value
  PRINTLN_P(" GHz)");
  // END RF_CH register
  
  // BEGIN RF_SETUP register  
  readReg(pin, 0x06, 1, &val);
  PRINT_P("Continuous Carrier TX ");
  Serial.print(val & (1<<7) ? "ON" : "OFF");
  PRINT_P("; Force PLL Lock ");
  Serial.print(val & (1<<4) ? "ON" : "OFF");
  PRINT_P("; ");
  uint8_t rf_dr_low = (val>>5)&0x1;
  uint8_t rf_dr_high = (val>>3)&0x1;
  switch((rf_dr_low<<1) | rf_dr_high) {
    case 0: PRINT_P("1 Mbps"); break;
    case 1: PRINT_P("2 Mbps"); break;
    case 2: PRINT_P("250 kbps"); break;
    case 3: PRINT_P("**RESERVED RATE**"); break;
  }
  PRINT_P("; Transmit Power ");
  uint8_t rf_pwr = (val>>1)&0x3;
  switch(rf_pwr) {
    case 0: PRINT_P("-18 dBm");
    case 1: PRINT_P("-12 dBm");
    case 2: PRINT_P("-6 dBm");
    case 3: PRINT_P("0 dBm");
  }
  Serial.println();
  // END RF_SETUP register
  
  // Begin STATUS register
  readReg(pin, 0x07, 1, &val);
  PRINT_P("");
  print_p((val & (1<<6)) ? PSTR("Data Ready; ") : PSTR("No Data Ready; "));
  print_p((val & (1<<5)) ? PSTR("Data Sent; ") : PSTR("Data Not Sent; "));
  print_p((val & (1<<4)) ? PSTR("Max Retransmits Hit; ") : PSTR("Max Retransmits Not Hit; "));
  uint8_t plno = (val>>1) & 0x7;
  switch(plno)
  {
    case 0: case 1: case 2: case 3: case 4: case 5:
      PRINT_P("RX FIFO has data from pipe ");
      Serial.print(plno, DEC);
      PRINT_P("; ");
      break;
    case 6:
      PRINT_P("INVALID RX_P_NO!!! ");
      break;
    case 7:
      PRINT_P("RX FIFO empty; ");
      break;
  }
  println_p((val & 0x1) ? PSTR("TX FIFO full") : PSTR("TX FIFO not full"));
  // End STATUS register
  
  // Begin OBSERVE_TX
  readReg(pin, 0x08, 1, &val);
  PRINT_P("Packets lost: ");
  Serial.print((val >> 4) & 0xF, DEC);
  PRINT_P("; Retransmitted: ");
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
  PRINTLN_P("RX Pipes:");
  for(uint8_t i=0; i<6; ++i)
  {
    uint8_t len = 5;
    if(i >= 2)
      len = 1;
    readReg(pin, 0x0A + i, len, addr);
    Serial.print(i,DEC);
    PRINT_P(": ");
    print_addr(addr);
    // TODO: PRINT_P
    print_p(autoAck & (1<<i) ? PSTR(" AA") : PSTR("   "));
    print_p(enableAddr & (1<<i) ? PSTR(" RX") : PSTR("   "));
    print_p(dynamicPayloadLen & (1<<i) ? PSTR(" DPL") : PSTR("    "));
    
    uint8_t pw;
    readReg(pin, 0x11 + i, 1, &pw);
    
    PRINT_P("PW=");
    Serial.print(pw,DEC);
    PRINTLN_P(" bytes");
  }
  
  // Begin TX_ADDR
  readReg(pin, 0x10, 5, addr);
  PRINT_P("TX Addr: ");
  print_addr(addr);
  Serial.println();
  // End TX_ADDR
  
  // Begin FIFO_STATUS
  readReg(pin, 0x17, 1, &val);
  print_p((val & (1<<6)) ? PSTR("Reusing TX payload; ") : PSTR("Not reusing TX payload; "));
  print_p((val & (1<<5)) ? PSTR("TX FIFO full; ") : PSTR("TX FIFO not full; "));
  print_p((val & (1<<4)) ? PSTR("TX FIFO empty; ") : PSTR("TX FIFO nonempty; "));
  print_p((val & (1<<1)) ? PSTR("RX FIFO full; ") : PSTR("RX FIFO not full; "));
  println_p((val & (1<<0)) ? PSTR("RX FIFO empty") : PSTR("RX FIFO nonempty"));
  // End FIFO_STATUS
  
  // Begin FEATURE
  readReg(pin, 0x1D, 1, &val);
  print_p((val & (1<<2)) ? PSTR("Dynamic Payload Length Enabled; ") : PSTR("Dynamic Payload Length Disabled; "));  
  print_p((val & (1<<1)) ? PSTR("ACK Payload Enabled; ") : PSTR("ACK Payload Disabled; "));
  println_p((val & (1<<0)) ? PSTR("Dynamic ACK enabled") : PSTR("Dynamic ACK Disabled"));
  // End FEATURE
  
  Serial.println();
}

void print_addr(uint8_t* addr)
{
  PRINT_P("0x");
  for(int i = 4; i >= 0; --i) {
    if(addr[i] < 10)
      PRINT_P("0");
    Serial.print(addr[i], HEX);
  }  
}
