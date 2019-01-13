// Rough and ready 2.4 GHz band scanner using nRF24L01+ module with 128x64 graphic OLED display
//
// ceptimus.  November 2016.
// Edited by Benik3 January 2019 - added range select, dynamic displaying and async scanning with timer

#include "SSD1X06.h"

/* nRF24L01+ module connections

   module   Arduino
   1 GND ---- GND
   2 VCC ---- 3.3V  Note: 5V on VCC will destroy module (but other pins are 5V tolerant)
   3 CE ----- D9
   4 CSN ---- D10
   5 SCK ---- D13 (SCK)
   6 MOSI --- D11 (MOSI)
   7 MISO --- D12 (MISO)
   8 IRQ ---- not connected
*/

#define rstPin 4  //OLED reset pin
//I2C is "overclocked" to get faster refresh rates. See define in SSD1X06.h

// the nRF24L01+ can tune to 128 channels with 1 MHz spacing from 2.400 GHz to 2.527 GHz.
#define CHANNELS 128
#define STARTCHANNEL 0

// SPI definitions and macros
#define CE_pin    9
#define CS_pin   10
#define MOSI_pin 11
#define MISO_pin 12
#define SCK_pin  13

#define  CE_on    PORTB |= 0x02
#define  CE_off   PORTB &= 0xFD
#define  CS_on    PORTB |= 0x04
#define  CS_off   PORTB &= 0xFB
#define  MOSI_on  PORTB |= 0x08
#define  MOSI_off PORTB &= 0xF7
#define  MISO_on  (PINB & 0x10)  // input
#define  SCK_on   PORTB |= 0x20
#define  SCK_off  PORTB &= 0xDF

// nRF24 Register map
enum {
  NRF24L01_00_CONFIG      = 0x00,
  NRF24L01_01_EN_AA       = 0x01,
  NRF24L01_02_EN_RXADDR   = 0x02,
  NRF24L01_03_SETUP_AW    = 0x03,
  NRF24L01_04_SETUP_RETR  = 0x04,
  NRF24L01_05_RF_CH       = 0x05,
  NRF24L01_06_RF_SETUP    = 0x06,
  NRF24L01_07_STATUS      = 0x07,
  NRF24L01_08_OBSERVE_TX  = 0x08,
  NRF24L01_09_CD          = 0x09,
  NRF24L01_0A_RX_ADDR_P0  = 0x0A,
  NRF24L01_0B_RX_ADDR_P1  = 0x0B,
  NRF24L01_0C_RX_ADDR_P2  = 0x0C,
  NRF24L01_0D_RX_ADDR_P3  = 0x0D,
  NRF24L01_0E_RX_ADDR_P4  = 0x0E,
  NRF24L01_0F_RX_ADDR_P5  = 0x0F,
  NRF24L01_10_TX_ADDR     = 0x10,
  NRF24L01_11_RX_PW_P0    = 0x11,
  NRF24L01_12_RX_PW_P1    = 0x12,
  NRF24L01_13_RX_PW_P2    = 0x13,
  NRF24L01_14_RX_PW_P3    = 0x14,
  NRF24L01_15_RX_PW_P4    = 0x15,
  NRF24L01_16_RX_PW_P5    = 0x16,
  NRF24L01_17_FIFO_STATUS = 0x17,
  NRF24L01_1C_DYNPD       = 0x1C,
  NRF24L01_1D_FEATURE     = 0x1D,
  //Instructions
  NRF24L01_61_RX_PAYLOAD  = 0x61,
  NRF24L01_A0_TX_PAYLOAD  = 0xA0,
  NRF24L01_E1_FLUSH_TX    = 0xE1,
  NRF24L01_E2_FLUSH_RX    = 0xE2,
  NRF24L01_E3_REUSE_TX_PL = 0xE3,
  NRF24L01_50_ACTIVATE    = 0x50,
  NRF24L01_60_R_RX_PL_WID = 0x60,
  NRF24L01_B0_TX_PYLD_NOACK = 0xB0,
  NRF24L01_FF_NOP         = 0xFF,
  NRF24L01_A8_W_ACK_PAYLOAD0 = 0xA8,
  NRF24L01_A8_W_ACK_PAYLOAD1 = 0xA9,
  NRF24L01_A8_W_ACK_PAYLOAD2 = 0xAA,
  NRF24L01_A8_W_ACK_PAYLOAD3 = 0xAB,
  NRF24L01_A8_W_ACK_PAYLOAD4 = 0xAC,
  NRF24L01_A8_W_ACK_PAYLOAD5 = 0xAD,
};

// Bit mnemonics
enum {
  NRF24L01_00_MASK_RX_DR  = 6,
  NRF24L01_00_MASK_TX_DS  = 5,
  NRF24L01_00_MASK_MAX_RT = 4,
  NRF24L01_00_EN_CRC      = 3,
  NRF24L01_00_CRCO        = 2,
  NRF24L01_00_PWR_UP      = 1,
  NRF24L01_00_PRIM_RX     = 0,

  NRF24L01_07_RX_DR       = 6,
  NRF24L01_07_TX_DS       = 5,
  NRF24L01_07_MAX_RT      = 4,

  NRF2401_1D_EN_DYN_ACK   = 0,
  NRF2401_1D_EN_ACK_PAY   = 1,
  NRF2401_1D_EN_DPL       = 2,
};

enum TXRX_State {
  TXRX_OFF,
  TX_EN,
  RX_EN,
};

uint8_t MHz = STARTCHANNEL;
uint16_t signalStrength[128];   // smooths signal strength with numerical range 0 - 0x7FFF
uint8_t prevStrength[128];      // save signal strength displayed on OLED for comparison with actual value
uint8_t column = STARTCHANNEL;
uint16_t strength;
uint8_t row = 0;
uint8_t b = 0;
const uint8_t ff[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void setup() {
  pinMode(rstPin, OUTPUT);  // Set RST pin as OUTPUT
  digitalWrite(rstPin, LOW);  // Bring RST low, reset the display
  delay(10);  // wait 10ms
  digitalWrite(rstPin, HIGH); // Set RST HIGH, bring out of reset
  SSD1X06::start();
  delay(300);
  SSD1X06::fillDisplay(' ');
  SSD1X06::displayString6x8(1, 0, F("2.4GHz band scanner 4"), 0);
  SSD1X06::displayString6x8(3, 0, F("By ceptimus. Nov '16"), 0);
  SSD1X06::displayString6x8(5, 0, F("Mod by Benik3 Jan '19"), 0);
  // prepare 'bit banging' SPI interface
  pinMode(MOSI_pin, OUTPUT);
  pinMode(SCK_pin, OUTPUT);
  pinMode(CS_pin, OUTPUT);
  pinMode(CE_pin, OUTPUT);
  pinMode(MISO_pin, INPUT);
  CS_on;
  CE_on;
  MOSI_on;
  SCK_on;
  delay(70);
  CS_off;
  CE_off;
  MOSI_off;
  SCK_off;
  delay(100);
  CS_on;
  delay(10);

  NRF24L01_Reset();
  delay(150);

  NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);     // switch off Shockburst mode
  NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0x0F);  // write default value to setup register
  NRF24L01_SetTxRxMode(RX_EN);                    // switch to receive mode

  delay(3000); // start up message

  for (int x = 0; x < 128; x++) {
    uint8_t b = 0x01;  // baseline
    if (!(x % 10)) {
      b |= 0x06; // graduation tick every 10 MHz
    }
    if (x == 10 || x == 60 || x == 110) {
      b |= 0xF8; // scale markers at 2.41, 2.46, and 2.51 GHz
    }
    SSD1X06::displayByte(6, x, b);
  }
  SSD1X06::displayString6x8(7, 0, F("2.41"), 0);
  SSD1X06::displayString6x8(7, 50, F("2.46"), 0);
  SSD1X06::displayString6x8(7, 100, F("2.51"), 0);

  SSD1X06::displayString6x8(1, 0, F("                     "), 0); //clear lines with texts so there can't be any orphans on display
  SSD1X06::displayString6x8(3, 0, F("                     "), 0);
  SSD1X06::displayString6x8(5, 0, F("                     "), 0);

  //setup Timer1 for NRF scanning
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 45;                           // cca 6kHz
  TCCR1B |= (1 << WGM12);               // turn on CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10);  // Set 64 prescaler
  TIMSK1 |= (1 << OCIE1A);              // enable timer compare interrupt

  NRF24L01_WriteReg(NRF24L01_05_RF_CH, MHz);
  CE_on;        // start receiving
  sei();        // allow interrupts
}

ISR(TIMER1_COMPA_vect) {                                        //Timer2 intterupt vector, time to get data from nrf
  CE_off;                                                       // stop receiving - one bit is now set if received power was > -64 dBm at that instant
  if (NRF24L01_ReadReg(NRF24L01_09_CD)) {                       // signal detected so increase signalStrength unless already maxed out
    signalStrength[MHz] += (0x7FFF - signalStrength[MHz]) >> 5; // increase rapidly when previous value was low, with increase reducing exponentially as value approaches maximum
  } else {                                                      // no signal detected so reduce signalStrength unless already at minimum
    signalStrength[MHz] -= signalStrength[MHz] >> 5;            // decrease rapidly when previous value was high, with decrease reducing exponentially as value approaches zero
  }

  MHz++;
  if (MHz == CHANNELS + STARTCHANNEL) MHz = STARTCHANNEL;

  NRF24L01_WriteReg(NRF24L01_05_RF_CH, MHz);                    // Set new freqency for scan
  CE_on;                                                        // start receiving
  TCNT1  = 0;                                                   // clear timer counter
  OCR1A = random(35, 55);                                       // make the measuring slightly random in time
}

void loop() {
  if (CHANNELS + STARTCHANNEL > 128) {  //check if user didn't put wrong values for channel settings
    SSD1X06::displayString6x8(1, 2, F("Wrong channel config!"), 0);
  }
  else {
    strength = (signalStrength[column] + 0x0040) >> 7;
    if (strength > 48) {
      strength = 48; // limit to maximum height that fits display - 6 rows 8 bits
    }

    uint8_t arr[6] = {0};

    if (strength > prevStrength[column]) {
      uint8_t val = strength - ((prevStrength[column] / 8) * 8); //only the value to change
      row = 6 - (strength / 8);                                  //starting row, lower rows are 0 and unchanged
      if (strength % 8) row--;
      uint8_t nrow = ((val - 1) / 8);                            // Number of aditional rows to change

      memcpy(arr, ff, nrow + 1);                                // set bytes of array to 0xff
      if (val % 8) {                                            // The first row which may not contain full 8 pixels
        arr[0] = 0xFF << (8 - (val % 8));
      }

      SSD1X06::drawLine(row, column, nrow, arr);
      prevStrength[column] = strength;
    }

    else if (strength < prevStrength[column]) {
      row = 6 - (prevStrength[column] / 8);
      if (prevStrength[column] % 8) row--;
      uint8_t nrow = 5 - (strength / 8) - row;

      arr[nrow] = 0xFF << (8 - (strength % 8));
      SSD1X06::drawLine(row, column, nrow, arr);
      prevStrength[column] = strength;
    }

    column++;
    if (column == CHANNELS + STARTCHANNEL) column = STARTCHANNEL;
  }
}

uint8_t _spi_write(uint8_t command)
{
  uint8_t result = 0;
  uint8_t n = 8;
  SCK_off;
  MOSI_off;
  while (n--) {
    if (command & 0x80)
      MOSI_on;
    else
      MOSI_off;
    if (MISO_on)
      result |= 0x01;
    SCK_on;
    _NOP();
    SCK_off;
    command = command << 1;
    result = result << 1;
  }
  MOSI_on;
  return result;
}

void _spi_write_address(uint8_t address, uint8_t data)
{
  CS_off;
  _spi_write(address);
  _NOP();
  _spi_write(data);
  CS_on;
}

uint8_t _spi_read()
{
  uint8_t result = 0;
  uint8_t i;
  MOSI_off;
  _NOP();
  for (i = 0; i < 8; i++) {
    if (MISO_on) // if MISO is HIGH
      result = (result << 1) | 0x01;
    else
      result = result << 1;
    SCK_on;
    _NOP();
    SCK_off;
    _NOP();
  }
  return result;
}

uint8_t _spi_read_address(uint8_t address)
{
  uint8_t result;
  CS_off;
  _spi_write(address);
  result = _spi_read();
  CS_on;
  return (result);
}

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

uint8_t NRF24L01_WriteReg(uint8_t address, uint8_t data)
{
  CS_off;
  _spi_write_address(address | W_REGISTER, data);
  CS_on;
  return 1;
}

uint8_t NRF24L01_FlushTx()
{
  return Strobe(FLUSH_TX);
}

uint8_t NRF24L01_FlushRx()
{
  return Strobe(FLUSH_RX);
}

static uint8_t Strobe(uint8_t state)
{
  uint8_t result;
  CS_off;
  result = _spi_write(state);
  CS_on;
  return result;
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
  CS_off;
  uint8_t data = _spi_read_address(reg);
  CS_on;
  return data;
}

void NRF24L01_SetTxRxMode(uint8_t mode)
{
  if (mode == TX_EN) {
    CE_off;
    NRF24L01_WriteReg(NRF24L01_07_STATUS,
                      (1 << NRF24L01_07_RX_DR)    // reset the flag(s)
                      | (1 << NRF24L01_07_TX_DS)
                      | (1 << NRF24L01_07_MAX_RT));
    NRF24L01_WriteReg(NRF24L01_00_CONFIG,
                      (1 << NRF24L01_00_EN_CRC)   // switch to TX mode
                      | (1 << NRF24L01_00_CRCO)
                      | (1 << NRF24L01_00_PWR_UP));
    delayMicroseconds(130);
    CE_on;
  } else if (mode == RX_EN) {
    CE_off;
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);        // reset the flag(s)
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);        // switch to RX mode
    NRF24L01_WriteReg(NRF24L01_07_STATUS,
                      (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                      | (1 << NRF24L01_07_TX_DS)
                      | (1 << NRF24L01_07_MAX_RT));
    NRF24L01_WriteReg(NRF24L01_00_CONFIG,
                      (1 << NRF24L01_00_EN_CRC)   // switch to RX mode
                      | (1 << NRF24L01_00_CRCO)
                      | (1 << NRF24L01_00_PWR_UP)
                      | (1 << NRF24L01_00_PRIM_RX));
    delayMicroseconds(130);
    CE_on;
  } else {
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)); // PowerDown
    CE_off;
  }
}

uint8_t NRF24L01_Reset()
{
  NRF24L01_FlushTx();
  NRF24L01_FlushRx();
  uint8_t status1 = Strobe(0xFF); // NOP
  uint8_t status2 = NRF24L01_ReadReg(0x07);
  NRF24L01_SetTxRxMode(TXRX_OFF);
  return (status1 == status2 && (status1 & 0x0f) == 0x0e);
}
