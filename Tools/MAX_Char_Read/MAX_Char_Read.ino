// Code to copy a MCM font file to the Arduino + Max7456 OSD
//
// MAX7456_font Sketch
// at 9600 baud it take about 3min to download a mcm file
// http://www.maxim-ic.com/tools/evkit/index.cfm?EVKit=558
// max7456 evaluation kit software

#define DATAOUT 11//11-MOSI
#define DATAIN  12//12-MISO
#define SPICLOCK  13//13-sck
#define MAX7456SELECT 6 //6
#define USBSELECT 10//10-ss
#define VSYNC 2// INT0

//MAX7456 opcodes
#define VM0_reg   0x00
#define VM1_reg   0x01
#define DMM_reg   0x04
#define DMAH_reg  0x05
#define DMAL_reg  0x06
#define DMDI_reg  0x07
#define CMM_reg   0x08
#define CMAH_reg  0x09
#define CMAL_reg  0x0A
#define CMDI_reg  0x0B
#define CMDO_reg  0x0C
#define STAT_reg  0xA0

//MAX7456 commands
#define CLEAR_display 0x04
#define CLEAR_display_vert 0x06
#define END_string 0xff
#define WRITE_nvr 0xa0
#define READ_nvr 0x50

// with NTSC
#define ENABLE_display 0x08
#define ENABLE_display_vert 0x0c
#define MAX7456_reset 0x02
#define DISABLE_display 0x00

// with PAL
// all VM0_reg commands need bit 6 set
//#define ENABLE_display 0x48
//#define ENABLE_display_vert 0x4c
//#define MAX7456_reset 0x42
//#define DISABLE_display 0x40

#define WHITE_level_80 0x03
#define WHITE_level_90 0x02
#define WHITE_level_100 0x01
#define WHITE_level_120 0x00

#define MAX_font_rom 0xff
#define STATUS_reg_nvr_busy 0x20
#define NVM_ram_size 0x36

// with NTSC
#define MAX_screen_rows 0x0d //13

// with PAL
//#define MAX_screen_rows 0x10 //16

volatile byte screen_buffer[MAX_font_rom];
volatile byte character_bitmap[0x40];
volatile byte ascii_binary[0x08];

volatile int font_count;
volatile int  count;

String inputChars;

//////////////////////////////////////////////////////////////
void setup()
{
  byte spi_junk;
  int x;
  Serial.begin(38400);
  Serial.flush();

  digitalWrite(USBSELECT,HIGH); //disable USB chip

  pinMode(MAX7456SELECT,OUTPUT);
  digitalWrite(MAX7456SELECT,HIGH); //disable device

  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK,OUTPUT);
  pinMode(VSYNC, INPUT);

  // SPCR = 01010000
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //sample on leading edge of clk,system clock/4 rate (4 meg)
  SPCR = (1<<SPE)|(1<<MSTR);
  spi_junk=SPSR;
  spi_junk=SPDR;
  delay(250);

  // force soft reset on Max7456
  digitalWrite(MAX7456SELECT,LOW);
  spi_transfer(VM0_reg);
  spi_transfer(MAX7456_reset);
  digitalWrite(MAX7456SELECT,HIGH);
  delay(500);

  // set all rows to same character white level, 90%
  digitalWrite(MAX7456SELECT,LOW);
  for (x = 0; x < MAX_screen_rows; x++)
  {
    spi_transfer(x + 0x10);
    spi_transfer(WHITE_level_90);
  }

  // make sure the Max7456 is enabled
  spi_transfer(VM0_reg);
  spi_transfer(ENABLE_display);
  digitalWrite(MAX7456SELECT,HIGH);

  count = 0;
  font_count = 0;

  //display all 256 internal MAX7456 characters
  for (x = 0; x < MAX_font_rom; x++)
  {
    screen_buffer[x] = x;
  }
   count = MAX_font_rom;
   write_new_screen();

  inputChars = "";

  Serial.println("Ready for text file download");
  Serial.println("");
  delay(100);
}

void processInput()
{
char intBuffer[12];
int i;
byte x;
int intLength = inputChars.length() + 1;
inputChars.toCharArray(intBuffer, intLength);

if (inputChars == "MAX7456")
{
  for(i=0;i<256;i++)
    {
      font_count = i;
      for(x = 0; x < 64; x++) character_bitmap[x] = 0;
      read_NVM();
      for(x = 0; x < 64; x++) Serial.println(character_bitmap[x], BIN);
    }
inputChars = "";
return;
}


inputChars = "";
i = atoi(intBuffer);
if (i<0 || i>255) {
    Serial.println("Input should be between 0 and 255. You entered: "+i);
    return;
    }

font_count = i;
for(x = 0; x < 64; x++) character_bitmap[x] = 0;
read_NVM();

for(x = 0; x < 64; x++) Serial.println(character_bitmap[x], BIN);
Serial.println("");
}


//////////////////////////////////////////////////////////////
void loop()
{
 while (Serial.available()) {
        int ch = Serial.read();
        if (ch == -1) {
            break;
        }
        else if (ch == (int)'\n' || ch == (int)'\r') {
            processInput();
            break;
        }
        else {
            inputChars += (char) ch;
        }
    }
}

//////////////////////////////////////////////////////////////
byte spi_transfer(volatile byte data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
  {
  };
  return SPDR;                    // return the received byte
}

//////////////////////////////////////////////////////////////
void write_new_screen()
{
  int x, local_count;
  byte char_address_hi, char_address_lo;
  byte screen_char;

  local_count = count;

  char_address_hi = 0;
  char_address_lo = 60; // start on third line
 //Serial.println("write_new_screen");

  // clear the screen
  digitalWrite(MAX7456SELECT,LOW);
  spi_transfer(DMM_reg);
  spi_transfer(CLEAR_display);
  digitalWrite(MAX7456SELECT,HIGH);

  // disable display
  digitalWrite(MAX7456SELECT,LOW);
  spi_transfer(VM0_reg);
  spi_transfer(DISABLE_display);

  spi_transfer(DMM_reg); //dmm
  //spi_transfer(0x21); //16 bit trans background
  spi_transfer(0x01); //16 bit trans w/o background

  spi_transfer(DMAH_reg); // set start address high
  spi_transfer(char_address_hi);

  spi_transfer(DMAL_reg); // set start address low
  spi_transfer(char_address_lo);

  x = 0;
  while(local_count) // write out full screen
  {
    screen_char = screen_buffer[x];
    spi_transfer(DMDI_reg);
    spi_transfer(screen_char);
    x++;
    local_count--;
  }

  spi_transfer(DMDI_reg);
  spi_transfer(END_string);

  spi_transfer(VM0_reg); // turn on screen next vertical
  spi_transfer(ENABLE_display_vert);
  digitalWrite(MAX7456SELECT,HIGH);
}

//////////////////////////////////////////////////////////////
void read_NVM()
{
  byte x;
  byte char_address_hi, char_address_lo;
  byte screen_char;

  char_address_hi = font_count;
  char_address_lo = 0;
 //Serial.println("read_new_screen");

  // disable display
  digitalWrite(MAX7456SELECT,LOW);
  spi_transfer(VM0_reg);
  spi_transfer(DISABLE_display);

/*
1) Write VM0[3] = 0 to disable the OSD image.
2) Write CMAH[7:0] = xxH to select the character
(0–255) to be read (Figures 10 and 13).
3) Write CMM[7:0] = 0101xxxx to read the character
data from the NVM to the shadow RAM (Figure 13).

4) Write CMAL[7:0] = xxH to select the 4-pixel byte
(0–63) in the character to be read (Figures 10 and 13).
5) Read CMDO[7:0] = xxH to read the selected 4-pixel
byte of data (Figures 11 and 13).
6) Repeat steps 4 and 5 to read other bytes of 4-pixel
data.
7) Write VM0[3] = 1 to enable the OSD image display.
*/

  spi_transfer(CMAH_reg); // set start address high
  spi_transfer(char_address_hi);

  // transfer 54 bytes from NVM to shadow ram
  spi_transfer(CMM_reg);
  spi_transfer(READ_nvr);

  for(x = 0; x < NVM_ram_size; x++) // read 54 (out of 64) bytes of character from shadow ram
  {
    spi_transfer(CMAL_reg); // set start address low
    spi_transfer(x);

    character_bitmap[x] = spi_transfer(CMDO_reg);
  }


  // wait until bit 5 in the status register returns to 0 (12ms)
  while ((spi_transfer(STAT_reg) & STATUS_reg_nvr_busy) != 0x00);

  spi_transfer(VM0_reg); // turn on screen next vertical
  spi_transfer(ENABLE_display_vert);
  digitalWrite(MAX7456SELECT,HIGH);
}
