
// ST_CP = SCK
// SH_CP = RCK
// SDI   = DIO
// Common anode
#define DS 10
#define STCP 11
#define SHCP 12
#define SPEED 500
boolean numbersDef[10][8] =
{
  {1,1,1,1,1,1,0}, //zero
  {0,1,1,0,0,0,0}, //one
  {1,1,0,1,1,0,1}, //two
  {1,1,1,1,0,0,1}, //three
  {0,1,1,0,0,1,1}, //four
  {1,0,1,1,0,1,1}, //five
  {1,0,1,1,1,1,1}, //six
  {1,1,1,0,0,0,0}, //seven
  {1,1,1,1,1,1,1}, //eight
  {1,1,1,1,0,1,1}  //nine
};

boolean digitsTable[8][8] =
{
  {0,0,0,0,1,0,0,0}, // first digit
  {0,0,0,0,0,1,0,0}, // second
  {0,0,0,0,0,0,1,0}, // third
  {1,0,0,0,0,0,0,0}, // forth
  {0,1,0,0,0,0,0,0}, // fifth
  {0,0,1,0,0,0,0,0}  // sixth  
};

void setup() {
  pinMode(DS, OUTPUT);
  pinMode(STCP, OUTPUT);
  pinMode(SHCP, OUTPUT);
  digitalWrite(DS, LOW);
  digitalWrite(STCP, LOW);
  digitalWrite(SHCP, LOW);
}

boolean display_buffer[16];
void prepareDisplayBuffer(int number, int digit_order, boolean showDot)
{
  for(int index=7; index>=0; index--)
  {
    display_buffer[index] = digitsTable[digit_order-1][index];
  }
  for(int index=14; index>=8; index--)
  {
    display_buffer[index] = !numbersDef[number-1][index]; //because logic is sanity, right?
  }
  if(showDot == true)
    display_buffer[15] = 0;
  else
    display_buffer[15] = 1;
}

void writeDigit(int number, int order, bool showDot = false)
{
  prepareDisplayBuffer(number, order, showDot);
  digitalWrite(SHCP, LOW);
  for(int i=15; i>=0; i--)
  {
      digitalWrite(STCP, LOW);
      digitalWrite(DS, display_buffer[i]); //output LOW - enable segments, HIGH - disable segments
      digitalWrite(STCP, HIGH);
   }
  digitalWrite(SHCP, HIGH);
}

void loop() {
  writeDigit(0, 1);
  writeDigit(2, 2, true);
  writeDigit(3, 3);
  writeDigit(4, 4, true);
  writeDigit(5, 5);
  writeDigit(6, 6);
}