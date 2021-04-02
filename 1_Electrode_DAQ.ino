#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Defining pins from arduino nano to DDS AD9850. 
// Defining is constant. 
#define PIN_DDS_RESET 11
#define PIN_DDS_DATA 10
#define PIN_DDS_FU_UD 9
#define PIN_DDS_W_CLK 8

#define PIN_LED LED_BUILTIN
#define DELAY_US 500
#define DDS_CLOCK 125000000
#define BACKLIGHT_PIN     13

/* ADC Setup */
#define analog0 A0 // Pin A0

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

/* DECLARATIONS */
/* Void is a "free for all" datatype. Can store integers, decimals, words, etc. */ 
// Passing info. / data by value from one point to another. 
// There's an address to each, when program hits this point, it will jump to the function address and fill it with information. 
void parse_cmd(void);
void parse_cmd_custom(void);
void send(unsigned long word);
void clock_tick();      
void reset_tick();      
void freq_update_tick();
unsigned long word_compute(unsigned long frequency); // unsigned long brings 8 bytes. Where it's called, 
void send_bit(int value);


/* GLOBALS - can be called and used from anywhere, aka a constant*/
int incoming_byte;
char cmd[128];      //Array.. Allocates 128 bytes to CMD... 1 Char = 1 byte, 1 byte = 8 bits... 128*8 bits allocated. 
char frequency[128]; //Array.. Allocates 128 bytes to frequency 
int i = 0;

/* ADC Setup */
int val0 = 0; // Variable to store the value read from conditioning circuit 

/* DAQ Setup */
String dataLabel1 = "Electrode 1";
bool label = true;

void setup() {

// set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
// Print a message to the LCD.
  lcd.clear();

// Setting AREF for reference voltage of 3.3V
  analogReference(EXTERNAL); 
    
//Sets Arduino pins as O/P
    pinMode(PIN_DDS_RESET, OUTPUT);
    pinMode(PIN_DDS_DATA, OUTPUT);
    pinMode(PIN_DDS_FU_UD, OUTPUT);
    pinMode(PIN_DDS_W_CLK, OUTPUT);
    pinMode(PIN_LED, OUTPUT);

//Sets Arduino pins to Low / Off / 0. 
    digitalWrite(PIN_DDS_RESET, LOW);
    digitalWrite(PIN_DDS_DATA, LOW);
    digitalWrite(PIN_DDS_FU_UD, LOW);
    digitalWrite(PIN_DDS_W_CLK, LOW);
    digitalWrite(PIN_LED, LOW);
    
    Serial.begin(115200, SERIAL_8N1);
    delay(10);
    //Serial.println("Program Starting");
    // Clears bits in array to 0, "Fresh start" 
    memset(cmd, '\0', sizeof(cmd));
    memset(frequency, '\0', sizeof(frequency));


    //Serial.write("DDS reset... ");
    reset_tick();
   // Serial.write(" - Completed!\r\n");
    delay(1000);
   // Serial.write("DDS reset... ");
    reset_tick();
    //Serial.write(" - Completed!\r\n");
    send(0);
    delay(1000);
    
    /* Setting Frequency...Unsigned long bc we want to store large value, and non-negative values. */
    unsigned long freq_word = 12500;       
    unsigned long word = word_compute(freq_word); // word_compute goes to its function, does calc. then saves that into word. 
    
    send(word); // Goes to function send, bring the data in word (100kHz) to send function, 
    Serial.println("\n");
    delay(1000);

}

void loop() 
{

  /* DAQ Setup */
  while(label){ //runs once
    //enable headers
    Serial.print(dataLabel1);
    label = false;
    
   }

  /* Reading Analog Input Note: using AREF via 3.3V jumper */
  val0 = analogRead(analog0);
   
    // Data in CSV format
    Serial.println("\t");
    Serial.print("   ");
    Serial.print(val0);

    /* Converting Serial # to voltage */
    float voltage0 = 0.0;
    voltage0 = (val0*(3.3/1024.0));
    
    // LCD Display
    lcd.setCursor(0,0);
    lcd.print("V1:");
    lcd.print(voltage0, 3);

    delay(100); 
  
}

void send(unsigned long word) {
    for(int i = 0; i < 32; i++) {   // Parsing bit by bit through "word"
        send_bit(word & 0b1);   // Parses through word, read 1 bit, sends it to send_bit, then reads next bit. 
        word >>= 1;
    }

    // Control bits
    send_bit(0);
    send_bit(0);

    // Power-down
    send_bit(0);

    // Phase
    send_bit(0);
    send_bit(0);
    send_bit(0);
    send_bit(0);
    send_bit(0);
    
    freq_update_tick();
}

void send_bit(int value) {
    if(value)
        digitalWrite(PIN_DDS_DATA, HIGH);
    else
        digitalWrite(PIN_DDS_DATA, LOW);

    clock_tick();
}

void clock_tick() {
    delayMicroseconds(DELAY_US);
    digitalWrite(PIN_DDS_W_CLK, HIGH);
    delayMicroseconds(DELAY_US);
    digitalWrite(PIN_DDS_W_CLK, LOW);
}
void reset_tick() {
    digitalWrite(PIN_DDS_RESET, HIGH);
    delayMicroseconds(DELAY_US);
    digitalWrite(PIN_DDS_RESET, LOW);
    delayMicroseconds(DELAY_US);
}

void freq_update_tick() {
    digitalWrite(PIN_DDS_FU_UD, HIGH);
    delayMicroseconds(DELAY_US);
    delayMicroseconds(DELAY_US);
    digitalWrite(PIN_DDS_FU_UD, LOW);
}

unsigned long word_compute(unsigned long frequency) {
    return (4294967296 * frequency) / DDS_CLOCK;
}
