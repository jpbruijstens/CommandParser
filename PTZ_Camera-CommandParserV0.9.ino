#include "CommandParser.h"
#include <Wire.h>
#include <TimerOne.h>
#include <string.h>
#include <math.h>
#include <ezButton.h>

#include <SPI.h>
#include <Ethernet.h>

#define SW_REV "PTZ_camV0.9"
#define HW_REV "N/A"
#define BATCH_NR "N/A"

#define HOMING_XL 250                  // tmp values -> need adjustments
#define HOMING_XR 500                  // tmp values -> need adjustments
#define HOMING_YU 250                  // tmp values -> need adjustments
#define HOMING_YD 500                  // tmp values -> need adjustments

const int pulsePinX             = 5;    // Pin 5 generate the pulse train 
const int dirPinX               = 6;    // Pin 6 is Direction
// const int posSensorX            = 4;    // Pin 7 is for Homing -> Blue cable from microswitch, Black cable goes to GND
ezButton switchX(8);                // create ezButton object that attach to pin 7;

const int pulsePinY             = 9;    // Pin 8 generate the pulse train 
const int dirPinY               = 7;    // Pin 9 is Direction
// const int posSensorY            = 7;   // Pin 10 is for Homing
ezButton switchY(3);               // create ezButton object that attach to pin 11;

const uint8_t bitmask5 = 1 << 5;        // bitmask for digital pin 5
const uint8_t bitmask8 = 1 << 1;        // bitmask for digital pin 8

// Initialize the timer count and pin state variables
volatile uint16_t timerCountX   = 0;    //Counter for interrupt 
volatile uint16_t timerCountY   = 0;    //Counter for interrupt 
volatile uint16_t pulseCountX   = 0;    // Counter for amount of pulses past
volatile uint16_t pulseCountY   = 0;    // Counter for amount of pulses past
volatile int16_t numCyclesX     = 0;    // Total amount of pulses need to be sent
volatile int16_t numCyclesY     = 0;    // Total amount of pulses need to be sent
volatile uint16_t varDelayX     = 0;    // Variable delay for pulse train
volatile uint16_t varDelayY     = 0;    // Variable delay for pulse train
volatile uint16_t iterateX      = 0;    // i for calculating varDelay
volatile uint16_t iterateY      = 0;    // i for calculating varDelay
volatile int16_t speedX         = 0;    // speed percentage
volatile int16_t speedY         = 0;    // speed percentage


const int arrayLength   = 36;           // (maxDelay - minDelay) / 50
volatile int16_t array[arrayLength];    // lookup table for varDelay
#define maxValue 2000   // max delay between pulses -> 20 mili seconds
#define minValue 250      // min delay between pulses -> 2.5 mili seconds

int uartrx;

unsigned long StartTime = millis();

// sersors
bool homeXL                     = 0;
bool homeXR                     = 0;
bool homeYU                     = 0;
bool homeYD                     = 0;
bool reports                    = false;
volatile bool homingDone        = false;       // Homing completed?

Stream *console = &Serial;

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

IPAddress ip(192, 168, 1, 177);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

// Http data
String reqData; // Request from Smartphone
String header;
int contentSize = -1;
String CONTENT_LENGTH_TXT = "Content-Length: ";

void setup() 
{    
    Wire.begin();
    setupSerial();
    pinSetup();
    Timer1.initialize(10);
    Timer1.attachInterrupt(my_ISR);

    Ethernet.init(10);  // Most Arduino shields
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Ethernet WebServer Example");
    // start the Ethernet connection and the server:
    Ethernet.begin(mac, ip);

    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        while (true) {
            delay(1); // do nothing, no point running without Ethernet hardware
        }
    }
    if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
    }

    // start the server
    server.begin();
    Serial.print("server is at ");
    Serial.println(Ethernet.localIP());


    switchX.setDebounceTime(50); // set debounce time to 50 milliseconds
    switchY.setDebounceTime(50); // set debounce time to 50 milliseconds 

    //homing(NULL, NULL, NULL);
    if (homeXL && homeXR && homeYU && homeYD){
        homingDone = true;
    }
    
    array[0] = maxValue;

    for (int i = 1; i < arrayLength; i++) {
        array[i] = array[i-1] - 50; // calculate ramping before
    }

    delay(500); 
    Serial.println("Setup Done!");
}

void loop()
{
    switchX.loop(); // MUST call the loop() function first
    switchX.loop(); // MUST call the loop() function first
    processUSB();

    unsigned long CurrentTime = millis();
    unsigned long ElapsedTime = CurrentTime - StartTime;
    EthernetClient client = server.available(); // Is there a client (Our Android smartphone)
    
    if (client) {
        // Let's start reading
        boolean isLastLine = true;
        boolean isBody = false;
        header = "";
        reqData = "";
        int contentLen = 0;
        
        Serial.print("Client connected!");
        while (client.connected()) {
                if (client.available()) {
                // Read data
                char c = client.read();
                
                    Serial.print(c);
                    
                if (contentSize == contentLen) {
                // Serial.println("Body ["+reqData+"]");
                    
                    int idx = reqData.indexOf(":");
                    String status = reqData.substring(idx + 1, idx + 2);
                    Serial.println("Status : " + status);
                    if (status.equals("1")) {
                        client.println("reqData = 1");
                        client.println(reqData);
                    }
                    else {
                        client.println("reqData = !1");
                        client.println(reqData);
                    }
                    client.println(status);
                    
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: text/html");
                    client.println("Connection: close");
                    client.println();
                    // send web page
                    client.println("");
                    client.println("");
                    delay(1);
                    break;
                }
                
                
                if (c == '\n' && isLastLine) {
                    isBody = true;
                    int pos = header.indexOf(CONTENT_LENGTH_TXT);
                    String tmp = header.substring(pos, header.length());
                    //Serial.println("Tmp ["+tmp+"]");
                    int pos1 = tmp.indexOf("\r\n");
                    String size = tmp.substring(CONTENT_LENGTH_TXT.length(), pos1);
                    Serial.println("Size ["+size+"]");
                    contentSize = size.toInt();
                    
                }
                
                if (isBody) {
                    reqData += c;
                    contentLen++;
                }
                else {
                header += c;
                }
                
                
                if (c == '\n' ) {
                isLastLine = true;
                }
                else if (c != '\r' ) {
                    isLastLine = false;
                }
                
                
                
                }
        }
        
        // Close connection
        Serial.println("Stop..");
        client.stop();
    }

    if ( reports ) {
        noInterrupts();
    } 
}


const CommandParser::cmd_t commands[] = 
{
    { "info_rev", cmd_info_rev },
    { "homing", homing },
    { "send", sent_stream },
    { "help", cmd_help },
    { NULL, NULL }
};
CommandParser cp ( commands );

void setupSerial()
{
    Serial.begin ( 9600 );
}

void pinSetup() 
{
    pinMode(dirPinX, OUTPUT);
    pinMode(pulsePinX, OUTPUT);
    // pinMode(posSensorX, INPUT);

    pinMode(dirPinY, OUTPUT);
    pinMode(pulsePinY, OUTPUT);
    // pinMode(posSensorY, INPUT);
}

void my_ISR(void)
{  
     if (timerCountX == 1 && pulseCountX + 1){
        PORTD &= ~bitmask5;
        varDelayX = ramping(iterateX, numCyclesX, speedX); // calculate next delay
        iterateX++;
    } else if (timerCountX == varDelayX && pulseCountX){
        PORTD |= bitmask5;
        timerCountX = 0;     // end of pulse so reset
        pulseCountX--;       // did we sent all pulses yet, if not, minus 1
    }


    if (timerCountY == 1 && pulseCountY + 1){
        PORTB &= ~bitmask8;
        varDelayY = ramping(iterateY, numCyclesY, speedY); // calculate next delay
        iterateY++;
    } else if (timerCountY == varDelayY && pulseCountY){
        PORTB |= bitmask8;
        timerCountY = 0;     // end of pulse so reset
        pulseCountY--;       // did we sent all pulses yet, if not, minus 1
    }
    timerCountX++;           // How many times did the interrupt occur for both motors?
    timerCountY++;           
}

void resetStream()
{
    timerCountX = 0;
    timerCountY = 0;
    pulseCountX = 0;
    pulseCountY = 0;
}

void processUSB()
{
    if ( Serial.available() ) { // MEUK
        ++uartrx;
        console = &Serial;
        cp.process(console);
    }
}

void cmd_info_rev ( char* arg1, char* arg2, char* arg3 ) 
{
    console->println("SW_REV = " SW_REV);
    console->println("HW_REV = " HW_REV);
    console->println("BATCH = " BATCH_NR);
}

void cmd_help ( char* arg1, char* arg2, char* arg3 ) 
{
    Serial.println("Syntax for sending is: TotalPulsesforX,SpeedForMotorX TotalPulsesforY,SpeedForMotorY");
    Serial.println("Example: send 250,50 125,25");
    Serial.println();
    cp.listCommands(console);
}

void sent_stream ( char* arg1, char* arg2, char* arg3 ) 
{
//    if ( homingDone ){
    // Generate pulse train
    noInterrupts();
    resetStream(); 

    // parsing for arg1
    char *pt1;
    pt1 = strtok (arg1,","); // get pulse amount
    numCyclesX = atoi(pt1);
    pt1 = strtok (NULL, ",");// get speed
    speedX = atoi(pt1);
    speedX = map(speedX, 1, 100, maxValue, minValue);


    // parsing for arg2
    char *pt2;
    pt2 = strtok (arg2,","); // get pulse amount
    numCyclesY = atoi(pt2);
    pt2 = strtok (NULL, ",");// get speed
    speedY = atoi(pt2);
    speedY = map(speedY, 1, 100, maxValue, minValue);


    // check positive or negative for directions.
    if ( numCyclesX < 0 ){
        digitalWrite(dirPinX, LOW);
    } else {
        digitalWrite(dirPinX, HIGH);
    }

    if (numCyclesY < 0 ){
        digitalWrite(dirPinY, LOW);
    } else {
        digitalWrite(dirPinY, HIGH);
    }

    numCyclesX = abs(numCyclesX);
    numCyclesY = abs(numCyclesY);
    pulseCountX = numCyclesX;
    pulseCountY = numCyclesY;

    iterateX = 0;
    iterateY = 0;

    delayMicroseconds(50); //needs time, see Datasheet
    interrupts();
}

int ramping (int i, int pulsesTotal, int minSpeed)  // send current cycle, total amount of pulses, minimal delay
{  
    int acc = 0;

        if ( pulsesTotal > 2 * (arrayLength -1) ) {     // check if the amount of pulses is above twice the amount ramp pulses ( ramping up + ramping down)
            if ( i < arrayLength ) {                    // ramp up
                acc = array[i];
                if ( acc < minSpeed) {
                    acc = minSpeed;
                }
            } else if ( i >= arrayLength && i < pulsesTotal - arrayLength) { // stay at minimal delay
                acc = minSpeed;
            }
        
            else {                                      // ramp up
                acc = array[abs(i - pulsesTotal)];
                if ( acc < minSpeed) {
                    acc = minSpeed;
                }        
            }
        } 
        
        else {                                          // if below twice the amount of ramp pulses
            if ( i < arrayLength ) {
                acc = array[i];                         // ramp up
            }
            else {
                acc = array[abs(i - pulsesTotal)];      // ramp down
            }
        }  

    return acc; 
}

// Homing for both motors -> no sensors available for now!! 
void homing ( char* arg1, char* arg2, char* arg3 ) 
{
    int i = 0;
    digitalWrite(dirPinX, LOW);
    while (switchX.getState() && i < HOMING_XL ) {
        PORTD |= bitmask5;        //binary pin 5 high/low, others low.
        delayMicroseconds(11);
        PORTD &= ~bitmask5;        //binary pin 5 high/low, others low.
        delay(ramping(i, HOMING_XL, 20));
        homeXL = 1;
        i++;
    }
    i = 0;
    digitalWrite(dirPinX, HIGH);
    while (switchX.getState() && i < HOMING_XR ) {
        PORTD |= bitmask5;        //binary pin 5 high/low, others low.
        delayMicroseconds(11);
        PORTD &= ~bitmask5;        //binary pin 5 high/low, others low.
        delay(ramping(i, HOMING_XR, 20));
        homeXR = 1;
        i++;
    }

    i = 0;
    digitalWrite(dirPinY, LOW);
    while (switchY.getState() && i < HOMING_YU ) {
        PORTB |= bitmask8;        //binary pin 8 high/low, others low.
        delayMicroseconds(11);
        PORTB &= ~bitmask8;        //binary pin 8 high/low, others low.
        delay(ramping(i, HOMING_YU, 20));
        homeYU = 1;
        i++;
    }

    i = 0;
    digitalWrite(dirPinY, HIGH);
    while (switchY.getState() && i < HOMING_YD ) {
        PORTB |= bitmask8;        //binary pin 8 high/low, others low.
        delayMicroseconds(11);
        PORTB &= ~bitmask8;        //binary pin 8 high/low, others low.
        delay(ramping(i, HOMING_YD, 20));
        homeYD = 1;
        i++;
    }
}
