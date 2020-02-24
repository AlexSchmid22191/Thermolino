#include <Arduino.h>
#include "SPI.h"
#include <LiquidCrystal.h>
#include <Nanoshield_Termopar.h>

#define MINIMAL true

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
Nanoshield_Termopar tc(8, TC_TYPE_K, TC_AVG_16_SAMPLES);


//Measured variables
float temp;
float voltage;

//For button debouncing and display updating
long int button_last_pressed;
long int last_display_update;

//TC type and ADC channel
byte tc_type = 0;
byte adc_chan = 0;

//Pin definitions
const byte tc_switch_button = 9;
const byte adc_switch_button = 10;

const byte termopar_cs = 8;

const byte adc_cs = A0;

const byte lcd_rs = 2;
const byte lcd_en = 3;
const byte lcd_d1 = 4;
const byte lcd_d2 = 4;
const byte lcd_d3 = 5;
const byte lcd_d4 = 6;


//Thermoelement types and display strings
const TcType type[8] =
        {
                TC_TYPE_K,
                TC_TYPE_S,
                TC_TYPE_B,
                TC_TYPE_J,
                TC_TYPE_N,
                TC_TYPE_R,
                TC_TYPE_T,
                TC_TYPE_E
        };

const char Type_String[] =
        {
                'K',
                'S',
                'B',
                'J',
                'N',
                'R',
                'T',
                'E'
        };

//Control sequence for the ADC, 2 channel differential signed
const byte adc_controlByte[2] =
        {
                0b01100000,
                0b00110000
        };

//For serial communication
char mode = 'U';


void read_adc();

void toggle_tc_type();

void update_lcd();
void print_temperature();
void print_errors();
void print_adc();

void listen_to_serial();



void setup()
{
    //LCD sceeen
    pinMode(lcd_rs, OUTPUT);
    pinMode(lcd_en, OUTPUT);
    pinMode(lcd_d1, OUTPUT);
    pinMode(lcd_d2, OUTPUT);
    pinMode(lcd_d3, OUTPUT);
    pinMode(lcd_d4, OUTPUT);

    //Termopar Shield
    pinMode(termopar_cs, OUTPUT);

    //ADC Chip
    pinMode(adc_cs, OUTPUT);
    digitalWrite(adc_cs, HIGH);

    //Pushbuttons
    pinMode(adc_switch_button, INPUT_PULLUP);
    pinMode(tc_switch_button, INPUT_PULLUP);


    Serial.begin(9600);
    lcd.begin(16, 2);
    tc.begin();
    SPI.begin();

    lcd.print("Thermolino 2.0");
    lcd.setCursor(0,1);
    lcd.print("Starting...");
    delay(1000);
    lcd.clear();

    //Initialize timing variables
    button_last_pressed = millis();
    last_display_update = millis();
}

void loop()
{
    //Measure variables
    tc.read();
    read_adc();

    //Exponential moving average filter to smooth temperature
    temp = temp*0.5 + tc.getExternal()*0.5;

    //Update LCD display every second
    if((millis() - last_display_update) > 1000)
    {
        last_display_update = millis();
        update_lcd();
    }

    //Check if thermocouple button was pressed
    if(digitalRead(tc_switch_button) == LOW && (millis() - button_last_pressed) > 300)
    {
        button_last_pressed = millis();
        toggle_tc_type();
    }

    //Check if AC button was pressed
    if(digitalRead(adc_switch_button) == LOW && (millis() - button_last_pressed) > 300)
    {
        button_last_pressed = millis();
        adc_chan += 1;
        if(adc_chan==2)
        {
            adc_chan=0;
        }
    }

    //Check for serial communication
    listen_to_serial();
}

void read_adc()
{
    byte hiByte = 0;  //Storage for the first byte from the ADC
    byte loByte = 0;  //Storage for the second byte from the ADC
    int result = 0;   //Result

    //Begin SPI transaction
    SPI.beginTransaction(SPISettings(uint32_t(14000000), MSBFIRST, SPI_MODE0));

    //Take the chip select low to select the device
    digitalWrite(adc_cs, LOW);
    delay(1);

    //Send request and receive first byte
    hiByte = SPI.transfer(adc_controlByte[adc_chan]);
    //Send empty byte and fetch second byte
    loByte = SPI.transfer(0x00);

    //Shift high byte and combine hi and lo bytes
    result = (hiByte<<8)|loByte;

    //Deselect slave
    digitalWrite(adc_cs, HIGH);
    SPI.endTransaction();

    //Calculate actual voltage and apply exponential moving average filter
    voltage = (2048.0*result/32768)*0.001 + voltage*0.999;
}

void toggle_tc_type()
{
    //Select new TC type and begin new instance
    tc_type++;

    if(tc_type > 7)
    {
        tc_type = 0;
    }

    tc = Nanoshield_Termopar(termopar_cs, type[tc_type], TC_AVG_16_SAMPLES);
    tc.begin();
}

void print_temperature()
{
    char tc_string_lcd[17];
    char temp_string[9];

    dtostrf(temp, 8, 2, temp_string);
    sprintf(tc_string_lcd, "TC: %c %s%cC", Type_String[tc_type] ,temp_string, 223);

    lcd.setCursor(0, 0);
    lcd.print(tc_string_lcd);
}

void print_errors()
{
    lcd.setCursor(0,0);
    if(tc.isOpen())
    {
        lcd.print("Open circuit");
    }
    else if(tc.isOverUnderVoltage())
    {
        lcd.print("OverUnderVoltage");
    }
    else if(tc.isInternalOutOfRange())
    {
        lcd.print("Internal T OOR");
    }
    else if(tc.isExternalOutOfRange())
    {
        lcd.print("External T OOR");
    }
}


void print_adc()
{
    char adc_string_lcd[17];
    char voltage_string[8];

    dtostrf(voltage, 7, 1, voltage_string);
    sprintf(adc_string_lcd, "AD: %i %s mV", adc_chan+1, voltage_string);

    lcd.setCursor(0, 1);
    lcd.print(adc_string_lcd);
}

void update_lcd()
{
    lcd.clear();
    if(tc.hasError())
    {
        print_errors();
    }
    else
    {
        print_temperature();
    }
    #if MINIMAL == false
        print_adc();
    #endif
}

void listen_to_serial()
{
    char commandBuffer[51];
    char *commandPointers[2];


    //Check if a serial communication is requested
    while((bool)Serial.available())
    {
        //Read bytes until the start character (:) is encounterd
        int x = Serial.read();
        if(x == ':')
        {
            //Read up to 50 bytes into the command buffer until a line feed is encountered
            memset(commandBuffer, 0, 50);
            Serial.readBytesUntil(0x0A, commandBuffer, 50);

            //Slice the command string at every delimiter (blank space) and store pointers in command pointers
            commandPointers[0] = strtok(commandBuffer, " ");
            byte i = 0;
            while(commandPointers[i] != nullptr)
            {
                i++;
                commandPointers[i] = strtok(nullptr, ",");
            }

            //Check if a function set is requested
            if(!(bool)strncmp(commandPointers[0], "FUNC", 4))
            {
                if(!(bool)strncmp(commandPointers[1], "'VOLT'", 6))
                {
                    mode = 'U';
                }
                if(!(bool)strncmp(commandPointers[1], "'TEMP'", 6))
                {
                    mode = 'T';
                }
            }

            //Check if a data read is requested
            if(!(bool)strncmp(commandPointers[0], "read?", 5))
            {
                if(mode == 'T')
                {
                    if(!tc.hasError())
                    {
                        char printString[15];
                        dtostre(temp, printString, 7, DTOSTR_UPPERCASE | DTOSTR_PLUS_SIGN);
                        Serial.println(printString);
                    }
                }

                if(mode == 'U')
                {
                    char printString[15];
                    dtostre(voltage/1000.0, printString, 7, DTOSTR_UPPERCASE | DTOSTR_PLUS_SIGN);
                    Serial.println(printString);
                }
            }
        }
    }
}
