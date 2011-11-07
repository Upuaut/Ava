/*
 AVA Flight Computer V0.115
 --------------------------

 Created 2011
 by Upu/2E0UPU http://ava.upuaut.net
 RTTY code from Rob Harrison Icarus Project.

 */

#include <ctype.h>
#include <string.h>
#include <util/crc16.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2

boolean DEBUG=false;
boolean RADIOON=true;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
int COUNT=0;
byte GPS_TX=3;
byte GPS_RX=4;
SoftwareSerial GPS = SoftwareSerial(GPS_TX, GPS_RX);
byte pinState = 0;
char SS_IN_BYTE;
char GPS_DATA[200];
char GPS_HEADER[6]="GPGGA";
float GPS_TIME;
char * VRPARAM ;
int LATITUDE_DEGREES;
int LATITUDE_MINUTES;
float LATITUDE_SECONDS;
long LATITUDE_DECIMAL;
int LONGITUDE_DEGREES;
int LONGITUDE_MINUTES;
float LONGITUDE_SECONDS;
long LONGITUDE_DECIMAL;
char LATSIGN ='0';
char LONGSIGN ='0';
int COUNTER=0;
int GPS_LOCK;
int SATELLITES;
long ALTITUDE;


char TELEMETRYSTRING[200];
int TEMPINT;
int TEMPEXT;
void rtty_txbit (int bit);
void rtty_txbyte (char c);
void rtty_txstring (char * string);
int RADIO_SPACE_PIN=8;
int RADIO_MARK_PIN=7;

int ALTITUDE_DELTA;
boolean PYRO_FIRED=false;
long TIMETICKS;
long PREVIOUSTIME;
char FLIGHT_STATUS='G';
long PREVIOUS_ALTITUDE;
int VERTICAL_RATE;
boolean LANDING_MODE=0;

void setup()
{
    pinMode(GPS_TX, INPUT);
    pinMode(GPS_RX, OUTPUT);
    pinMode(RADIO_SPACE_PIN,OUTPUT);
    pinMode(RADIO_MARK_PIN,OUTPUT);
    GPS.begin(4800);
    if(DEBUG)
    {
        Serial.begin(9600);
        Serial.println("AVA Flight computer V0.115 DEBUG MODE initiated");
    }
    sensors.begin();
    sensors.requestTemperatures();
    if(sensors.getTempCByIndex(0)<=0||sensors.getTempCByIndex(1)>=84) //Catch the Temp Sensors being silly and reset ? Not sure doesn't happen all the time.
    {
        sensors.begin();
    }
}

void loop()
{
    
        char SS_IN_BYTE = GPS.read();
        while (SS_IN_BYTE != '$')
        {
            char SS_IN_BYTE = GPS.read();
            if (SS_IN_BYTE== '$')
            {
                break;
            }
        }

        SS_IN_BYTE=' '; // NEED TO CHANGE THE BYTE OR NEXT BIT WON'T WORK :)
        COUNT=0;
        while (SS_IN_BYTE != '$')
        {
            char SS_IN_BYTE = GPS.read();
            if (SS_IN_BYTE== '$')
            {
                break;
            }
            GPS_DATA[COUNT] = SS_IN_BYTE;
            COUNT++;
        }
        GPS_LOCK=0;
        if(strncmp(GPS_DATA, GPS_HEADER,5) == 0)
        {

            char * GPS_DATA_VALUES;
            GPS_DATA_VALUES = strtok_r (GPS_DATA,",",&VRPARAM);
            COUNT=0;
            while (GPS_DATA_VALUES != NULL)
            {
                GPS_DATA_VALUES = strtok_r (NULL, ",", &VRPARAM);
                if(COUNT==0)
                {
                    GPS_TIME = strtod(GPS_DATA_VALUES,NULL);
                }
                if(COUNT==1)
                {
                    float LATITUDE= strtod(GPS_DATA_VALUES,NULL);
                    LATITUDE_DEGREES = int(LATITUDE/100);
                    LATITUDE_MINUTES = int(LATITUDE-(LATITUDE_DEGREES*100));
                    LATITUDE_SECONDS= float(60*(LATITUDE-int(LATITUDE)));
                    LATITUDE_DECIMAL = LATITUDE_MINUTES * 1000000L / 60 + LATITUDE_SECONDS * 1000000L / 3600;
                }
                if(COUNT==2)
                {
                    if(*GPS_DATA_VALUES=='S')
                    {
                        LATSIGN='-';
                    }
                    else
                    {
                        LATSIGN='+';
                    }
                }
                if(COUNT==3)
                {
                    float LONGITUDE= strtod(GPS_DATA_VALUES,NULL);
                    LONGITUDE_DEGREES = int(LONGITUDE/100);
                    LONGITUDE_MINUTES = int(LONGITUDE-(LONGITUDE_DEGREES*100));
                    LONGITUDE_SECONDS= float(60*(LONGITUDE-int(LONGITUDE)));
                    LONGITUDE_DECIMAL = LONGITUDE_MINUTES * 1000000L / 60 + LONGITUDE_SECONDS * 1000000L / 3600;
                }
                if(COUNT==4)
                {
                    if(*GPS_DATA_VALUES=='W')
                    {
                        LONGSIGN='-';
                    }
                    else
                    {
                        LONGSIGN='+';
                    }
                }
                if(COUNT==5)
                {
                    if(strtod(GPS_DATA_VALUES,NULL)!=0)
                    {
                        GPS_LOCK=1;
                    }
                    else
                    {
                        GPS_LOCK=0;
                        FLIGHT_STATUS='N';
                    }
                }
                if(COUNT==6)
                {
                    SATELLITES=(int) (strtod(GPS_DATA_VALUES,NULL));
                }
                if(COUNT==8)
                {
                    PREVIOUS_ALTITUDE=ALTITUDE;
                    ALTITUDE=(long) (strtod(GPS_DATA_VALUES,NULL));
                    ALTITUDE_DELTA=ALTITUDE-PREVIOUS_ALTITUDE;
                }
                COUNT++;
            }
        }

        COUNTER++;
        sensors.requestTemperatures();
        TEMPINT=sensors.getTempCByIndex(0);
        TEMPEXT=sensors.getTempCByIndex(1);
        int GPS_HOUR = (int) (GPS_TIME/10000);
        int GPS_MIN = (int) ((GPS_TIME/100-(GPS_HOUR)*100)) ;
        int GPS_SEC = (int) (GPS_TIME-(GPS_HOUR)*10000-GPS_MIN*100);
        PREVIOUSTIME=TIMETICKS;
        TIMETICKS= (GPS_HOUR*3600L)+(GPS_MIN*60)+GPS_SEC;
        long TIMEDELTA=TIMETICKS-PREVIOUSTIME;
        VERTICAL_RATE=float(ALTITUDE_DELTA/TIMEDELTA);

        if(ALTITUDE<1000&&VERTICAL_RATE>=-3&&VERTICAL_RATE<=3)
        {
            FLIGHT_STATUS='G';
        }
        else
        {
            if(VERTICAL_RATE>1)
            {
                FLIGHT_STATUS='A';
            }
            else
            {
                if(VERTICAL_RATE>=-1&&VERTICAL_RATE<=1&&ALTITUDE>10000)
                {
                    FLIGHT_STATUS='F';
                    if(!PYRO_FIRED)
                    {
                        PYRO_FIRED=!PYRO_FIRED;
                        FLIGHT_STATUS='Q';
                        //FIRE PYRO
                    }
                }
                else
                {
                    if(VERTICAL_RATE<0&&VERTICAL_RATE>-3&&ALTITUDE>10000)
                    {
                        FLIGHT_STATUS='L';
                        if(!PYRO_FIRED)
                        {
                            PYRO_FIRED=!PYRO_FIRED;
                            FLIGHT_STATUS='R';
                            //FIRE PYRO
                        }
                    }
                    else
                    {
                        FLIGHT_STATUS='D';
                        if(ALTITUDE<500)
                        {
                            // FIRE RETRO ROCKETS!!
                            FLIGHT_STATUS='S';
                            LANDING_MODE=!LANDING_MODE; // Trigger a siren ? A flare ? :)
                        }
                        if(!PYRO_FIRED)
                        {
                            PYRO_FIRED=!PYRO_FIRED;
                            FLIGHT_STATUS='P';
                            //FIRE PYRO
                        }
                    }
                }
            }

        }



        if (LONGSIGN=='-')
        {
            sprintf(TELEMETRYSTRING,"$$AVA,%u,%02u:%02u:%02u,%i.%06li,%c%i.%06li,%li,%u,%i,%i,%i,%c",COUNTER,GPS_HOUR,GPS_MIN,GPS_SEC,LATITUDE_DEGREES,LATITUDE_DECIMAL,LONGSIGN,LONGITUDE_DEGREES,LONGITUDE_DECIMAL,ALTITUDE,SATELLITES,TEMPINT,TEMPEXT,VERTICAL_RATE,FLIGHT_STATUS);
        }
        else
        {
            sprintf(TELEMETRYSTRING,"$$AVA,%u,%02u:%02u:%02u,%i.%06li,%i.%06li,%li,%u,%i,%i,%i,%c",COUNTER,GPS_HOUR,GPS_MIN,GPS_SEC,LATITUDE_DEGREES,LATITUDE_DECIMAL,LONGITUDE_DEGREES,LONGITUDE_DECIMAL,ALTITUDE,SATELLITES,TEMPINT,TEMPEXT,VERTICAL_RATE,FLIGHT_STATUS);
        }

        unsigned int CHECKSUM = gps_CRC16_checksum(TELEMETRYSTRING);
        char checksum_str[6];


        //  sprintf(checksum_str, "*%04X", CHECKSUM);
        sprintf(checksum_str, "*%04X\n", CHECKSUM); // FOR REAL FLIGHT COMPUTER USE THIS LINE
        strcat(TELEMETRYSTRING, checksum_str);
        if(DEBUG)
        {
            Serial.println(TELEMETRYSTRING);
        }

        if(RADIOON)
        {
            noInterrupts();
            rtty_txstring (TELEMETRYSTRING);
            interrupts();
            delay(1000);
        }

    
}

uint16_t gps_CRC16_checksum (char *string)
{
    size_t i;
    uint16_t crc;
    uint8_t c;

    crc = 0xFFFF;

    // Calculate checksum ignoring the first two $s
    for (i = 2; i < strlen(string); i++)
    {
        c = string[i];
        crc = _crc_xmodem_update (crc, c);
    }

    return crc;
}
//void toggle(int pinNum) {
//   set the LED pin using the pinState variable:
//  digitalWrite(pinNum, pinState);
// if pinState = 0, set it to 1, and vice versa:
//  pinState = !pinState;
//}
void rtty_txstring (char * string)
{

    /* Simple function to sent a char at a time to
    ** rtty_txbyte function.
    ** NB Each char is one byte (8 Bits)
    */

    char c;

    c = *string++;

    while ( c != '\0')
    {
        rtty_txbyte (c);
        c = *string++;
    }
}


void rtty_txbyte (char c)
{
    /* Simple function to sent each bit of a char to
    ** rtty_txbit function.
    ** NB The bits are sent Least Significant Bit first
    **
    ** All chars should be preceded with a 0 and
    ** proceded with a 1. 0 = Start bit; 1 = Stop bit
    **
    ** ASCII_BIT = 7 or 8 for ASCII-7 / ASCII-8
    */

    int i;

    rtty_txbit (0); // Start bit

    // Send bits for for char LSB first

    for (i=0; i<8; i++)
    {
        if (c & 1) rtty_txbit(1);

        else rtty_txbit(0);

        c = c >> 1;

    }

    rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
    if (bit)
    {
        // high
        digitalWrite(RADIO_MARK_PIN, HIGH);
        digitalWrite(RADIO_SPACE_PIN, LOW);

    }
    else
    {
        // low
        digitalWrite(RADIO_SPACE_PIN, HIGH);
        digitalWrite(RADIO_MARK_PIN, LOW);
    }
    delayMicroseconds(10000);
    delayMicroseconds(10150);
//                delay(3000);
}

void callback()
{
    digitalWrite(RADIO_SPACE_PIN, digitalRead(RADIO_SPACE_PIN) ^ 1);
}


