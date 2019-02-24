// Arduino APRS Tracker (aat) with Arduino Pro Mini 3.3V/8 MHz
// Fork of my https://github.com/billygr/arduino-aprs-tracker for stationary use (without GPS)
// Update the lat/lon variables with your location

#include <SimpleTimer.h>
#include <LibAPRS.h>

// Single shot button
#define BUTTON_PIN 10

// LibAPRS
#define OPEN_SQUELCH false
#define ADC_REFERENCE REF_3V3

// APRS settings
char APRS_CALLSIGN[]="NOCALL";
const int APRS_SSID=5;
char APRS_SYMBOL='>';

// Timer
#define TIMER_DISABLED -1

SimpleTimer timer;

char aprs_update_timer_id = TIMER_DISABLED;
bool send_aprs_update = false;
//long instead of float for latitude and longitude
long lat = 0;
long lon = 0;

// buffer for conversions
#define CONV_BUF_SIZE 16
static char conv_buf[CONV_BUF_SIZE];

void setup()  
{
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  Serial.println(F("Arduino APRS Tracker"));

  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(APRS_CALLSIGN,APRS_SSID);
  APRS_setSymbol(APRS_SYMBOL);
  
  aprs_update_timer_id=timer.setInterval(2L*60L*1000L, setAprsUpdateFlag);
}

void loop()
{
  if (digitalRead(BUTTON_PIN)==0)
  {
    while(digitalRead(BUTTON_PIN)==0) {}; //debounce
    Serial.println(F("MANUAL UPDATE"));
    locationUpdate();
  }
  
  if (send_aprs_update) {
    Serial.println(F("APRS UPDATE"));
    locationUpdate();
    send_aprs_update = false;
  }

  timer.run();
}

void aprs_msg_callback(struct AX25Msg *msg) {
}

void locationUpdate() {
  char comment []= "Arduino APRS Tracker";

  APRS_setLat((char*)deg_to_nmea(lat, true));
  APRS_setLon((char*)deg_to_nmea(lon, false));
      
  
  // TX
  APRS_sendLoc(comment, strlen(comment));
 
  // read TX LED pin and wait till TX has finished (PB5) digital write 13 LED_BUILTIN
  while(bitRead(PORTB,5));

}

/*
**  Convert degrees in long format to APRS string format
**  DDMM.hhN for latitude and DDDMM.hhW for longitude
**  D is degrees, M is minutes and h is hundredths of minutes.
**  http://www.aprs.net/vm/DOS/PROTOCOL.HTM
*/
char* deg_to_nmea(long deg, boolean is_lat) {
  bool is_negative=0;
  if (deg < 0) is_negative=1;

  // Use the absolute number for calculation and update the buffer at the end
  deg = labs(deg);

  unsigned long b = (deg % 1000000UL) * 60UL;
  unsigned long a = (deg / 1000000UL) * 100UL + b / 1000000UL;
  b = (b % 1000000UL) / 10000UL;

  conv_buf[0] = '0';
  // in case latitude is a 3 digit number (degrees in long format)
  if( a > 9999) { snprintf(conv_buf , 6, "%04u", a);} else snprintf(conv_buf + 1, 5, "%04u", a);

  conv_buf[5] = '.';
  snprintf(conv_buf + 6, 3, "%02u", b);
  conv_buf[9] = '\0';
  if (is_lat) {
    if (is_negative) {conv_buf[8]='S';}
    else conv_buf[8]='N';
    return conv_buf+1;
    // conv_buf +1 because we want to omit the leading zero
    }
  else {
    if (is_negative) {conv_buf[8]='W';}
    else conv_buf[8]='E';
    return conv_buf;
    }
}

void setAprsUpdateFlag() {
  send_aprs_update = true;
}
