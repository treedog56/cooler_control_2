#include <ctime>
#include <WiFiUdp.h>

WiFiUDP udp;

/* Don't hardwire the IP address or we won't get the benefits of the pool.
    Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP(132, 163, 96, 4); // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
long t_time;
int ht_time;
int mt_time;
int st_time;
uint64_t t_epoch = 0;

// Pad time digits with zeros for HTML
String calcDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  String stringDigi = "";
  if (digits < 10) stringDigi = "0" + String(digits);
  else stringDigi = String(digits);
  return stringDigi;
}


String Print_time(int ht, int mt, int st) {
  String ptime =  calcDigits(ht) + ":" + calcDigits(mt) + ":" + calcDigits(st);
  return ptime;
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

long read_NTP()
{
  //get a random server from the pool
  //WiFi.hostByName(ntpServerName, timeServerIP);
  
  Serial.print("Time Server Ip Address");
  Serial.println(timeServerIP);
  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  int cb;
  unsigned long start = millis();
  do {
    cb = udp.parsePacket();
    Serial.print("packet received, length=");
    Serial.println(cb);
    if (cb >= 1) {
      Serial.print("packet received, length=");
      Serial.println(cb);
      // We've received a packet, read the data from it
      udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:

      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      Serial.print("Seconds since Jan 1 1900 = " );
      Serial.println(secsSince1900);

      // now convert NTP time into everyday time:
      Serial.print("Unix time = ");
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      // subtract seventy years:
      unsigned long epoch = secsSince1900 - seventyYears;
      // print Unix time:
      Serial.println(epoch);

      time_t now = secsSince1900 + 1725;
      tm *ltm = localtime(&now);
      t_time = epoch;
      ht_time = ltm->tm_hour;
      mt_time = ltm->tm_min;
      st_time = ltm->tm_sec;
      Serial.print("Time = ");
      Serial.print(ht_time);
      Serial.print(":");
      Serial.print(mt_time);
      Serial.print(":");
      Serial.print(st_time);
      Serial.println(" ");

      return t_time;


      // print the hour, minute and second:
      //->Serial1.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
      //->Serial1.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
      //->Serial1.print(':');
      if ( ((epoch % 3600) / 60) < 10 ) {
        // In the first 10 minutes of each hour, we'll want a leading '0'
        //->Serial1.print('0');
      }
      //->Serial1.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
      //->Serial1.print(':');
      if ( (epoch % 60) < 10 ) {
        // In the first 10 seconds of each minute, we'll want a leading '0'
        //->Serial1.print('0');
      }
      //->Serial1.println(epoch % 60); // print the second
      break;
    } else {
      Serial.println("no packet yet");
    }
  }
  while ((millis() - start) < 750);

}


long UpdateTime() {
  int daylight_check;
  int TimeArray[24] = {17, 18, 19, 20, 21, 22, 23, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  int DayLightTimeArray[24] = {18, 19, 20, 21, 22, 23, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
  t_time++;
  time_t now = t_time;
  tm *ltm = localtime(&now);
  int wdt_time = ltm->tm_wday;
  int mth_time = ltm->tm_mon;
  mt_time = ltm->tm_min;
  st_time = ltm->tm_sec;
  // adjust for MST
  ht_time = ltm->tm_hour;
  //ht_time = TimeArray[ht_time];

  // adjust for  start of daylight saving time
  //are we in of daylight saving time
  if (mth_time >= 3 && mth_time <= 11) {
    daylight_check = 1;
  }
  //are we out of daylight saving time
  if ((mth_time >= 1 && mth_time < 3) || (mth_time >= 11 && mth_time <= 12)) {
    daylight_check = 0;
  }
  // check if Daylight saving time is changing
  if (wdt_time == 0 && mth_time == 3 && ht_time == 2 && mt_time == 0 && st_time == 0) {
    daylight_check = 1;
  }
  if (wdt_time == 0 && mth_time == 11 && ht_time == 2 && mt_time == 0 && st_time == 0) {
    daylight_check = 0;
  }

  //check if we are in daylight savings time already

  if ((mth_time <= 2) || (mth_time >= 11 )) {
    daylight_check = 0;
  }
  if (daylight_check == 1) {
    ht_time = DayLightTimeArray[ht_time];
  } else {
    //adjust for MST
    ht_time = TimeArray[ht_time];
  }

  Serial.print("Time = ");
  Serial.print(calcDigits(ht_time));
  Serial.print(":");
  Serial.print(calcDigits(mt_time));
  Serial.print(":");
  Serial.print(calcDigits(st_time));
  Serial.println(" ");

  return t_time;

}



void OnUpdateTaskGetNTPTime(uint32_t deltaTime) {
  t_epoch = read_NTP();
  Serial.println("Get Time ");
  //taskManager.StartTask(&taskGetNTPTime); // start the task to turn the LED off
  //taskManager.StopTask(&taskGetNTPTime); // stop trying to turn the LED On
}

