#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <RTClib.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

/***************************************************************************/
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define CONTROL_PIN D5
#define BUTTON_PIN D6
#define RESET_BUTTON_PIN D7
#define LED_GREEN_PIN D0
#define LED_RED_PIN   D8

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_INA219 ina219;
ESP8266WebServer server(80);
RTC_DS3231 rtc;

unsigned long previousMillis = 0;
unsigned long interval = 500; // 500 ms

float shuntvoltage = 0.0;
float busvoltage = 0.0;
float current = 0.0;
float loadvoltage = 0.0;
float power = 0.0;
float zeroCurrentOffset_mA = 0.0;
float voltageScale = 1.0;

bool loadState = false; // ON/OFF
bool counting = false;
unsigned long startMillis = 0;
unsigned long elapsedMillis = 0;
float accumulatedWh = 0.0;
float ina219ScaleFactor = 10.0; // New shunt = 0.01? instead of 0.1?

enum DisplayMode {SHOW_TIMER, SHOW_TIME, SHOW_DATE, SHOW_WEEKDAY};
DisplayMode currentMode = SHOW_TIMER;
unsigned long lastToggle = 0;
const unsigned long toggleInterval = 5000; // 5 sec

bool manualOverride = false;
bool manualState = false;
bool lastScheduleState = false;

/***************************************************************************/
// Schedule struct
#define MAX_SCHEDULES 10
struct Schedule {
 bool active = false;
 uint8_t dayOfWeek = 255; // 0=Sun..6=Sat, 255=Everyday
 uint8_t hourOn = 0;
 uint8_t minuteOn = 0;
 uint8_t hourOff = 0;
 uint8_t minuteOff = 0;
};
Schedule schedules[MAX_SCHEDULES];

/***************************************************************************/
// Helper functions
void updateLED() {
 if (loadState) {
   digitalWrite(LED_GREEN_PIN, HIGH);
   digitalWrite(LED_RED_PIN, LOW);
 } else {
   digitalWrite(LED_GREEN_PIN, LOW);
   digitalWrite(LED_RED_PIN, HIGH);
 }
}

void loadOn() {
 // *** MODIFIED FOR ACTIVE-HIGH CONTROL (3904) ***
 pinMode(CONTROL_PIN, OUTPUT);
 digitalWrite(CONTROL_PIN, HIGH); // HIGH to turn ON
 loadState = true;
 updateLED();
 Serial.println("Load turned ON (Active-HIGH)");
}

void loadOff() {
 // *** MODIFIED FOR ACTIVE-HIGH CONTROL (3904) ***
 pinMode(CONTROL_PIN, OUTPUT); // Keep as OUTPUT for defined LOW state
 digitalWrite(CONTROL_PIN, LOW); // LOW to turn OFF
 loadState = false;
 updateLED();
 Serial.println("Load turned OFF (Active-LOW)");
}

// Manual override
void loadOnManual() {
 loadOn();
 manualOverride = true;
 manualState = true;
 Serial.println("Manual override: load ON");
}

void loadOffManual() {
 loadOff();
 manualOverride = true;
 manualState = false;
 Serial.println("Manual override: load OFF");
}

/***************************************************************************/
// Persistent schedules
void saveSchedules() {
 File file = LittleFS.open("/schedules.json", "w");
 if (!file) return;

 file.print("[");
 bool first = true;
 for (int i = 0; i < MAX_SCHEDULES; i++) {
   if (!schedules[i].active) continue;
   if (!first) file.print(",");
   file.printf("{\"day\":%d,\"hourOn\":%d,\"minuteOn\":%d,\"hourOff\":%d,\"minuteOff\":%d}",
               schedules[i].dayOfWeek,
               schedules[i].hourOn,
               schedules[i].minuteOn,
               schedules[i].hourOff,
               schedules[i].minuteOff);
   first = false;
 }
 file.print("]");
 file.close();
}

void loadSchedules() {
 if (!LittleFS.exists("/schedules.json")) return;

 File file = LittleFS.open("/schedules.json", "r");
 if (!file) return;

 DynamicJsonDocument doc(1024);
 DeserializationError error = deserializeJson(doc, file);
 file.close();
 if (error) {
   Serial.println("Failed to parse schedules.json");
   return;
 }

 for (int i = 0; i < MAX_SCHEDULES; i++) schedules[i].active = false;
 int i = 0;
 for (JsonObject obj : doc.as<JsonArray>()) {
   if (i >= MAX_SCHEDULES) break;
   schedules[i].active = true;
   schedules[i].dayOfWeek = obj["day"];
   schedules[i].hourOn = obj["hourOn"];
   schedules[i].minuteOn = obj["minuteOn"];
   schedules[i].hourOff = obj["hourOff"];
   schedules[i].minuteOff = obj["minuteOff"];
   i++;
 }
}

/***************************************************************************/
// Schedule helpers
int scheduleStartMinutes(const Schedule &s) { return s.hourOn * 60 + s.minuteOn; }
int scheduleEndMinutes(const Schedule &s) { return s.hourOff * 60 + s.minuteOff; }

/***************************************************************************/
void checkSchedules() {
 DateTime now = rtc.now();
 uint8_t day = now.dayOfTheWeek();
 int nowMinutes = now.hour() * 60 + now.minute();

 // --- Step 1: Check if any schedules exist ---
 bool anySchedule = false;
 for (int i = 0; i < MAX_SCHEDULES; i++) {
   if (schedules[i].active) {
     anySchedule = true;
     break;
   }
 }
 if (!anySchedule) {
   // No schedules at all -> full manual control
   return;
 }

 // --- Step 2: Determine if a schedule is currently active (ON period) ---
 bool scheduleActiveNow = false;
 for (int i = 0; i < MAX_SCHEDULES; i++) {
   if (!schedules[i].active) continue;
   if (schedules[i].dayOfWeek != 255 && schedules[i].dayOfWeek != day) continue;

   int onMins = scheduleStartMinutes(schedules[i]);
   int offMins = scheduleEndMinutes(schedules[i]);

   if (onMins <= offMins) {
     if (nowMinutes >= onMins && nowMinutes < offMins) {
       scheduleActiveNow = true;
       break;
     }
   } else { // Overnight schedule (e.g. 22:00-06:00)
     if (nowMinutes >= onMins || nowMinutes < offMins) {
       scheduleActiveNow = true;
       break;
     }
   }
 }

 // --- Step 3: Handle schedule transitions ---
 // Detect when the schedule *changes phase* (OFF->ON or ON->OFF)
 if (scheduleActiveNow != lastScheduleState) {
   Serial.println("Schedule transition detected");
   lastScheduleState = scheduleActiveNow;
   manualOverride = false; // clear manual override at every transition

   if (scheduleActiveNow) {
     // ON period started
     loadOn();
     Serial.println("Schedule ON period started -> load ON");
   } else {
     // OFF period started
     loadOff();
     Serial.println("Schedule OFF period started -> load OFF");
   }
 } else {
   // --- Step 4: Between transitions, schedule does nothing ---
   if (manualOverride) {
     // User manually toggled -> ignore schedule until next transition
     Serial.println("Manual override active, ignoring schedule");
   } else {
     // No override, maintain current load state (no enforcement)
     Serial.println("Within same schedule phase, no changes needed");
   }
 }
}



/***************************************************************************/
// INA219
void calibrateINA219() {
 Serial.println("Calibrating INA219, ensure no load connected.");
 float sum = 0.0;
 const int samples = 50;
 for(int i=0;i<samples;i++){ sum += ina219.getCurrent_mA(); delay(20); }
 zeroCurrentOffset_mA = sum / samples;
 voltageScale = 1.0;
}

void readINA219() {
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V() * voltageScale;

  // Apply scaling for 0.01Ω shunt
  current = ((ina219.getCurrent_mA() - zeroCurrentOffset_mA) / 1000.0f) * ina219ScaleFactor;

  loadvoltage = busvoltage + (shuntvoltage / 1000.0f);
  power = loadvoltage * current;

  // Cleanup noise
  if (current < 0) current = -current;
  if (power < 0) power = -power;

  if (current < 0.01f && loadvoltage < 0.7f) {
    current = 0;
    loadvoltage = 0;
    power = 0;
  }

  // --- SAFETY PROTECTION LOGIC ---
  // If Voltage > 18V OR Current > 3.5A
  if (loadState && (loadvoltage > 18.0f || current > 3.5f)) {
    Serial.println("!!! SAFETY CUTOFF TRIGGERED !!!");
    if (loadvoltage > 18.0f) Serial.printf("Overvoltage: %.2fV\n", loadvoltage);
    if (current > 3.5f) Serial.printf("Overcurrent: %.2fA\n", current);
    
    loadOffManual(); // This sets manualOverride = true
  }
}

/***************************************************************************/
// OLED
void showDisplay() {
 display.clearDisplay();
 display.setTextColor(WHITE);
 int leftMargin = 5;

 display.setTextSize(2);
 display.setCursor(leftMargin, 0);
 display.print(loadvoltage, 3);
 display.setTextSize(1); display.print("V");

 display.setTextSize(2);
 display.setCursor(leftMargin, 22);
 display.print(current, 3);
 display.setTextSize(1); display.print("A");

 display.setTextSize(2);
 display.setCursor(leftMargin, 44);
 display.print(power, 3);
 display.setTextSize(1); display.print("W");

 display.setTextSize(1);
 DateTime now = rtc.now();
 char buf[16]; String label;
 int yLabel=0, yValue=8;

 switch(currentMode){
   case SHOW_TIMER: {
     unsigned long totalMillis=elapsedMillis + (counting?millis()-startMillis:0);
     unsigned long seconds=totalMillis/1000;
     sprintf(buf,"%02lu:%02lu:%02lu",seconds/3600,(seconds%3600)/60,seconds%60);
     label="Timer"; break;
   }
   case SHOW_TIME:
     sprintf(buf,"%02d:%02d:%02d",now.hour(),now.minute(),now.second());
     label="Time";
     break;
   case SHOW_DATE:
     sprintf(buf,"%02d.%02d.%04d",now.day(),now.month(),now.year());
     label="Date";
     break;
   case SHOW_WEEKDAY: {
     const char* days[]={"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
     sprintf(buf,"%s",days[now.dayOfTheWeek()]);
     label="Day";
     break;
   }
 }

 int valueWidth=strlen(buf)*6;
 int labelWidth=label.length()*6;
 int rightXValue=SCREEN_WIDTH-valueWidth;
 int rightXLabel=rightXValue+(valueWidth-labelWidth);
 display.setCursor(rightXLabel,yLabel); display.print(label);
 display.setCursor(rightXValue,yValue); display.print(buf);

 char whStr[12]; sprintf(whStr,"%.3f",accumulatedWh);
 display.setCursor(SCREEN_WIDTH-2*6,20); display.print("Wh");
 display.setCursor(SCREEN_WIDTH-strlen(whStr)*6,28); display.print(whStr);

 String sLoad="Load", sState=loadState?"ON":"OFF";
 display.setCursor(SCREEN_WIDTH-sLoad.length()*6,44); display.print(sLoad);
 display.setCursor(SCREEN_WIDTH-sState.length()*6,52); display.print(sState);

 display.display();
}

/***************************************************************************/
// Web Handlers
void handleRoot() {
 String html = "<html><head><title>IoT Energy Meter</title>"
               "<style>body{font-family:sans-serif;text-align:center;background:#111;color:white;} "
               "button{padding:10px 20px;margin:10px;font-size:16px;border:none;border-radius:6px;} "
               ".on{background:limegreen;color:white;} .off{background:tomato;color:white;} "
               ".reset{background:gold;color:black;}</style>"
               "<script>"
               "function fetchData(){fetch('/data').then(resp=>resp.json()).then(data=>{"
               "document.getElementById('voltage').innerText=data.voltage+' V';"
               "document.getElementById('current').innerText=data.current+' A';"
               "document.getElementById('power').innerText=data.power+' W';"
               "document.getElementById('state').innerText=data.loadState;"
               "document.getElementById('timer').innerText=data.timer;"
               "document.getElementById('time').innerText=data.time;"
               "document.getElementById('date').innerText=data.date;"
               "document.getElementById('day').innerText=data.day;"
               "document.getElementById('wh').innerText=data.wh;"
               "});} setInterval(fetchData,500);"
               "function resetData(){fetch('/reset');}</script></head><body>";

 html += "<h2>IoT Energy Meter</h2>";
 html += "<p>Voltage: <b id='voltage'>0.000 V</b></p>";
 html += "<p>Current: <b id='current'>0.000 A</b></p>";
 html += "<p>Power: <b id='power'>0.000 W</b></p>";
 html += "<p>Load Status: <b id='state'>OFF</b></p>";
 html += "<p>Timer: <b id='timer'>00:00:00</b></p>";
 html += "<p>RTC Time: <b id='time'>00:00:00</b></p>";
 html += "<p>RTC Date: <b id='date'>00.00.0000</b></p>";
 html += "<p>Day: <b id='day'>---</b></p>";
 html += "<p>Energy (Wh): <b id='wh'>0.000</b></p>";
 html += "<a href=\"/on\"><button class='on'>Turn ON</button></a>";
 html += "<a href=\"/off\"><button class='off'>Turn OFF</button></a>";
 html += "<button class='reset' onclick='resetData()'>Reset Timer & Wh</button>";

 // Set RTC
 html += "<h3>Set Date & Time</h3>"
         "<form action=\"/setdatetime\" method=\"get\">"
         "<input type='number' name='day' min='1' max='31' placeholder='DD' required> . "
         "<input type='number' name='month' min='1' max='12' placeholder='MM' required> . "
         "<input type='number' name='year' min='2000' max='2099' placeholder='YYYY' required>"
         "&nbsp;&nbsp;"
         "<input type='number' name='hour' min='0' max='23' placeholder='HH' required> : "
         "<input type='number' name='minute' min='0' max='59' placeholder='MM' required> : "
         "<input type='number' name='second' min='0' max='59' placeholder='SS' required>"
         "<button type='submit'>Set Date & Time</button>"
         "</form>";

 // Add Schedule
 html += "<h3>Add Schedule</h3>"
         "<form action='/addSchedule' method='get'>"
         "<select name='day'><option value='255'>Everyday</option>"
         "<option value='0'>Sunday</option><option value='1'>Monday</option><option value='2'>Tuesday</option>"
         "<option value='3'>Wednesday</option><option value='4'>Thursday</option><option value='5'>Friday</option>"
         "<option value='6'>Saturday</option></select> &nbsp;"
         "ON: <input type='time' name='timeOn' required> OFF: <input type='time' name='timeOff' required>"
         "<button type='submit'>Add Schedule</button></form>";

 // Remove schedule
 html += "<h3>Existing Schedules</h3><ul>";
 const char* days[]={"Sun","Mon","Tue","Wed","Thu","Fri","Sat","Everyday"};
 for(int i=0;i<MAX_SCHEDULES;i++){
   if(!schedules[i].active) continue;
   String dayStr=(schedules[i].dayOfWeek==255)? "Everyday" : days[schedules[i].dayOfWeek];
   char buf[16];
   sprintf(buf,"%02d:%02d-%02d:%02d", schedules[i].hourOn, schedules[i].minuteOn,
           schedules[i].hourOff, schedules[i].minuteOff);
   html += "<li>"+dayStr+" "+String(buf)+" <a href='/removeSchedule?index="+String(i)+"'>Remove</a></li>";
 }
 html += "</ul>";

 html += "<p><small>AP IP: 192.168.4.1</small></p>";
 html += "</body></html>";

 server.send(200,"text/html",html);
}

// AJAX JSON endpoint
void handleData() {
 unsigned long totalMillis = elapsedMillis + (counting?millis()-startMillis:0);
 unsigned long seconds = totalMillis/1000;
 unsigned long hh = seconds/3600;
 unsigned long mm = (seconds%3600)/60;
 unsigned long ss = seconds%60;
 char timerStr[9]; sprintf(timerStr,"%02lu:%02lu:%02lu",hh,mm,ss);

 DateTime now = rtc.now();
 char timeStr[9], dateStr[11], dayStr[4];
 sprintf(timeStr,"%02d:%02d:%02d",now.hour(),now.minute(),now.second());
 sprintf(dateStr,"%02d.%02d.%04d",now.day(),now.month(),now.year());
 const char* daysWeek[]={"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
 sprintf(dayStr,"%s",daysWeek[now.dayOfTheWeek()]);

 String json="{";
 json+="\"voltage\":\""+String(loadvoltage,3)+"\",";
 json+="\"current\":\""+String(current,3)+"\",";
 json+="\"power\":\""+String(power,3)+"\",";
 json+="\"loadState\":\""+String(loadState?"ON":"OFF")+"\",";
 json+="\"timer\":\""+String(timerStr)+"\",";
 json+="\"time\":\""+String(timeStr)+"\",";
 json+="\"date\":\""+String(dateStr)+"\",";
 json+="\"day\":\""+String(dayStr)+"\",";
 json+="\"wh\":\""+String(accumulatedWh,3)+"\"";
 json+="}";
 server.send(200,"application/json",json);
}

// Reset
void handleReset() { elapsedMillis=0; accumulatedWh=0; counting=false; startMillis=0; server.sendHeader("Location","/"); server.send(303); }

// Set RTC
void handleSetDateTime() {
 if(server.hasArg("day") && server.hasArg("month") && server.hasArg("year") &&
    server.hasArg("hour") && server.hasArg("minute") && server.hasArg("second")){
      int d=server.arg("day").toInt();
      int m=server.arg("month").toInt();
      int y=server.arg("year").toInt();
      int h=server.arg("hour").toInt();
      int min=server.arg("minute").toInt();
      int s=server.arg("second").toInt();
      rtc.adjust(DateTime(y,m,d,h,min,s));
      Serial.printf("RTC updated: %02d.%02d.%04d %02d:%02d:%02d\n",d,m,y,h,min,s);
 }
 server.sendHeader("Location","/"); server.send(303);
}

// Load control
void handleOn() { loadOnManual(); handleRoot(); }
void handleOff() { loadOffManual(); handleRoot(); }

// Add schedule
void handleAddSchedule() {
 if(!server.hasArg("day") || !server.hasArg("timeOn") || !server.hasArg("timeOff")) { server.sendHeader("Location","/"); server.send(303); return; }
 int day=server.arg("day").toInt();
 String timeOn=server.arg("timeOn"); String timeOff=server.arg("timeOff");
 int hOn=timeOn.substring(0,2).toInt(); int mOn=timeOn.substring(3,5).toInt();
 int hOff=timeOff.substring(0,2).toInt(); int mOff=timeOff.substring(3,5).toInt();

 for(int i=0;i<MAX_SCHEDULES;i++){
   if(!schedules[i].active){
     schedules[i].active=true;
     schedules[i].dayOfWeek=day;
     schedules[i].hourOn=hOn; schedules[i].minuteOn=mOn;
     schedules[i].hourOff=hOff; schedules[i].minuteOff=mOff;
     saveSchedules();
     break;
   }
 }
 server.sendHeader("Location","/"); server.send(303);
}

// Remove schedule
void handleRemoveSchedule() {
 if(!server.hasArg("index")) { server.sendHeader("Location","/"); server.send(303); return; }
 int idx=server.arg("index").toInt();
 if(idx>=0 && idx<MAX_SCHEDULES){ schedules[idx].active=false; saveSchedules(); }
 server.sendHeader("Location","/"); server.send(303);
}

/***************************************************************************/
void setup() {
 Serial.begin(115200);
 delay(1000);

 if(!LittleFS.begin()){ Serial.println("LittleFS failed"); while(1); }
 loadSchedules();
 loadOff();

 WiFi.softAP("EnergyMeter_AP","12345678");
 Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

 server.on("/", handleRoot);
 server.on("/data", handleData);
 server.on("/on", handleOn);
 server.on("/off", handleOff);
 server.on("/reset", handleReset);
 server.on("/setdatetime", handleSetDateTime);
 server.on("/addSchedule", handleAddSchedule);
 server.on("/removeSchedule", handleRemoveSchedule);
 server.begin();

 Wire.begin(D2,D1);
 if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)){ Serial.println("SSD1306 failed"); while(1); }
 display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(1);
 display.setCursor(10,30); display.print("IoT Energy Meter"); display.display(); delay(1000);

 if(!ina219.begin()){ Serial.println("INA219 not found"); while(1); }
 if(!rtc.begin()){ Serial.println("RTC not found"); while(1); }
 if(rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));

 pinMode(BUTTON_PIN, INPUT_PULLUP);
 pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
 pinMode(LED_GREEN_PIN, OUTPUT);
 pinMode(LED_RED_PIN, OUTPUT);
 updateLED();

 calibrateINA219();
}

void loop() {
 server.handleClient();

 static bool lastButtonState=HIGH;
 bool buttonState=digitalRead(BUTTON_PIN);
 if(lastButtonState==HIGH && buttonState==LOW){
   // toggle by physical button (respects manual override logic through loadOnManual()/loadOffManual())
   if(loadState) loadOffManual(); else loadOnManual();
   Serial.printf("Physical button pressed. loadState now: %s, manualOverride=%s\n", loadState ? "ON":"OFF", manualOverride ? "YES":"NO");
 }
 lastButtonState=buttonState;

 static bool lastResetState=HIGH;
 bool resetState=digitalRead(RESET_BUTTON_PIN);
 if(lastResetState==HIGH && resetState==LOW){ elapsedMillis=0; accumulatedWh=0; counting=false; startMillis=0; Serial.println("Reset button pressed: timer & Wh cleared"); }
 lastResetState=resetState;

 unsigned long currentMillis=millis();
 if(currentMillis-lastToggle>=toggleInterval){ lastToggle=currentMillis; currentMode=static_cast<DisplayMode>((currentMode+1)%4); }

 if(currentMillis-previousMillis>=interval){
   float deltaHours=(currentMillis-previousMillis)/3600000.0;
   previousMillis=currentMillis;
   readINA219();

   if(power>=0.1f){ if(!counting) startMillis=currentMillis; counting=true; accumulatedWh+=power*deltaHours; }
   else{ if(counting){ elapsedMillis+=currentMillis-startMillis; counting=false; } }

   checkSchedules();
   showDisplay();
 }
}