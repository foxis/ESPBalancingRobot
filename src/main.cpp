#include <EasyOTA.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>
#include "Robot.h"

#ifndef USE_LOGGING
#include <ESPUI.h>
#endif

#include "wificonfig.h"

#define LABEL_HEARTBEAT "Heart Beat:"
#define LABEL_VOLTAGE "Analog voltage:"
#define LABEL_INCLINATION "Inclination:"
#define LABEL_SPEED "Speed:"
#define LABEL_HEADING "Heading:"

EasyOTA OTA(ARDUINO_HOSTNAME);
Robot robot(500, 124.0);
#ifdef USE_LOGGING
AsyncWebServer logger_server(80);
AsyncWebSocket logger("/log");
#endif

void heartBeat(unsigned long now) {
  static long prevMillis = 0;
  static bool dotState = 0;

  if ( now - prevMillis > 500000 ) {
    prevMillis = now;
		#ifndef USE_LOGGING
    //dotState = !dotState;
    //ESPUI.print(LABEL_HEARTBEAT, dotState ? "Ping" : "Pong");

    ESPUI.print(LABEL_INCLINATION, robot.getInclination());
    ESPUI.print(LABEL_SPEED, robot.getSpeed());
    ESPUI.print(LABEL_HEADING, robot.getHeading());
    ESPUI.print(LABEL_VOLTAGE, String(10.0 * (analogRead(A0) / 1024.0)));
		#endif
  }
}

#ifndef USE_LOGGING

void speedSlider(Control sender, int type) {
  robot.setSpeed(0.5f * (sender.value.toInt() / 100.0f - .5f));
}
void headingSlider(Control sender, int type) {
  robot.setHeading(2.0f * (sender.value.toInt() / 100.0f - .5f));
}

double ip=0, ii=0, id=0;
double sp=0, si=0, sd=0;

void ipSlider(Control sender, int type) { ip = 25.0 * sender.value.toInt() / 100.0;  robot.setInclinationPID(ip, ii, id); }
void iiSlider(Control sender, int type) { ii = 25.0 * sender.value.toInt() / 100.0;  robot.setInclinationPID(ip, ii, id); }
void idSlider(Control sender, int type) { id = sender.value.toInt() / 100.0;  robot.setInclinationPID(ip, ii, id); }

void spSlider(Control sender, int type) { sp = 10 * sender.value.toInt() / 100.0;  robot.setSpeedPID(sp, si, sd); }
void siSlider(Control sender, int type) { si = sender.value.toInt() / 100.0;  robot.setSpeedPID(sp, si, sd); }
void sdSlider(Control sender, int type) { sd = sender.value.toInt() / 100.0;  robot.setSpeedPID(sp, si, sd); }

void inclinationSlider(Control sender, int type) { robot.setInclination((sender.value.toInt() / 100.0f - .5f) / 4.0); }

#endif

void setup() {
  robot.begin();

  OTA.addAP(WIFI_SSID, WIFI_PASSWORD);

#ifndef USE_LOGGING
  //ESPUI.label(LABEL_HEARTBEAT, COLOR_TURQUOISE, "Ping");
  ESPUI.label(LABEL_VOLTAGE, COLOR_TURQUOISE, "0.0");
  ESPUI.label(LABEL_INCLINATION, COLOR_TURQUOISE, "-");
  ESPUI.label(LABEL_SPEED, COLOR_TURQUOISE, "-");
  ESPUI.label(LABEL_HEADING, COLOR_TURQUOISE, "-");
	//ESPUI.slider("Inclination", &inclinationSlider, COLOR_NONE, "31");
  ESPUI.slider("Speed", &speedSlider, COLOR_NONE, "50");
  ESPUI.slider("Heading", &headingSlider, COLOR_NONE, "50");

	//ESPUI.slider("I P", &ipSlider, COLOR_NONE, "35");
	//ESPUI.slider("I I", &iiSlider, COLOR_NONE, "0");
	//ESPUI.slider("I D", &idSlider, COLOR_NONE, "9");

	//ESPUI.slider("S P", &spSlider, COLOR_NONE, "0");
	// ESPUI.slider("S I", &siSlider, COLOR_NONE, "0");
	//ESPUI.slider("S D", &sdSlider, COLOR_NONE, "0");

  ESPUI.begin("BRC", false);
#endif

#ifdef USE_LOGGING
	SPIFFS.begin();
	logger.onEvent([](AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
		if (type == WS_EVT_DATA && strcmp((char *)data, "get-data") == 0)
		{
			log_t log[LOG_SIZE];
			char buf[255];
			size_t size = robot.dump_log((log_t *)log, 1);

			for (size_t i=0; i < size; i++)
			{
				sprintf(buf, "{\"now\":%u,\"ci\":%.5f, \"ti\":%.5f, \"cs\":%.5f, \"fc\":%.5f, \"ts\":%.5f}",
				  log[i].now,
					log[i].currentInclination, log[i].targetInclination,
					log[i].currentSpeed, log[i].fwdControl, log[i].targetSpeed
				);
				client->text(buf);
			}
		}
	});
	logger_server.addHandler(&logger);
	logger_server.serveStatic("/", SPIFFS, "/").setDefaultFile("log.html");
	// Heap for general Servertest
	logger_server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send(200, "text/plain", String(ESP.getFreeHeap()));
	});

	logger_server.onNotFound(
			[](AsyncWebServerRequest *request) { request->send(404); });
	logger_server.begin();
#endif
}

void loop() {
  unsigned long now = micros();

  OTA.loop(now);

  heartBeat(now);
  robot.loop(now);
}
