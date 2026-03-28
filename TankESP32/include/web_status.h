#ifndef WEB_STATUS_H
#define WEB_STATUS_H

void setupWebServer();  // Register routes + start WiFi AP (call once)
void startWiFi();       // Start WiFi AP + web server
void stopWiFi();        // Stop web server + WiFi radio

#endif // WEB_STATUS_H
