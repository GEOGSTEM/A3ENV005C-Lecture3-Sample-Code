#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <HTTPClient.h>

Adafruit_SSD1306 SSD1306(128, 64);

String packSize = "--";
String packet ;
String wifirssi;
float randfloat;

//WiFi Setting--------------------------------------------
char ssid[] = ""; //Edit your Wi-Fi name
char password[] = ""; //Edit your Wi-Fi password
String url = "http://api.thingspeak.com/update?api_key="; //Edit your API Key

void wifi(void) {
  Serial.println("Start connection");
  HTTPClient http;
  wifirssi = WiFi.RSSI();
  
  randfloat = random(200, 300)/10.0;
  
  String url1 = url + "&field1=" + randfloat + "&field2=" + wifirssi;
  Serial.println(url1);
  http.begin(url1);

  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK)      {
    String payload = http.getString();
    Serial.print("Webpage Content=");
    Serial.println(payload);
  } else {
    Serial.println("Transmission Failed");
  }
  http.end();
  
  SSD1306.clearDisplay();
  SSD1306.setCursor(0, 0);
  SSD1306.print("Data Sent"); 
  SSD1306.setCursor(0, 15);
  SSD1306.print("Randfloat:");
  SSD1306.setCursor(25, 15);
  SSD1306.print(randfloat);
  
  
  SSD1306.display();
};

void setup(void) {
  Serial.begin(115200);
  SSD1306.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  SSD1306.setRotation(2);
  SSD1306.setTextSize(1);
  SSD1306.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  SSD1306.invertDisplay(false);
  SSD1306.clearDisplay();
  SSD1306.display();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("WiFi Connected");
}


void loop(void) {
  wifi();
  delay(15000);
}
