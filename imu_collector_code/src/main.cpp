// TTGO T-Display ST7789 OLED
//  Using AdafruitGFX library for graphical display
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SD.h>
#include <SPI.h>

// multicast example from espressif esp32 arduino:
//  https://github.com/espressif/arduino-esp32/blob/master/libraries/AsyncUDP/examples/AsyncUDPMulticastServer/AsyncUDPMulticastServer.ino
#include <AsyncUDP.h>
#include <WiFi.h>

// adafruit gps library
#include <Adafruit_GPS.h>
#include <ESP32Time.h>
#include <SoftwareSerial.h>

#include "RTClib.h"

RTC_DS3231 rtc_notgps;
#define I2C_BUS_2_SCL 17
#define I2C_BUS_2_SDA 13

// -----------------------------------------------------------  Logging pin
#define LOG_PIN 33
// SPI pins for SD card
#define SD_MISO 27
#define SD_MOSI 25
#define SD_SCLK 26
#define SD_CS 32

// ------------------------------------------------------------  WIFI STUFF
const IPAddress multicast_ip(224, 10, 10, 10);
const uint16_t multicast_port = 1234;

// wifi settings
const char *ssid = "rtc_gang";
const char *password = "roadtrip#1";

// create a udp object
AsyncUDP udp;

// ------------------------------------------------------------  GUI STUFF
// pinouts for display (TTGO T-Display ST7789 OLED)
// These pinout can be found at https://github.com/Xinyuan-LilyGO/TTGO-T-Display
#define TFT_CS 5
#define TFT_RST 23
#define TFT_DC 16
#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_BL 4  // Display backlight control pin
// Define panel height and padding
const int padding = 5;
const int panel_height = 240 / 3 - 10;
const int text_size = 2;

#define SCREEN_WIDTH 135
#define SCREEN_HEIGHT 240

// Create display object
Adafruit_ST7789 tft =
    Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Declare variables to hold the states of the indicators
bool isMulticastServerUp = false;
bool isDataLoggingEnabled = false;
// Flag to let user know SD card I/O is completed
bool shutdownSafe = false;
bool isRtcUpdated = false;
bool state_changed = true;

TaskHandle_t drawScreenTaskHandle = NULL;
TaskHandle_t sd_card_task_handle;
TaskHandle_t time_multicast_handle;

// ------------------------------------------------------------  GPS + RTC STUFF
#define GPS_RX 21
#define GPS_TX 22

SoftwareSerial gps_ss(GPS_TX, GPS_RX);
Adafruit_GPS GPS(&gps_ss);

// Configuring type for storing gps
#define DATA_LEN 500
#define GPS_DATA_LEN sizeof(uint32_t) * DATA_LEN + sizeof(float) * DATA_LEN * 5

// Create gps_file_data_t struct for writing to SD card
typedef union
{
  struct __attribute__((packed))
  {
    // Timestamp array
    uint32_t timestamp[DATA_LEN];

    // Gyro data array
    float latitude[DATA_LEN];
    float longitude[DATA_LEN];
    float altitude[DATA_LEN];
    float angle[DATA_LEN];
    float speed[DATA_LEN];
  };
  uint8_t raw[GPS_DATA_LEN];
} gps_file_data_t;

// Create two instances of gps_file_data_t for double buffering
gps_file_data_t *gps_file_data;
gps_file_data_t *gps_file_data_out;
bool data_ready_to_write = false;
// cur location in data
int cur_count = 0;

ESP32Time rtc(0);

void drawScreenTask(void *pvParameters)
{
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    state_changed = true;

    // sleep until 500ms after now
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500));
  }
}

void cb_time_multicast(void *parameter)
{
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;){
    Serial.println("Sending multicast packet");
  // Send multicast packet with the current time
  if (isRtcUpdated)
  {
    // Serial.println("Sending multicast packet");
    char message[50];
    sprintf(message, rtc.getTime("%d:%H:%M:%S").c_str());
    udp.writeTo((const uint8_t *)message, strlen(message), multicast_ip,
                multicast_port);
  }
  else
  {
    Serial.println("Multicast Failed: RTC not updated");
  }
  // sleep until 5s after now
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(5000));
  }
}

void sd_card_write(const char *file_prefix, gps_file_data_t *data_out)
{
  // Create file name
  // File name format
  // prefix_MMDD_HHMMSS.bin
  // If file exists, append _1, _2, etc.

  // Retrieve current time str
  //String time_str = rtc.getTime("%H_%M_%S");
  char time_str[60];
  sprintf(time_str, rtc.getTime("%H_%M_%S").c_str());

  // Create file name
  char gps_file_name[100];
  sprintf(gps_file_name, "/%s_%s.bin", file_prefix, time_str);

  //   Serial.println("Writing");

  // Check if file exists
  if (SD.exists(gps_file_name))
  {
    // Serial.println("File exists");
    // If file exists, append _1, _2, etc.
    
    int file_count = 1;
    // Create temp file name
    char gps_file_name_temp[100];
    sprintf(gps_file_name_temp, "/%s_%s_%d.bin", file_prefix, time_str,
            file_count);
    while (SD.exists(gps_file_name_temp))
    {
      file_count++;
      sprintf(gps_file_name_temp, "/%s_%s_%d.bin", file_prefix, time_str,
              file_count);
    }
    sprintf(gps_file_name, "/%s_%s_%d.bin", file_prefix, time_str, file_count);
  }

  //   sprintf(imu_file_name, "_%d.bin", millis()?);
  //   Serial.println("Opening File");
  // Create file
  File gps_file = SD.open(gps_file_name, FILE_WRITE);
  // Write data to file
  gps_file.write(data_out->raw, GPS_DATA_LEN);
  // Close file
  gps_file.close();
  Serial.println("Done Writing");
}

void sd_card_loop(void *parameter)
{
  // create spi instance
  // cs 27
  // sck 17
  // mosi 21
  // miso 22
  SPIClass sd_spi(VSPI);

  sd_spi.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

  // Create sd card instance
  if (SD.begin(SD_CS, sd_spi))
  {
    Serial.println("SD Card Initialized");
  }
  else
  {
    Serial.println("SD Card Failed");
  }

  // sleep this thread for .5 seconds
  vTaskDelay(pdMS_TO_TICKS(500));

  // Wait until shutdown is set false before starting main loop
  while (!isDataLoggingEnabled)
  {
    // Wait for logging to be enabled
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  while (isDataLoggingEnabled)
  {
     vTaskDelay(pdMS_TO_TICKS(100));
    // confirm that imu_file_data_out is not null
    // Serial.println("data ready to write: " + String(data_ready_to_write) + " gps_file_data_out: " + String(gps_file_data_out != nullptr));
    if (data_ready_to_write && gps_file_data_out != nullptr)
    {
        Serial.println("Writing imu");
      sd_card_write("gps", gps_file_data_out);

      // Delete gps_file_data_out
      delete gps_file_data_out;
      gps_file_data_out = nullptr;
      data_ready_to_write = false;
    }
  }
  // Let user know that file I/O is completed
  shutdownSafe = true;
  Serial.println("SD Card Loop Ended");
  while (true)
  {
    delay(1000);
  }
  
}

void setup()
{
  Serial.begin(115200);

  //create i2c bus for rtc and sparkfun imu
  Wire.begin(I2C_BUS_2_SDA, I2C_BUS_2_SCL);
  rtc_notgps.begin(&Wire);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.print("UDP Broadcasting on IP: ");
  Serial.println(WiFi.softAPIP());

  // test the ST7789 display
  tft.init(135, 240); // Init ST7789 display 135x240 pixel
  // write the backlight high
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  // clear the screen with black
  tft.fillScreen(ST77XX_BLACK);
  // set rotation to 3
  tft.setRotation(2);
  // gui task
  xTaskCreatePinnedToCore(drawScreenTask, "drawScreenTask", 2048, NULL, 1,
                          &drawScreenTaskHandle, 0);

  // Create instances of gps_file_data_t
  gps_file_data = new gps_file_data_t();
  gps_file_data_out = nullptr;

  // create a task unpinned to core to send multicast packets
  xTaskCreate(cb_time_multicast, "cb_time_multicast", 2048, NULL, 1, &time_multicast_handle);

  xTaskCreate(sd_card_loop, "sd_card_loop", /* Name of the task */
                          4096,                        /* Stack size in words */
                          NULL,                         /* Task input parameter */
                          1,                            /* Priority of the task */
                          &sd_card_task_handle);         /* Task handle. */
  // set gpio 35 as input button and pull high
  pinMode(35, INPUT_PULLUP);
  // set gpio 0 as input button and pull high
  pinMode(0, INPUT_PULLUP);

  // Set logging pin as output and low
  // Commands all microcontrollers to log
  pinMode(LOG_PIN, OUTPUT);
  digitalWrite(LOG_PIN, LOW);

  // setup gps and rtc
  gps_ss.begin(9600);
  // send all data command and 10hz update rate
  gps_ss.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps_ss.println(PMTK_SET_NMEA_UPDATE_1HZ);

  // Create a canvas with the same dimensions as the screen
  GFXcanvas16 canvas = GFXcanvas16(SCREEN_WIDTH, SCREEN_HEIGHT);
}

void drawIndicators()
{
  GFXcanvas16 *canvas = new GFXcanvas16(SCREEN_WIDTH, SCREEN_HEIGHT);
  canvas->fillScreen(ST77XX_BLACK);
  canvas->setTextSize(text_size);
  canvas->setTextColor(ST77XX_WHITE);

  int y = padding;

  // Draw the Multicast Server indicator
  canvas->fillRoundRect(padding, y, SCREEN_WIDTH - padding * 2, panel_height,
                        10, isMulticastServerUp ? ST77XX_CYAN : 0xf5e942);
  canvas->setCursor(padding * 2, y + panel_height / 2 - (text_size * 4));
  canvas->setTextColor(isMulticastServerUp ? ST77XX_BLACK : ST77XX_WHITE);
  canvas->print("MC");

  // Draw the Data Logging indicator
  y += panel_height + padding;
  canvas->fillRoundRect(padding, y, SCREEN_WIDTH - padding * 2, panel_height,
                        10, isDataLoggingEnabled ? ST77XX_CYAN : 0xf5e942);
  canvas->setCursor(padding * 2, y + panel_height / 2 - (text_size * 4));
  canvas->setTextColor(isDataLoggingEnabled ? ST77XX_BLACK : ST77XX_WHITE);
  canvas->print("Log");

  // Draw the RTC indicator
  y += panel_height + padding;
  canvas->fillRoundRect(padding, y, SCREEN_WIDTH - padding * 2, panel_height,
                        10, isRtcUpdated ? ST77XX_CYAN : 0xf5e942);
  canvas->setCursor(padding * 2, y + panel_height / 2 - (text_size * 4));
  canvas->setTextColor(isRtcUpdated ? ST77XX_BLACK : ST77XX_WHITE);
  // print the date and time
  // canvas->print(rtc_notgps.now().timestamp(DateTime::TIMESTAMP_FULL));
  canvas->print(rtc.getTime("%H:%M:%S").c_str());
  // canvas->print("RTC");

  tft.startWrite();
  tft.drawRGBBitmap(0, 0, canvas->getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT);
  tft.endWrite();

  delete canvas;
}

// check is the multicast server is running
bool isMulticastServerRunning()
{
  // // check if async udp is on
  // if (!udp.connected())
  // {
  //   return false;
  // }
  // else
    return true;
}

// check if the data logging button is pressed
bool isDataLogging()
{
  // if I press gpio35 to low, toggle logging on and off
  if (digitalRead(35) == LOW)
  {
    isDataLoggingEnabled = !isDataLoggingEnabled;
    state_changed = true;
    Serial.println("Logging: " + String(isDataLoggingEnabled));
  }
  return isDataLoggingEnabled;
}

bool isRtcUpdatedSinceLastPowerOn()
{
  // return true if year is 2023
  if (rtc.getYear() == 2023)
  {
    return true;
  }
  else
  { 
    // Serial.println("Setting ESP RTC with the crusty box RTC!");
    struct tm wt;
    // get the time from the rtc_notgps
    DateTime time_now = rtc_notgps.now();
    wt.tm_hour = time_now.hour();
    wt.tm_min = time_now.minute();
    wt.tm_sec = time_now.second();
    wt.tm_mday = time_now.day();
    wt.tm_mon = time_now.month();
    wt.tm_year = time_now.year() - 1900;

    // set the time to the rtc
    rtc.setTime((unsigned long)mktime(&wt), 0);
    return false;
  }
}

void updateRTC()
{
  // state_changed = true;

  // time_t candidateDateAndTime = 0;  // Initialize with a default value

  // // struct tm wt;
  // // wt.tm_hour = GPS.hour;
  // // wt.tm_min = GPS.minute;
  // // wt.tm_sec = GPS.seconds;

  // // wt.tm_mday = GPS.day;
  // // wt.tm_mon = GPS.month - 1;
  // // wt.tm_year = GPS.year + 100;

  // // // TODO: maybe do another check on this time....
  // // candidateDateAndTime = mktime(&wt);

  // // Set the date and time
  // // rtc.setTime((unsigned long)candidateDateAndTime, 0);

  // rtc_notgps.adjust(DateTime(GPS.year + 2000, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds));
}


uint32_t timer = millis();
uint32_t gps_data_timer = millis();
int i =0;
void loop()
{
  // Get the state of the indicators
  isMulticastServerUp = isMulticastServerRunning();
  isDataLoggingEnabled = isDataLogging();
  isRtcUpdated = isRtcUpdatedSinceLastPowerOn();

  // only draw the indicators if the state has changed
  if (state_changed)
  {
    // Update logging pin on state change
    digitalWrite(LOG_PIN, isDataLoggingEnabled);

    drawIndicators();
    state_changed = false;
  }

  while (gps_ss.available() > 0)
  {
    GPS.read();
  }

  if (GPS.newNMEAreceived())
  {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
    else if (GPS.hour >= 0 && GPS.minute >= 0 && GPS.seconds >= 0 &&
             GPS.day >= 1 && GPS.month >= 1 && GPS.year >= 23)
    {
      if (isRtcUpdatedSinceLastPowerOn())
      {
        Serial.println("RTC already updated");
      }
      else
        updateRTC();
    }
    else
      Serial.println("timecheck failed");
  }

  // Get GPS position data
  if (isDataLoggingEnabled && cur_count < DATA_LEN)
  {
    if (i == 100)
    {
      if (GPS.fix)
      {
        gps_file_data->timestamp[cur_count] = millis();
        gps_file_data->latitude[cur_count] = GPS.latitude;
        gps_file_data->longitude[cur_count] = GPS.longitude;
        gps_file_data->speed[cur_count] = GPS.speed;
        gps_file_data->angle[cur_count] = GPS.angle;
        gps_file_data->altitude[cur_count] = GPS.altitude;
      }
      else
      {
        gps_file_data->timestamp[cur_count] = millis();
        gps_file_data->latitude[cur_count] = 0;
        gps_file_data->longitude[cur_count] = 0;
        gps_file_data->speed[cur_count] = 0;
        gps_file_data->angle[cur_count] = 0;
        gps_file_data->altitude[cur_count] = 0;
      }
      cur_count++;
      i = 0;
    }
    else
    {
      i++;
    }
  }

  if (cur_count == DATA_LEN && gps_file_data_out == nullptr)
  {
    Serial.println("Data ready to write");
    gps_file_data_out = gps_file_data;
    data_ready_to_write = true;
    cur_count = 0;
    gps_file_data = new gps_file_data_t();
  }

}