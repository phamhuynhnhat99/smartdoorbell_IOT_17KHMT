//Viral Science www.youtube.com/c/viralscience  www.viralsciencecreativity.com
//ESP Camera Artificial Intelligence Face Detection Automatic Door Lock
#include "esp_camera.h"
#include <WiFi.h>
#include <Servo_ESP32.h>
#include "camera_pins.h"
#include "ESP32_ISR_Servo.h"

//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#define Red 13
#define Green 12
#define servoPin 14
#define MIN_MICROS      800  //544
#define MAX_MICROS      2450

const char* ssid = "Helicoland";
const char* password = "chichxongchay";

#define TIMER_INTERRUPT_DEBUG       1
#define ISR_SERVO_DEBUG             1

int my_servo  = -1;

void startCameraServer();

boolean matchFace = false;
boolean open_door = false;
long prevMillis=0;
int interval = 5000;

void setup() {
  pinMode(servoPin,OUTPUT);
  
  //Select ESP32 timer USE_ESP32_TIMER_NO
  ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);
  my_servo = ESP32_ISR_Servos.setupServo(servoPin, MIN_MICROS, MAX_MICROS);
  
  pinMode(Red,OUTPUT);
  pinMode(Green,OUTPUT);
  digitalWrite(Red,HIGH);
  digitalWrite(Green,LOW);
  
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

volatile int angle = 0;
volatile int processing = 0;
volatile int angleStep = 5;
int angleMin = 0;
int angleMax = 45;
void loop() {
    int position;
    if (processing == 1) {
        for (position = 0; position <= 180; position++) 
        {
            ESP32_ISR_Servos.setPosition(my_servo, position);
        }
        processing = 0;
    }
    else if (processing == 2) {
        for (position = 180; position >= 0; position--) 
        {
            ESP32_ISR_Servos.setPosition(my_servo, position);
        }
        processing = 0;
    }
    else {
        if(matchFace==true && open_door==false)
        {
            processing = 1;
            open_door=true;
            digitalWrite(Green,HIGH);
            digitalWrite(Red,LOW);
            prevMillis=millis();
        }
        if (open_door == true && millis()-prevMillis > interval)
        {
            processing = 2;
            open_door=false;
            matchFace=false;
            digitalWrite(Green,LOW);
            digitalWrite(Red,HIGH);
        }   
    }
}
