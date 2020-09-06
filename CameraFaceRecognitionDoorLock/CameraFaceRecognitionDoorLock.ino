//Viral Science www.youtube.com/c/viralscience  www.viralsciencecreativity.com
//ESP Camera Artificial Intelligence Face Detection Automatic Door Lock
#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <FirebaseESP32.h>
#include "Base64.h"

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

#define FIREBASE_HOST "iot-project-288607.firebaseio.com"
#define FIREBASE_AUTH "q6lhvIv21AV6Ve5ulxXzyrVt7REtroDdCA7qMAtJ"
FirebaseData firebaseData;

#define Red 13
#define Green 12
#define servoPin 14
#define buttonPin 15
#define MIN_MICROS      800  //544
#define MAX_MICROS      2450

#include <Servo_ESP32.h>
#include "camera_pins.h"
#include "ESP32_ISR_Servo.h"

const char* ssid = "Aichan";
const char* password = "20121998";

const char* mqtt_server = "192.168.43.10";
const uint16_t mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

#define TIMER_INTERRUPT_DEBUG       1
#define ISR_SERVO_DEBUG             1

int my_servo  = -1;

void startCameraServer();

int matchFace = 0;
long prevMillis=0;
int interval = 5000;

static int taskCore = 1;

void connect() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
       Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    startCameraServer();
    Serial.print("Camera Ready! Use 'http://");
    Serial.println(WiFi.localIP());
    
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    Firebase.reconnectWiFi(true);
    Firebase.setMaxRetry(firebaseData, 10);
    Firebase.setMaxErrorQueue(firebaseData, 30); 
    Firebase.enableClassicRequest(firebaseData, true);
}

void setup() {
  pinMode(servoPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  
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

  connect();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
 
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

volatile int angle = 0;
volatile int processing = 0;
volatile bool open_door = false;
volatile int lastState = HIGH;
int angleStep = 5;
int angleMin = 0;
int angleMax = 180;

void loop() {
    if(WiFi.status() != WL_CONNECTED){
        connect();
        return;
    }
    if (!client.connected()) {
        reconnect();
    }
    
    int position;
    if (processing == 1) {
        if (angle == angleMin) {
            for (position = angleMin; position <= angleMax; position+=angleStep) {
                ESP32_ISR_Servos.setPosition(my_servo, position);
            }
            angle = angleMax;
        }
        processing = 0;
    }
    else if (processing == 2) {
        if (angle == angleMax) {
            for (position = angleMax; position >= angleMin; position-=angleStep) {
                ESP32_ISR_Servos.setPosition(my_servo, position);
            }
            angle = angleMin;
        }
        processing = 0;
    }
    else {
        int buttonState = digitalRead(buttonPin);
        if (buttonState != lastState) {
            if (buttonState == LOW) {
                if (open_door == false) {
                    matchFace=0;
                    processing = 1;
                    open_door=true;
                    digitalWrite(Green,HIGH);
                    digitalWrite(Red,LOW);
                    prevMillis=millis();
                }
                else {
                    matchFace=0;
                    processing = 2;
                    open_door=false;
                    digitalWrite(Green,LOW);
                    digitalWrite(Red,HIGH);
                }
            }
            lastState = buttonState;
        }
        else {
            if(matchFace==1 && open_door==false) {
                matchFace=0;
                processing = 1;
                open_door=true;
                digitalWrite(Green,HIGH);
                digitalWrite(Red,LOW);
                prevMillis=millis();
                Serial.println("++++++++++++++++++++++++++++++++++++++");
            }
            else if (matchFace==2) {
                matchFace=0;
                processing = 2;
                open_door=false;
                digitalWrite(Green,LOW);
                digitalWrite(Red,HIGH);
                Serial.println("--------------------------------------");
                char payl[2];
                snprintf (payl, 10, "%d", 2);
                delay(1000);
                client.publish("send_email", payl);
            }
            if (open_door == true && millis()-prevMillis > interval) {
                matchFace=0;
                processing = 2;
                open_door=false;
                digitalWrite(Green,LOW);
                digitalWrite(Red,HIGH);
            }
        }   
    }
}

void post_img() {
    xTaskCreatePinnedToCore(
                    coreTask,   /* Function to implement the task */
                    "coreTask", /* Name of the task */
                    100000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    0,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    taskCore);  /* Core where the task should run */
}

void coreTask( void * pvParameters ){
    FirebaseJson json;
    json.setJsonData("{\"photo\":\"" + Photo2Base64() + "\"}");
    String photoPath = "/esp32cam";
          
    if (Firebase.pushJSON(firebaseData, photoPath, json)) {
        Serial.println("Post successfully...<3");
    } else {
        Serial.println(firebaseData.errorReason());
    }
}

String Photo2Base64() {
    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();  
    if(!fb) {
      Serial.println("Camera capture failed");
      return "";
    }
  
    String imageFile = "data:image/jpeg;base64,";
    char *input = (char *)fb->buf;
    char output[base64_enc_len(3)];
    for (int i=0;i<fb->len;i++) {
      base64_encode(output, (input++), 3);
      if (i%3==0) imageFile += urlencode(String(output));
    }

    esp_camera_fb_return(fb);
    
    return imageFile;
}

String urlencode(String str)
{
    String encodedString="";
    char c;
    char code0;
    char code1;
    char code2;
    for (int i =0; i < str.length(); i++){
      c=str.charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

}
