/////////////////////////////////////////////////////////////////
/*
  This project is structured as follows:
  {camera + mic} -> {MQTT publisher} -> {AWS IoT Core} -> {Lambda function} -> {S3 bucket}
  Created by Pei - 2023/03/16
  Maintained by - XXX
*/

/*
  Some tutorials URL link:
  1. ESP32-S3-Korvo platform introduction:
    https://espressif-docs.readthedocs-hosted.com/projects/esp-adf/en/latest/design-guide/dev-boards/user-guide-esp32-s3-korvo-2.html
  2. ESP32-S3 Arduino IDE library installation:
    [ESP32-S3 Board] https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
    [ESP32-S3 Korvo Audio Kit] https://github.com/pschatzmann/arduino-audiokit
  3. ESP32 connect to AWS IoT Core based on Arduino IDE: 
    [Youtube-based] https://www.youtube.com/watch?v=7_3qbou_keg
    [Text-based] https://aws.amazon.com/blogs/compute/building-an-aws-iot-core-device-using-aws-serverless-and-an-esp32/
  4. Data from AWS IoT Core -> a Lambda function -> S3 bucket:
    https://devopstar.com/2020/05/16/aws-iot-esp32-cam-setup/
*/

/*
  Some pre-configuration guidance:
  1. Set the WiFi UUID & Password and the AWS IoT URL in the secrets.h file
  2. 
*/
/////////////////////////////////////////////////////////////////

/*[Headers - WiFi & MQTT]*/
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include "WiFi.h"

/*[Headers - Camera and Board Selection]*/
#include "esp_camera.h"
//#define CAMERA_MODEL_ESP32S3_EYE  // Has PSRAM
#define CAMERA_MODEL_ESP32S3_KORVO2
#include "camera_pins.h"

/*[Headers - Audio input and output]*/
#include "AudioKitHAL.h"

/*[Headers - Timer & Timer Interrupt]*/
#include <arduino-timer.h>

/*[Macros - MQTT publisher and subscriber identifications]*/
// The MQTT topics that this device should publish/subscribe
#define AWS_IOT_PUBLISH_TOPIC_CAM0   "esp32/cam_0"
#define AWS_IOT_PUBLISH_TOPIC_MIC0   "esp32/mic_0"
#define AWS_IOT_SUBSCRIBE_TOPIC      "esp32/sub"

/*[Macros - Camera Resolution]*/
//FRAMESIZE_VGA-640x480, FRAMESIZE_SVGA-800x600, FRAMESIZE_XGA-1024x768, FRAMESIZE_HD-1280x720
#define CAM_FRAME_SIZE FRAMESIZE_HD

/*[Global variables - Buffer for the Camera and MIC]*/
const uint32_t bufferSize_Cam0 = 160000; // Maximum 32kB for the Image size
const uint16_t bufferSize_Mic0 = 64000; //Frame size of the audio
uint8_t buffer_mic0[bufferSize_Mic0]; //Audio frame buffer

/*[Handlers - MQTT & WiFi client setup]*/
WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(bufferSize_Cam0 + bufferSize_Mic0); 

/*[Handlers - Audio configuration structure]*/
AudioKit kit; //Configure the ES8311 (DAC) Codec and ES7210 (ADC) Encoder
audio_hal_codec_config_t es7210_cfg; //ES7210 (ADC for MIC) initialization config

/*[Handlers - Create a timer object & set the timer period]*/
auto timer = timer_create_default(); // create a timer with default settings
const int timer_period_1 = 2000; // In ms
const int timer_period_2 = 1000;

/*[Function - Connect to WiFi and the MQTT server]*/
void connectAWS()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED){delay(500);Serial.print(".");}

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.begin(AWS_IOT_ENDPOINT, 8883, net);
  client.setCleanSession(true);
  
  // Create a message handler for incoming message
  client.onMessage(messageHandler);

  Serial.println("Connecting to AWS IOT");

  while (!client.connect(THINGNAME)) {Serial.print(".");delay(100);}

  if(!client.connected()){Serial.println("AWS IoT Timeout!");ESP.restart();return;}

  Serial.println("AWS IoT Connected!");

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
}

/*[Function - Initilize the camera]*/
void camInit(void)
{
  Serial.println("Entering Camera Init!");

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = CAM_FRAME_SIZE;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (psramFound()) {Serial.printf("PSRAM Found!\n");
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {Serial.printf("PSRAM NOT Found!\n");
    // Limit the frame size to 480*320 when PSRAM is not available
    config.frame_size = FRAMESIZE_HVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {Serial.printf("Camera init failed with error 0x%x", err);return;}

  //Flip the image to make it normal
  //sensor_t* s = esp_camera_sensor_get();
  //s->set_vflip(s, 1);

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  Serial.println("Camera Init Success!");  
}

/*[Function - Capture an image frame]*/
void grabImage(){
  camera_fb_t * fb = esp_camera_fb_get();

  if(fb != NULL && fb->format == PIXFORMAT_JPEG && fb->len < bufferSize_Cam0){
    bool result = client.publish(AWS_IOT_PUBLISH_TOPIC_CAM0, (const char*)fb->buf, fb->len);

    if(!result){Serial.println("Publishing Image to MQTT server failed! Rebooting the system!");ESP.restart();}
    else Serial.printf("Image Length: %d    Publish Image Successful: %d \n", fb->len, result);
  }
  else Serial.printf("Image size = %d bytes > bufferSize_Cam0 = %d bytes! Frame dropped!\n", fb->len, bufferSize_Cam0);

  esp_camera_fb_return(fb);
  //delay(10);
}

/*[Function - Initilize the Microphone and the Speaker]*/
void Audio_Init()
{
  auto cfg = kit.defaultConfig(AudioInputOutput);
  cfg.adc_input = AUDIO_HAL_ADC_INPUT_ALL; // microphone input
  cfg.sample_rate = AUDIO_HAL_16K_SAMPLES; //How many samples per second

  kit.begin(cfg);
  kit.setVolume(60);
   
  es7210_cfg.i2s_iface.samples = AUDIO_HAL_16K_SAMPLES;
  es7210_adc_init(&es7210_cfg); //Initialize ES7210 Encoder
  es7210_adc_set_gain(GAIN_36DB);//Set MIC gain
  //es7210_adc_set_volume(50);

}

/*[Function - Capture an audio frame]*/
void grabAudio()
{
  size_t len = kit.read(buffer_mic0, bufferSize_Mic0);
  
  bool result = client.publish(AWS_IOT_PUBLISH_TOPIC_MIC0, (const char*)buffer_mic0, len);

  if(!result){Serial.println("Publishing Audio to MQTT server failed! Rebooting the system!");ESP.restart();}
  else Serial.printf("Audio Length: %d    Publish Audio Successful: %d \n", len, result);
  
  //delay(10);
}

/*[Function - MQTT incoming message handler]*/
void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

void setup() {
  Serial.begin(115200);
  
  // Set the timer interval to timer_period_1 second
  timer.every(timer_period_1, timer_callback_1);
  timer.every(timer_period_2, timer_callback_2);

  camInit();
  Audio_Init();
  connectAWS(); 
}

void loop() {
  client.loop();
  timer.tick(); // tick the timer
}

// Timer callback function - triggered every timer_period_1 milliseconds
bool timer_callback_1(void *) {
if(client.connected())grabImage();
    else {Serial.println("AWS IoT Disconnected!");ESP.restart();}
  return true;
}

// Timer callback function - triggered every timer_period_2 milliseconds
bool timer_callback_2(void *) {
  if(client.connected())grabAudio();
    else {Serial.println("AWS IoT Disconnected!");ESP.restart();} 
  return true;
}