/*[Headers - WiFi & MQTT]*/
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include "WiFi.h"

/*[Headers -BLE]*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/*[Headers - SD/MMC Card & File System]*/
#include "SD_MMC.h"
#include "FS.h"

/*[Headers - Camera and Board Selection]*/
#include "esp_camera.h"
#define CAMERA_MODEL_ESP32S3_KORVO2
#include "camera_pins.h"

/*[Headers - Face Detection]*/
#include "human_face_detect_msr01.hpp"

/*[Headers - Audio input and output]*/
#include "AudioKitHAL.h"

/*[Headers - System Timer for MIC Sampling]*/
#include <arduino-timer.h>

/*[GPIO Setup - Infrared Sensor Input GPIO & Interrupt Setup]*/
const int Infrared_Pin = 1;
//variables to keep track of the timing of recent gpio interrupts for the infrared detection
unsigned int last_infrared_time = 0;
unsigned int infrared_timeout_ms = 10000;

unsigned int last_face_detected_time = 0;
unsigned int face_detected_timeout_ms = 30000;

/*[Global Variables - Global flag]*/
uint8_t global_flag = 0;//"0": reset, "1": set. [*] is bit *. [7] [6] [5] [4] [3] [2] [1]Is a human face [0]Motion detected
bool deviceConnected = false, previousConnected = false; // Indicate the status of the ble connection
bool motion_speech = false, ble_advertising = false; // Do action only upon status change
bool do_audio_sampling = true; // Flag for enabling audio sampling

/*[Global variables - Buffer for the Camera and MIC]*/
const uint32_t bufferSize_Cam0 = 100000; // Frame size of the video

uint8_t audio_sampling_period_ms = 100; //Aduio sampling time period
// Frame size of the audio [sample rate = 8000 / s, 16 bits / sample, 2 channels -> 3200 bytes / 100 ms]
const uint16_t bufferSize_Mic0 = 8000 * 2 * 2 / 10; 
uint8_t buffer_mic0[bufferSize_Mic0]; //Audio frame buffer

/*[Macros - Camera Resolution]*/
//FRAMESIZE_VGA-640x480, FRAMESIZE_SVGA-800x600, FRAMESIZE_XGA-1024x768, FRAMESIZE_HD-1280x720
#define CAM_FRAME_SIZE FRAMESIZE_HD

/*[Macros - MQTT publisher and subscriber identifications]*/
// The MQTT topics that this device should publish/subscribe
#define AWS_IOT_PUBLISH_TOPIC_CAM0   "esp32/cam_0"
#define AWS_IOT_PUBLISH_TOPIC_MIC0   "esp32/mic_0"
#define AWS_IOT_PUBLISH_TOPIC_BLE0   "esp32/ble_0"
#define AWS_IOT_SUBSCRIBE_TOPIC      "esp32/sub"

/*[Macros - BLE Advertised Name and Service & Characteristics]*/
#define ADV_NAME             "SmartDoorBell"
#define BLE_KEY_FOR_DEMO     "ITE-351"
#define SERVICE_UUID         "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID1 "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID2 "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID3 "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"

/*[Macros - Define the 1-wire SD GPIO port]*/
#define SD_CLK   15
#define SD_CMD   7
#define SD_DATA0 4

#define PATH_VOICE_MOTION "/TTS_Voice/motiondetected.wav"
#define PATH_VOICE_FACE_CAM "/TTS_Voice/facethecam.wav"
#define PATH_VOICE_IS_HUMAN "/TTS_Voice/youareahuman.wav"
#define PATH_VOICE_TIMEOUT "/TTS_Voice/timeout.wav"
#define PATH_VOICE_REBOOT "/TTS_Voice/rebooting.wav"
#define PATH_VOICE_AWS_INIT_BEGIN "/TTS_Voice/aws_init_begin.wav"
#define PATH_VOICE_AWS_INIT_FAIL "/TTS_Voice/aws_init_failed.wav"
#define PATH_VOICE_AWS_INIT_OK "/TTS_Voice/aws_init_ok.wav"
#define PATH_VOICE_AWS_CONN_LOST "/TTS_Voice/aws_conn_lost.wav"
#define PATH_VOICE_IDCONFIRMED "/TTS_Voice/idconfirmed.wav"

/*[Handlers - MQTT & WiFi client setup]*/
WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(bufferSize_Cam0 + bufferSize_Mic0 + 1024);

/*[Handlers - Audio configuration structure]*/
AudioKit kit; //Configure the ES8311 (DAC) Codec and ES7210 (ADC) Encoder
audio_hal_codec_config_t es7210_cfg; //ES7210 (ADC for MIC) initialization config

/*[Handlers - BLE Service and Characteristics]*/
BLEServer* pServer;
BLEService* pService;
BLECharacteristic* pCharacteristic1;
BLECharacteristic* pCharacteristic2;
BLECharacteristic* pCharacteristic3;

uint8_t characteristic1Data = 0;
uint8_t characteristic2Data = 0;
uint8_t characteristic3Data = 0;

/*[Handlers - create a timer with default settings]*/
auto timer = timer_create_default();

void setup() {
  Serial.begin(115200);
  
  Timer_Init(audio_sampling_period_ms);
  Infrared_Init();
  SDMMC_Init(); 
  Audio_Init();    
  Camera_Init();

  BLE_Init();
  AWS_Init();
}

void loop() {
  client.loop();//keep the MQTT client running
  timer.tick(); // tick the timer

  BLE_Status_Update();
  Flag_Update_And_Action();
}

/*--------------------------------------------------*/
/*---------------------- Interrupt Service Routine ---------------------*/
/*--------------------------------------------------*/
void IRAM_ATTR isr_infrared() 
{
  last_infrared_time = millis();
  global_flag = global_flag | (1<<0);
}

/*[Function - MQTT incoming message handler]*/
void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

/*--------------------------------------------------*/
/*---------------------- Functions ---------------------*/
/*--------------------------------------------------*/
/*[Function - Initilize the timer]*/
void Timer_Init(uint16_t time_ms)
{
  timer.every(time_ms, Push_Audio_to_AWS);
}

/*[Function - Initilize the infrared sensor input pin and interrupt]*/
void Infrared_Init(void)
{
  pinMode(Infrared_Pin, INPUT_PULLDOWN);
  attachInterrupt(Infrared_Pin, isr_infrared, RISING);
}

/*[Function - Initilize the SD/MMC card] */
void SDMMC_Init(void)
{
  SD_MMC.setPins(SD_CLK, SD_CMD, SD_DATA0);
  if(!SD_MMC.begin("/sdcard",true)){Serial.println("Card Mount Failed!");return;}
  else {Serial.println("Card Mount Successful!");}
}

/*[Function - Connect to WiFi and the MQTT server]*/
void AWS_Init(void)
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

  if(!client.connected()){
    Serial.println("AWS IoT Timeout!");
    ESP.restart();
    return;
  }

  Serial.println("AWS IoT Connected!");

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
}

/*[Function - BLE Server Callback Function]*/
class MyBLEServerCallbacks: public BLEServerCallbacks 
{
    void onConnect(BLEServer* pServer) {deviceConnected = true;}
    void onDisconnect(BLEServer* pServer) {deviceConnected = false;}
};

/*[Function - BLE Characteristic Callback Function]*/
class MyBLECallbacksChar1: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristic) 
    {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.printf("Received Value with length [%d]: ", rxValue.length());
        for (int i = 0; i < rxValue.length(); i++)Serial.print(rxValue[i]);
        Serial.println();

        const char * rxValueTemp = rxValue.c_str();
        bool result = client.publish(AWS_IOT_PUBLISH_TOPIC_BLE0, rxValueTemp, rxValue.length());
        if(!result)
        {
          Serial.println("Publishing BLE information to MQTT server failed! Rebooting the system!");
          Play_WAV_from_SD(PATH_VOICE_REBOOT);
          ESP.restart();
        }

        int ble_key_match = strcmp(rxValueTemp, BLE_KEY_FOR_DEMO);
        Serial.printf("rxValueTemp: %s\n", rxValueTemp);
        Serial.printf("ble_key_match = %d\n", ble_key_match);
        if (ble_key_match == 0)Serial.println("Identification confirmed! Starting the next operation!");
        else Serial.println("Identification NOT confirmed! Please send your ID again!");
      }
    }
};

/*[Function - Initialize the BLE]*/
void BLE_Init(void)
{
  Serial.println("Initializing the BLE Service.");

  BLEDevice::init(ADV_NAME);// Create the BLE Device
  pServer = BLEDevice::createServer();// Create the BLE Server
  pServer -> setCallbacks(new MyBLEServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);// Create the BLE Service
  
  Serial.printf("Initialization - BLE MTU: %d\n",BLEDevice::getMTU());
  // Create the first characteristic
  pCharacteristic1 = pService->createCharacteristic(CHARACTERISTIC_UUID1, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic1 -> setCallbacks(new MyBLECallbacksChar1());

  // Create the second characteristic
  pCharacteristic2 = pService->createCharacteristic(CHARACTERISTIC_UUID2, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic3 = pService->createCharacteristic(CHARACTERISTIC_UUID3, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  // Set the initial values for the characteristics
  pCharacteristic1->setValue(&characteristic1Data, 1);
  pCharacteristic2->setValue(&characteristic2Data, 1);
  pCharacteristic3->setValue(&characteristic3Data, 1);
 
  //pService->start();// Start the service
  
  //pServer->getAdvertising()->start();// Start advertising
  //Serial.print("BLE \"");Serial.print(ADV_NAME);Serial.println("\" is advertising!");
  Serial.println("BLE Service Initialized!");
}

/*[Function - Start/Stop BLE Service & Advertising]*/
void BLE_Start_Advertising(bool command)
{
  if(command == true){ pService->start(); pServer->getAdvertising()->start(); }
  else { pServer->getAdvertising()->stop(); pService->stop(); }
}

/*[Function - BLE Action in the main() loop]*/
void BLE_Status_Update(void)
{
  if (deviceConnected) {
        //pTxCharacteristic->setValue(&txValue, 1);
        //pTxCharacteristic->notify();
	}

  // disconnecting
  if (!deviceConnected && previousConnected) 
  {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      previousConnected = deviceConnected;
  }
  
  // connecting
  if (deviceConnected && !previousConnected) {previousConnected = deviceConnected;/*do stuff here on connecting*/}
}

/*[Function - Initilize the camera]*/
void Camera_Init(void)
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
  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    // Limit the frame size to 480*320 when PSRAM is not available
    config.frame_size = FRAMESIZE_HVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {Serial.printf("Camera init failed with error 0x%x", err);return;}
  Serial.println("Camera Init Success!");
}

/*[Function - Capture an image frame and push it to the AWS IoT]*/
void Push_Image_to_AWS(void)
{ 
  camera_fb_t * fb = esp_camera_fb_get();

  if(fb != NULL && fb->format == PIXFORMAT_JPEG && fb->len < bufferSize_Cam0)
  {
    bool result = client.publish(AWS_IOT_PUBLISH_TOPIC_CAM0, (const char*)fb->buf, fb->len);
    if(!result)
    {
      Serial.println("Publishing Image to MQTT server failed! Rebooting the system!");
      Play_WAV_from_SD(PATH_VOICE_REBOOT);
      ESP.restart();
    }
    else Serial.printf("Image Length: %d    Publish Image Successful: %d \n", fb->len, result);
  }
  else Serial.printf("Image size = %d bytes > bufferSize_Cam0 = %d bytes! Frame dropped!\n", fb->len, bufferSize_Cam0);

  esp_camera_fb_return(fb);
}

/*[Function - Initilize the Microphone and the Speaker]*/
void Audio_Init(void)
{
  auto cfg = kit.defaultConfig(AudioInputOutput);
  cfg.adc_input = AUDIO_HAL_ADC_INPUT_ALL; // microphone input
  cfg.sample_rate = AUDIO_HAL_08K_SAMPLES; //How many samples per second

  kit.begin(cfg);
  kit.setVolume(80);
   
  es7210_cfg.i2s_iface.samples = AUDIO_HAL_08K_SAMPLES;
  es7210_adc_init(&es7210_cfg); //Initialize ES7210 Encoder
  es7210_adc_set_gain(GAIN_36DB);//Set MIC gain
  //es7210_adc_set_volume(80);
}

/*[Function - Capture an audio frame and push it to the AWS IoT]*/
bool Push_Audio_to_AWS(void *)
{
  if(do_audio_sampling == true)
  {
    size_t len = kit.read(buffer_mic0, bufferSize_Mic0);  
    bool result = client.publish(AWS_IOT_PUBLISH_TOPIC_MIC0, (const char*)buffer_mic0, len);

    if(!result)
    {
      Serial.println("Publishing Audio to MQTT server failed! Rebooting the system!");
      Play_WAV_from_SD(PATH_VOICE_REBOOT);
      return false;
      ESP.restart();
    }
    else 
    {
      //Serial.printf("Audio Length: %d    Publish Audio Successful: %d \n", len, result);
      return true;
    }
  }
  return true;
}

/*[Function - Play .wav file from SD Card]*/
void Play_WAV_from_SD(const char * wav_path)
{
  // Read the WAV file
  File audioFile = SD_MMC.open(wav_path);
  if (!audioFile) {Serial.printf("Failed to open %s\n", wav_path);return;}

  while (audioFile.available()) {
    uint8_t buffer[128];
    int bytesRead = audioFile.read(buffer, sizeof(buffer));
    kit.write(buffer, bytesRead);
  }
 
  audioFile.close();// Close the file
}

/*[Function - Do face detection and return the result: -1: failed ; 0: no face detected ; 1: face detected]*/
int Face_Detect(void)
{
  camera_fb_t *fb = esp_camera_fb_get();
  if(fb != NULL && fb->len < bufferSize_Cam0){Serial.printf("Image Length [%d * %d]: %d bytes\n",fb->width, fb->height, fb->len);}
  else{Serial.printf("Camera capture failed! Length: %d\n", fb->len);return -1;}

  if (fb->format != PIXFORMAT_RGB565)
  {
    size_t out_len, out_width, out_height;
    uint8_t *out_buf;
    bool s;

    out_len = fb->width * fb->height * 3;
    out_width = fb->width;
    out_height = fb->height;

    out_buf = (uint8_t*)malloc(out_len);
    if (!out_buf) {log_e("out_buf malloc failed");return -1;}

    s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);

    esp_camera_fb_return(fb);
    if (!s) {free(out_buf);log_e("To rgb888 failed");return -1;}

    HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
    std::list<dl::detect::result_t> &results = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});

    free(out_buf);

    if (results.size() > 0)return 1;
    else return 0;
  }
  else 
  {
    HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
    std::list<dl::detect::result_t> &results = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
    
    if (results.size() > 0)return 1;
    else return 0;
    esp_camera_fb_return(fb);
  }

  return -1;
}

void Flag_Update_And_Action(void)
{ 
  if( global_flag & (1<<0) ) // See if a motion is detected
  {
    if( !(global_flag & (1<<1)) ) // See if a "face-detected" action is on-going. If is, then ignore this motion detected action.
    {
      if( (millis() - last_infrared_time) < infrared_timeout_ms )
      {
        if(motion_speech == false)
        {
          Serial.println("Motion Detected! Please face the camera.");
          Play_WAV_from_SD(PATH_VOICE_MOTION);
          Play_WAV_from_SD(PATH_VOICE_FACE_CAM);
          motion_speech = true;
        }
        
        int face_detected = Face_Detect();

        if (face_detected == 1)
        {
          last_face_detected_time = millis();
          global_flag = global_flag | (1<<1);

          Serial.printf("[ *** Human Face Detected *** ]\n");
          Play_WAV_from_SD(PATH_VOICE_IS_HUMAN);
        }
        else if(face_detected == 0) Serial.printf("NO Human Face Detected!\n");
        else if(face_detected == -1)Serial.printf("Error occured when capturing the picture!\n", face_detected);
        
        delay(1000);
      }
      else {
        global_flag = global_flag & ~(1<<0);
        motion_speech = false;
        Serial.println("Infrared timeout! Reset the flag!");
        Play_WAV_from_SD(PATH_VOICE_TIMEOUT);
      }
    }
  }

  if( global_flag & (1<<1) )
  {
    if( (millis() - last_face_detected_time) < face_detected_timeout_ms )
    {
      if(ble_advertising == false){BLE_Start_Advertising(true);ble_advertising = true;}
    }
    else {
      global_flag = global_flag & ~(1<<0);
      global_flag = global_flag & ~(1<<1);
      motion_speech = false;
      //ble_advertising = false;
      Serial.println("\nFace detected timeout! Reset the flag!\n");
      Play_WAV_from_SD(PATH_VOICE_TIMEOUT);
    }
  }
}