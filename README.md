# ESP32S3Korvo_AWSIoT

[Summary]

This project is structured as follows:

{camera + mic} -> {MQTT publisher} -> {AWS IoT Core} -> {Lambda function} -> {S3 bucket}

Created by Pei - 2023/04/06

Maintained by - XXX



[Folders]

<esp32s3awsiot/esp32s3aws>

IDE: Arduino IDE 2.0.4

Platform: ESP32-S3-Korvo (URL: https://aws.amazon.com/blogs/compute/building-an-aws-iot-core-device-using-aws-serverless-and-an-esp32/)

Publish topics: "esp32/cam_0", "esp32/mic_0"

Subscribe topic: "esp32/sub"

Setup tutorial: (URL: https://aws.amazon.com/blogs/compute/building-an-aws-iot-core-device-using-aws-serverless-and-an-esp32/)

