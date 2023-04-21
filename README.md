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

Function: Capture images and audio from on-board camera sensor and microphone -> Publish the images and audio to the AWS IoT Core

Setup tutorial: (URL: https://aws.amazon.com/blogs/compute/building-an-aws-iot-core-device-using-aws-serverless-and-an-esp32/)

Arduino Library Installation: 

[ESP32-S3 Board] (https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)
[ESP32-S3_Korvo Audio Kit] (https://github.com/pschatzmann/arduino-audiokit)

<aws_lambda>

IDE: Python 3.7

Platform: AWS Lambda

Function: Subscribe the images from the AWS IoT Core and convert them into video files and store them in the S3 bucket "esp32s3s3bucket"

Setup tutorial: (URL: https://devopstar.com/2020/05/16/aws-iot-esp32-cam-setup/)
