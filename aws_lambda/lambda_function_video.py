import io
import json
import base64
import os
import uuid
import boto3
import cv2
import glob

from PIL import Image
import numpy as np
from datetime import datetime, timedelta

max_frame_per_video = 20
image_frame_count = 0
frameSize = (480, 320)
video_fps = 10
img_array = []

def lambda_handler(event, context):
    #Convert image from base64 payload
    imageStreamBytes = base64.b64decode(event['data_cam0'])
    jpg_as_np = np.frombuffer(imageStreamBytes, dtype=np.uint8)
    image_numpy = cv2.imdecode(jpg_as_np, flags=cv2.IMREAD_COLOR)
    
    global img_array
    img_array.append(image_numpy)
    
    global image_frame_count
    image_frame_count = image_frame_count + 1
    print("image_frame_count: " + str(image_frame_count))
    
    if image_frame_count == max_frame_per_video :
        timestamp = (datetime.utcnow() - timedelta(hours=4)).strftime('%Y%m%d%H%M%S')
        video_file_name = str(timestamp) + '.avi'
        video_file_path = '/tmp/' + video_file_name
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        video_out = cv2.VideoWriter(video_file_path, fourcc, video_fps, frameSize)
        
        for i in range(len(img_array)):
            video_out.write(img_array[i])
            
        video_out.release()
        img_array = []
        image_frame_count = 0
        
        s3 = boto3.client('s3')
        s3.upload_file(video_file_path, 'esp32s3s3bucket', 'cam0_video/{}'.format(video_file_name))
    
        print("Video Generated! " + video_file_name)
    else :
        print("Frame " + str(image_frame_count) + " of " + str(max_frame_per_video))
            
    return 1
