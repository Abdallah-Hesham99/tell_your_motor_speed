#!/usr/bin/env python3
# coding: utf-8



import numpy as np
#import tensorflow as tf
#from tensorflow.keras.models import load_model
import sounddevice as sd
from scipy.io.wavfile import write,read
from scipy.io import wavfile
from scipy.signal import stft
import cv2
from std_msgs.msg import UInt16
import rospy
from tflite_runtime.interpreter import Interpreter

TFLITE_FILE_PATH = '/home/abdallah/Machine_Learning/tf_lite_speech_model/model.tflite'

PREPROCESSOR_FILE_PATH = '/home/abdallah/Machine_Learning/tf_lite_speech_model/preprocess.tflite'

#model = load_model('/home/abdallah/Machine Learning/speech_model')
interpreter = Interpreter(TFLITE_FILE_PATH)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()


img_recording = cv2.imread('/home/abdallah/Downloads/images.jpeg')
img_ready =  cv2.imread('/home/abdallah/Downloads/index.png')
print('Recodding after 1 seconds')
cv2.imshow('get_ready',img_ready)
cv2.waitKey(1000)

cv2.destroyAllWindows()

fs = 16000  # Sample rate
seconds = 1  # Duration of recording
cv2.imshow('recording',img_recording)

myrecording = sd.rec(int(seconds * fs),dtype=np.int16, samplerate=fs, channels=1,blocking=False)
cv2.waitKey(1000)
#sd.wait()  # Wait until recording is finished
cv2.destroyAllWindows()

write('output.wav', fs, myrecording)  # Save as WAV file 





path = 'output.wav'



"""

binary= tf.io.read_file(path)
audio,_ = tf.audio.decode_wav(binary)
print(_)
audio = tf.squeeze(audio,-1)
zero_padding = tf.zeros([16000] - tf.shape(audio), dtype=tf.float32)
audio = tf.cast(audio, tf.float32)
equal_length = tf.concat([audio, zero_padding], 0)
spectrogram = tf.signal.stft(
equal_length, frame_length=255, frame_step=128)
spectrogram = tf.abs(spectrogram)
spectrogram = tf.expand_dims(spectrogram, -1)

"""





com = [8,5,4,9,1,7,6,10,3,2,0]   # This order of digits is what the model was trained on



def predict(path,comand_dict):
    '''
    binary= tf.io.read_file(path)
    audio,_ = tf.audio.decode_wav(binary)
    audio = tf.squeeze(audio,-1)
    '''
    '''
    zero_padding = tf.zeros([16000] - tf.shape(audio), dtype=tf.float32)
    audio = tf.cast(audio, tf.float32)
    equal_length = tf.concat([audio, zero_padding], 0)
    spectrogram = tf.signal.stft(
      equal_length, frame_length=255, frame_step=128)
    spectrogram = tf.abs(spectrogram)
    spectrogram = tf.expand_dims(spectrogram, -1)

    '''
    samplerate, data = wavfile.read(path)
    PREPROCESSOR_FILE_PATH = '/home/abdallah/Machine_Learning/tf_lite_speech_model/preprocess.tflite'

	
    pre_interpreter = Interpreter(PREPROCESSOR_FILE_PATH)
    pre_interpreter.allocate_tensors()

    pre_input_details = pre_interpreter.get_input_details()
    pre_output_details = pre_interpreter.get_output_details()

    pre_interpreter.set_tensor(pre_input_details[0]['index'], data.astype(np.float32))

    pre_interpreter.invoke()
    output_data = pre_interpreter.get_tensor(pre_output_details[0]['index']) 

    
    '''
    zero_padding = np.zeros([16000] - data.shape, dtype=np.float32)
    audio = np.cast['float32'](audio)
    equal_length = np.concatenate((audio, zero_padding), 0)
#    spectrogram = tf.signal.stft(
 #     equal_length, frame_length=255, frame_step=128)
      
    spectrogram = stft(
      equal_length, frame_length=255, frame_step=128)
      
      
      
      
    spectrogram = np.abs(spectrogram)
    spectrogram = np.expand_dims(spectrogram, -1)
    '''
#    pred = np.argmax(model.predict(spectrogram[tf.newaxis,...]),axis=1)
    interpreter.set_tensor(input_details[0]['index'], output_data.reshape(1,124,129,1))
#    out = my_signature(spectrogram[tf.newaxis,...])
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index']) 
    pred = np.argmax(output_data,axis=1)
    msg = comand_dict[pred[0]]
    print(msg)
    
    return msg

    '''
#predict(path,model,com)      #just for testing
    '''

rospy.init_node("speed_node")
pub = rospy.Publisher("Change_your_speed",UInt16,queue_size=10)
rate = rospy.Rate(20)
speed_msg = UInt16()
speed_msg.data = predict(path,com)
#speed_msg.linear.x =5
while not rospy.is_shutdown():
    pub.publish(speed_msg)
    rate.sleep()





