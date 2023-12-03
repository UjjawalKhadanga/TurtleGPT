#!/usr/bin/env python3

import rospy
from speech_recognition import Recognizer, Microphone
from std_msgs.msg import String
from turtle_gpt.srv import audioToText

def speech_to_text_callback(req):
    # Speech-to-text conversion logic
    recognizer = Recognizer()
    microphone = Microphone()

    with microphone as source:
        print("Say something...")
        audio_data = recognizer.listen(source)

    try:
        text_result = recognizer.recognize_google(audio_data)
        print("Text from Speech-to-Text:", text_result)
        return text_result
    except recognizer.UnknownValueError:
        print("Speech-to-Text could not understand audio")
        return ""
    except recognizer.RequestError as e:
        print("Could not request results from Speech-to-Text service; {0}".format(e))
        return ""

def speech_to_text_node():
    rospy.init_node('speech_to_text_node', anonymous=True)
    rospy.Service('/speech_to_text_conversion', audioToText, speech_to_text_callback)
    print("Speech-to-Text Node is ready.")
    rospy.spin()

if __name__ == '__main__':
    speech_to_text_node()