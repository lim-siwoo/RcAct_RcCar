import threading
from speech_text import init, listening
from mqtt.mqtt import MQTT
from data import get_sentence
from text_speech import tts
import time

def remove_whitespace_and_newlines(input_str):
    return input_str.replace(" ", "").replace("\n", "")

if __name__ == '__main__':
    init()

    stop_event = threading.Event()
    listening_thread = threading.Thread(target=listening, args=(stop_event,))
    listening_thread.start()

    mqtt_client = MQTT("127.0.0.1", 1883)
    # mqtt_client.connect()
    
    tts("안녕하세요. 저는 빅스비입니다. 무엇을 도와드릴까요?")
    try:
        called_time = 0
        called_timeout = 10
        called_flag = False
        while True:
            #get sentence
            sentence = remove_whitespace_and_newlines(get_sentence())
            if(sentence != ""):
                print(sentence)
            
            if "빅스비" in sentence:
                tts("네, 부르셨어요?")
                called_time = time.time()
                called_flag = True
            
            if((time.time() - called_time) <= called_timeout):
                if "따라와" in sentence:
                    tts("네, 따라갈게요.")
                    called_flag = False
                    mqtt_client.publish("command/", "follow")
                elif "돌아" in sentence:
                    tts("네, 개인기를 보여드릴게요.")
                    called_flag = False
                    mqtt_client.publish("command/", "turn")
                elif "멈춰" in sentence:
                    tts("네, 멈출게요.")
                    called_flag = False
                    mqtt_client.publish("command/", "stop")
                elif "목록" in sentence or "리스트" in sentence or "뭐할수있어" in sentence:
                    tts("제가 할 수 있는 일은 따라와, 돌아, 멈춰 입니다.")
                    called_flag = False
            elif called_flag:
                tts("하고픈 일이 있다면 다시 불러주세요.")
                called_flag = False
                
    tts("안녕히 가세요.")
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        tts("안녕히 가세요.")
    except Exception as e:
        print(e)
        tts("안녕히 가세요.")
    mqtt_client.disconnect()
    stop_event.set()
    listening_thread.join()