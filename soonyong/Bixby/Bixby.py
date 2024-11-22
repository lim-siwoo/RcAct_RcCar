import threading
from speech_text import init, listening
from mqtt.mqtt import MQTT
# from text_speech import Jarvis
from data import get_sentence
from text_speech import tts

if __name__ == '__main__':
    init()

    stop_event = threading.Event()
    listening_thread = threading.Thread(target=listening, args=(stop_event,))
    listening_thread.start()

    mqtt_client = MQTT("127.0.0.1", 1883)
    # mqtt_client.connect()
    
    tts("안녕하세요. 저는 빅스비입니다. 무엇을 도와드릴까요?")
    try:
        while True:
            #get sentence
            sentence = get_sentence()
            if(sentence != ""):
                print(sentence)
            if "빅스비" in sentence:
                try:
                    tts("네, 부르셨어요?")
                    # mqtt_client.publish("soonyong", last_word)
                except Exception as e:
                    # print("MQTT publish failed")
                    print(e)
            
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    except Exception as e:
        print(e)
    mqtt_client.disconnect()
    stop_event.set()
    listening_thread.join()
    tts("안녕히 가세요.")