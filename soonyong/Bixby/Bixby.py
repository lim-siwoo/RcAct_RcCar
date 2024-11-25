import threading
from speech_text import init, listening
from mqtt import MQTT
from data import get_sentence
from text_speech import tts
import time

ip = "70.12.229.60"
port = 1883
pub_topics = {
    "command" : "iot/bixby/command",
    "report" : "iot/bixby/report",
    "pose" : "iot/rccar/pose"
    }
sub_topics = {
    "pose" : "iot/rccar/pose",
    "report" : "iot/gpt/report"
    }

def remove_whitespace_and_newlines(input_str):
    return input_str.replace(" ", "").replace("\n", "")

if __name__ == '__main__':
    init()

    stop_event = threading.Event()
    listening_thread = threading.Thread(target=listening, args=(stop_event,))
    listening_thread.start()

    mqtt_client = MQTT(ip, port)
    try:
        mqtt_client.connect()
        for topic in sub_topics.values():
            mqtt_client.subscribe(topic)
    except Exception as e:
        print(e)
        tts("MQTT 서버에 연결할 수 없습니다. 프로그램을 종료합니다.")
        stop_event.set()
        listening_thread.join()
        exit()
    
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
                if "따라" in sentence:
                    tts("네, 따라갈게요.")
                    called_flag = False
                    mqtt_client.publish(pub_topics["command"], "follow")
                elif "돌아" in sentence:
                    tts("네, 개인기를 보여드릴게요.")
                    called_flag = False
                    mqtt_client.publish(pub_topics["command"], "turn")
                elif "멈춰" in sentence:
                    tts("네, 멈출게요.")
                    called_flag = False
                    mqtt_client.publish(pub_topics["command"], "stop")
                elif "목록" in sentence or "리스트" in sentence or "뭐할수있" in sentence:
                    tts("제가 할 수 있는 일은 따라와, 돌아, 멈춰 입니다.")
                    called_flag = False
                elif "보고" in sentence or "상태" in sentence or "요약" in sentence or "로그" in sentence:
                    mqtt_client.publish(pub_topics["report"], "report")
                    tts("로그를 요청했어요.")
                    tts("지금까지의 로그를 요약해드릴게요.")
                    report = mqtt_client.get_message(sub_topics["report"])
                    print(report)
                    if(report == ""):
                        tts("아직 로그가 없어요.")
                    else:
                        tts(str(report))
                    called_flag = False
            elif called_flag:
                tts("하고픈 일이 있다면 다시 불러주세요.")
                called_flag = False
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    except Exception as e:
        print(e)
        
    tts("안녕히 가세요.")
    mqtt_client.disconnect()
    stop_event.set()
    listening_thread.join()