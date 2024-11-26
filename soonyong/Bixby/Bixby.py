import threading
from speech_text import init, listening
from mqtt import MQTT
from data import get_sentence
from text_speech import tts
import time

ip = "70.12.229.60"
port = 1883
my_name = "아스라다"
pub_topics = {
    "command" : "iot/bixby/command",
    "report" : "iot/bixby/report",
    "pose" : "iot/bixby/pose",
    "chat" : "iot/bixby/chat"
    }
sub_topics = {
    "pose" : "iot/rccar/pose",
    "report" : "iot/gpt/report",
    "chat" : "iot/gpt/chat"
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
    
    tts(f"안녕하세요. 저는 {my_name}입니다. 무엇을 도와드릴까요?")
    try:
        called_time = 0
        called_timeout = 10
        called_flag = False
        while True:
            #get sentence
            sentence = remove_whitespace_and_newlines(get_sentence())
            if(sentence != ""):
                print(sentence)
            
            if (my_name in sentence or "아슬하다" in sentence or "아수라다" in sentence) and not called_flag:
                tts("네, 부르셨어요?")
                called_time = time.time()
                called_flag = True
            
            elif((time.time() - called_time) <= called_timeout):
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
                        tts(f"{str(report)}")
                    called_flag = False
                elif "포즈" in sentence or "자세" in sentence:
                    mqtt_client.publish(pub_topics["pose"], "pose")
                    tts("한번 들여다볼게요.")
                    pose = mqtt_client.get_message(sub_topics["pose"])
                    print(pose)
                    if(pose == "" or pose == "UNKNOWN"):
                        tts("무슨 자세인지 잘 모르겠어요.")
                    elif pose == "NO PERSON":
                        tts("사람이 없어요.")
                    else:
                        tts("지금 보고있는 사람의 자세를 알려드릴게요.")
                        tts(f"{str(pose)} 자세를 하고 계시네요.")
                    called_flag = False
                elif "갈게" in sentence or "바이" in sentence or "잘가" in sentence or "잘있어" in sentence:
                    break
                elif len(sentence) >= 3:
                    mqtt_client.publish(pub_topics["chat"], sentence)
                    tts("말씀하신 내용을 이해중이에요. 잠시만 기다려주세요.")
                    chat = mqtt_client.get_message(sub_topics["chat"])
                    print(str(chat))
                    if(chat == ""):
                        tts("죄송해요. 이해하지 못했어요.")
                    else:
                        tts(str(chat))
                    called_flag = False
                    
            elif called_flag:
                tts("하고픈 일이 있다면 다시 불러주세요.")
                called_flag = False
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    except Exception as e:
        print(e)
        
    mqtt_client.publish(pub_topics["command"], "stop")
    tts("안녕히 가세요.")
    mqtt_client.disconnect()
    stop_event.set()
    listening_thread.join()