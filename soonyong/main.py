import threading
from transcribe_streaming_mic import listening
from mqtt import soonyong_mqtt
from data import get_update_flag, get_last_sentence

if __name__ == '__main__':
    stop_event = threading.Event()
    stop_event.clear()
    listening_thread = threading.Thread(target=listening, args=(stop_event,))
    listening_thread.start()
    
    mqtt_client = soonyong_mqtt("127.0.0.1", 1883)
    mqtt_client.connect()
    try:
        while True:
            if get_update_flag():
                last_sentence = get_last_sentence()
                print(last_sentence)
                try:
                    mqtt_client.publish("soonyong", last_sentence)
                except:
                    print("MQTT publish failed")
            
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        stop_event.set()
        # print("wating for listening_thread to join")
        listening_thread.join()
        # print("listening_thread joined")
        mqtt_client.disconnect()
        # print("mqtt_client disconnected")