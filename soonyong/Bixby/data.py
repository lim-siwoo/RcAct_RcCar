import time

sentence = ""
timeout = 1
last_update = 0
last_report = ""

def update_word(word):
    global sentence, last_update
    sentence += (" " + word)
    last_update = time.time()
    
# def update_sentence(new_sentence):
#     global sentence, last_update
#     sentence = new_sentence
#     last_update = time.time()

def check_update_finish():
    global last_update, timeout
    if((time.time() - last_update) > timeout):
        return True
    return False

def get_sentence():
    global sentence
    if(check_update_finish()):
        return_sentence = sentence
        sentence = ""
        return return_sentence
    return ""

def update_report(report):
    global last_report
    last_report = report
    
def get_report():
    global last_report
    return last_report