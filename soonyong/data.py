last_sentence = None
update_flag = False

def update_last_sentence(sentence):
    global last_sentence, update_flag
    last_sentence = sentence
    update_flag = True
    
def get_update_flag():
    return update_flag

def get_last_sentence():
    global last_sentence, update_flag
    update_flag = False
    return last_sentence