last_sentence = ""
update_flag = False

def update_last_sentence(sentence):
    global last_sentence, update_flag
    update_flag = True
    last_sentence = sentence

def get_update_flag():
    global update_flag
    return update_flag

def get_last_sentence():
    global last_sentence, update_flag
    update_flag = False
    return last_sentence