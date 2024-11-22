last_word = ""
update_flag = False

def update_last_word(word):
    global last_word, update_flag
    update_flag = True
    last_word = word

def get_update_flag():
    global update_flag
    return update_flag

def get_last_word():
    global last_word, update_flag
    update_flag = False
    return last_word