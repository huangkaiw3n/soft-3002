import os

def say(something):
    os.system('espeak -v+f3 -s100 "{0}" --stdout |aplay'.format(something))

