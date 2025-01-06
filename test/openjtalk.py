#coding: utf-8
import subprocess
from datetime import datetime

text = 'こんにちは'
emotion = 'normal' # normal, happy, angry, sad, bashful
def jtalk(t):
    open_jtalk=['open_jtalk']
    mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
    htsvoice=['-m',f'/usr/share/hts-voice/mei/mei_{emotion}.htsvoice']
    speed=['-r','0.7']
    volume=['-g','4.0']
    outwav=['-ow',f'{text}_{emotion}.wav']
    cmd=open_jtalk+mech+htsvoice+speed+volume+outwav
    c = subprocess.Popen(cmd,stdin=subprocess.PIPE)
    c.stdin.write(t.encode())
    c.stdin.close()
    c.wait()
    aplay = ['aplay','-q',f'{text}_{emotion}.wav']
    wr = subprocess.Popen(aplay)
    
def say_datetime():
    jtalk(text)

if __name__ == '__main__':
    say_datetime()

