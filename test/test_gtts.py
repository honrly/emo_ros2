from gtts import gTTS

message = "Hello! Wellcome to Doly Labo!"
tts = gTTS(message, lang='en') 
tts.save('../Filler_EN/Hello_WellcometoDolyLabo.mp3')
