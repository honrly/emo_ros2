from neuropy3.neuropy3 import MindWave
from time import sleep

def print_eeg_data(eeg_data):
    alpha_l = eeg_data['alpha_l']
    beta_l = eeg_data['beta_l']
    print("Alpha low:", alpha_l)
    print("Beta low:", beta_l)


mw = MindWave(address='C4:64:E3:E7:C6:71', autostart=False, verbose=3)
# mw = MindWave(autostart=False, verbose=3)  # Autoscan for MindWave Mobile
# mw.set_callback('eeg', lambda x: print(x))
# mw.set_callback('values', lambda x: print(x))
mw.set_callback('eeg', print_eeg_data)
mw.start()
try:
    while True:
        sleep(1)

except KeyboardInterrupt:
    pass
finally:
    mw.unset_callback('eeg')
    mw.stop()