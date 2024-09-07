import matplotlib.pyplot as plt
from neuropy3.neuropy3 import MindWave
from time import sleep

# Initialize lists to store the last 30 values
beta_alpha = []
alpha_l_values = []
beta_l_values = []
threshold = [1.0] * 30

def update_plot():
    plt.clf()

    #plt.plot(alpha_l_values, label='Alpha low', color='blue')
    #plt.plot(beta_l_values, label='Beta low', color='red')
    plt.plot(threshold, label='threshold', color='red')

    plt.plot(beta_alpha, label='low_beta / low_alpha', color='blue')
    
    plt.xlabel('time')
    plt.ylabel('low_beta / low_alpha')
    plt.title('EEG Data (low_beta / low_alpha)')
    plt.xlim(0, 30)
    plt.ylim(0, 5)
    plt.legend()

    plt.pause(0.01)
    
    if beta_alpha:
        print(beta_alpha[-1])

def process_eeg_data(eeg_data):
    global alpha_l_values, beta_l_values
    alpha_l = eeg_data['alpha_l']
    beta_l = eeg_data['beta_l']

    if alpha_l != 0 and beta_l != 0:
        alpha_l_values.append(alpha_l)
        beta_l_values.append(beta_l)
        
        beta_alpha.append(beta_l/alpha_l)

        if len(alpha_l_values) > 30:
            alpha_l_values.pop(0)
        if len(beta_l_values) > 30:
            beta_l_values.pop(0)
        if len(beta_alpha) > 30:
            beta_alpha.pop(0)

mw = MindWave(address='C4:64:E3:E7:C6:71', autostart=False, verbose=3)
mw.set_callback('eeg', process_eeg_data)
mw.start()

plt.ion()
plt.figure()

try:
    while True:
        sleep(1)
        update_plot()

except KeyboardInterrupt:
    pass

finally:
    mw.unset_callback('signal')
    mw.unset_callback('values')
    mw.unset_callback('eeg')
    mw.stop()
