from scipy import signal
import matplotlib.pyplot as plt
import numpy as np

#Calculate filter parameters for digital butterworth filters
n = 2
wn = 2
fs = 50
Ftype = 'low'

b, a = signal.butter(n, 2, Ftype, fs=fs)
print('A: ', end='')
print(a)

print('B: ', end='')
print(b)

w, h = signal.freqs(b, a)
plt.semilogx(w, 20 * np.log10(abs(h)))
plt.title('Butterworth filter frequency response')
plt.xlabel('Frequency [rad/s]')
plt.ylabel('Amplitude [dB]')
plt.margins(0, 0.1)
plt.grid(which='both', axis='both')
plt.axvline(wn, color='green') # cutoff frequency
plt.show()