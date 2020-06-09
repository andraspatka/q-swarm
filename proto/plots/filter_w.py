import numpy as np
import utils

from scipy.signal import kaiserord, lfilter, firwin, freqz, remez


import matplotlib.pyplot as plt
import scipy

def fourierSpectrum(x, N):
    sp = scipy.fft.fft(x, n=N)
    sp = np.sqrt(sp.imag ** 2 + sp.real ** 2)
    return sp

stop_hz = 4.5
stop_width_hz = 0.3
cut_hz = 4.5

fs = 10
nyq_rate = fs / 2.0
width_bs = 0.4 / nyq_rate
ripple_db = 120.0

wv = np.genfromtxt(utils.createPathToLogs('w/follow2.csv'), delimiter=',')
w = np.transpose(wv)[0]
v = np.transpose(wv)[1]
N = len(w)
t = np.linspace(0, N, N)
f = np.linspace(0, fs, N)

# Use firwin with a Kaiser window to create a lowpass FIR filter.
N_taps, beta = kaiserord(ripple_db, width_bs)
print(N_taps)
#N_taps = 8
taps = firwin(N_taps, cut_hz / nyq_rate, window=('kaiser', beta))

print(repr(taps))
#taps = firwin(N_taps, [(stop_hz - stop_width_hz) / nyq_rate, (stop_hz + stop_width_hz) / nyq_rate])
w_filtered = lfilter(taps, 1.0, w)

sp_w = fourierSpectrum(w, N)
sp_w_filtered = fourierSpectrum(w_filtered, N)

sp_v = fourierSpectrum(v, N)
#sp_v_filtered = fourierSpectrum()


plt.figure()
plt.subplot(411)
plt.plot(t, w)
plt.subplot(412)
plt.plot(f, sp_w)

plt.subplot(413)
plt.plot(t, w_filtered, label="Filtered")

plt.subplot(414)
plt.plot(f, sp_w_filtered, label="Fourier for filtered")

# plt.figure()
# plt.subplot(411)
# plt.plot(t, v)
#
# plt.subplot(412)
# plt.plot(f, sp_v)


plt.show()