
# File: ps_noise_sniffer.py
# Description: Power Supply Noise Sniffer
# Author: Mark Thoren (mark.thoren@analog.com)
#         Antoniu Miclaus (antoniu.miclaus@analog.com)
#
# Copyright 2018(c) Analog Devices, Inc.
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#  - Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  - Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#  - Neither the name of Analog Devices, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#  - The use of this software may or may not infringe the patent rights
#    of one or more patent holders.  This license does not release you
#    from the requirement that you obtain separate licenses from these
#    patent holders to use this software.
#  - Use of the software either in source or binary form, must be run
#    on or directly connected to an Analog Devices Inc. component.
#
# THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

'''
Derived from an example posted here:
https://ez.analog.com/thread/99958-ad9361-libiio-api-python-binding

A useful nugget for verifying throughput, from:
https://wiki.analog.com/university/tools/pluto/devs/performance
ADALM-PLUTO Performance Metrics:
Data Throughput (USB)

Via IIOD USB backend
iio_readdev -u usb:1.100.5 -b 100000 cf-ad9361-lpc | pv > /dev/null
210GiB 1:19:51 [26,1MiB/s] [     <=>     

Via IIOD Network backend
iio_readdev -n 192.168.2.1 -b 100000 cf-ad9361-lpc | pv > /dev/null
203MiB 0:00:10 [20,4MiB/s] [              <=>  

'''

import sys, os
import time

try:
    import tkinter as tk
    from tkinter import filedialog
    import tkinter.scrolledtext as tkscrolled

except:
    print("Please install tkinter")
try:
    import csv
except:
    print("Please install csv")
try:
    import pandas as pd
except:
    print("Please install pandas")
try:
    import numpy as np
except:
    print("Please install numpy")

try:
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
except:
    print("Please install matplotlib")

try: 
    import iio
except:
    sys.path.append('C:/Users/fhurgoi/Documents/libiio-master/bindings/python') #add own path to python bindings
    import iio

try:
    import serial
except:
    print("Please install pywin32")

import plotly
import plotly.graph_objs as go

try:
    import scipy.signal
except:
    pritn("Please install scipy")

#Error
ERROR = -1

#Success
SUCCESS = 0

# channel name index mapping
# for phy:
TMP = 2

# buffer length index mapping
BUFLEN = 2 ** 18

# ADRV9009 operational parameters
N_BITS = 16

TRXLO = 1000e6

TX0BW = 225e6
TX0FS = 245.76e6
RX0BW = 100e6
RX0FS = 122.88e6#122.88e6

FREQ_BAND = 100e6

#TX tone frequency 
TXDAC_FREQ = 10e6 #MHz
 
#Center frequencies, start frequency, stop frequency index mapping
CENTER_FREQ = 0
START_FREQ = 1
STOP_FREQ = 2

dirname = os.path.dirname(__file__)

# Clear Plot and Message Log
def clr_content():
    txt1.delete('1.0', tk.END)
    a.cla()
    canvas.draw()
    
# Setup Context
def connect_device():
    
    global phy_a, phy_b, txdac, rxadc, ctx
    
    try:
        ctx = iio.Context('ip:10.48.65.126')
        #ctx = iio.Context('ip:192.168.1.219')
    except:
        return ERROR
   
    phy_a = ctx.find_device("adrv9009-phy") # Register control
    phy_b = ctx.find_device("adrv9009-phy-b")
    txdac = ctx.find_device("axi-adrv9009-tx-hpc") # TX/DAC Core in HDL for DMA (plus DDS)
    rxadc = ctx.find_device("axi-adrv9009-rx-hpc") # RX/ADC Core in HDL for DMA
    
    return phy_a, phy_b, txdac, rxadc, ctx

def connect_generator():

    global COMgen, ser

    COMgen = str('COM' + gen_COM.get())

    # configure the serial connections (the parameters differs on the device you are connecting to)
    ser = serial.Serial(
        port=COMgen,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )

    try:
        port = ser.isOpen()
        print (str(port))
    except serial.SerialException:
        ser.close()
        print ("port is closed")
        ser.isOpen()
        print ("port is open")

    #ser.write(('*IDN?' + '\r\n').encode('utf-8'))
    ser.write(('FREQ 70e6' + '\r\n').encode('utf-8'))
    ser.write(('POW ' + str(int(tx_pow.get())) + '\r\n').encode('utf-8'))
    ser.write(('OUTP OFF' + '\r\n').encode('utf-8'))

def disconnect_generator():
    # configure the serial connections (the parameters differs on the device you are connecting to)
    ser.close()

#Compute frequencies for each band
def comp_freq():
    center_freq = np.array([]) #array that holds the center frequencies
    
    #Convert input start/stop frequency values to integers
    try:
        start_f.get()
        stop_f.get()
    except:
        center_freq = np.append(center_freq, ERROR)
        return center_freq
    
    #Get start/stop frequency values
    start_freq = float(start_f.get()) * float(1e6) #MHz
    stop_freq = float(stop_f.get()) * float(1e6) #MHz
    
    #Check if start frequency is larger that input frequency
    if (start_freq > stop_freq or start_freq < 70e6 or stop_freq > 6e9):
        center_freq = np.append(center_freq, ERROR)
    else:
        nr_center_freq = int((stop_freq - start_freq)/FREQ_BAND) + 1 
        center_freq = np.append(center_freq, start_freq)
    
    #Compute center frequencies for each band
        for i in range(1, nr_center_freq):
            center_freq = np.append(center_freq, center_freq[i-1] + FREQ_BAND)
    return center_freq, start_freq, stop_freq

def configure_device():

    with open('Tx_BW200_IR245p76_Rx_BW100_OR122p88_ORx_BW200_OR245p76_DC245p76.txt', 'r') as myfile:
        filterfile = myfile.read()

    iio.Context.set_timeout(ctx, 30000);
    time.sleep(0.1)

    phy_a.attrs["profile_config"].value = filterfile
    phy_b.attrs['profile_config'].value = filterfile

    iio.Context.set_timeout(ctx, 3000);

def RXA1():

    rx_channel = phy_a.find_channel("voltage0")
    return rx_channel

def RXA2():

    rx_channel = phy_a.find_channel("voltage1")
    return rx_channel

def RXB1():

    rx_channel = phy_b.find_channel("voltage0")
    return rx_channel

def RXB2():

    rx_channel = phy_b.find_channel("voltage1")
    return rx_channel

#Transciever settings  
def setup_device():
    
    global tx, rx, TX1_I_F1, TX1_Q_F1, TX2_I_F1, TX2_Q_F1, TX2_Q_F1, TX3_I_F1, TX3_Q_F1, TX4_I_F1, TX4_Q_F1
    global TRX_LO_A, TRX_LO_B

    # Global configuration
    TRX_LO_A = phy_a.find_channel("altvoltage0", True)
    TRX_LO_A.attrs["frequency"].value = str(int(TRXLO))
    TRX_LO_A.attrs["frequency_hopping_mode_enable"].value = str(0)

    TRX_LO_B = phy_b.find_channel("altvoltage0", True)
    TRX_LO_B.attrs["frequency"].value = str(int(TRXLO))
    TRX_LO_B.attrs["frequency_hopping_mode_enable"].value = str(0)

    # Default RX Configuration

    #configure just the RX channel under test
    rx_chann = {"RX1": RXA1, "RX2": RXA2, "RX3": RXB1, "RX4": RXB2}
    func = rx_chann.get(rx_channel.get(), "nothing")
    rx = func()

    rx.attrs['gain_control_mode'].value = "manual"
    rx.attrs['hardwaregain'].value = rx_gain.get()
    #rx.attrs["rf_bandwidth"].value = (str(int(RX0BW)))
    #rx.attrs["sampling_frequency"].value = str(int(RX0FS))
    #rx.attrs["powerdown"].value = str(0)

    #rx.attrs["gain_control_pin_mode_en"].value = str(1)
    if(track_en.get() == True):
        rx.attrs['quadrature_tracking_en'].value = str(1)
    else:
        rx.attrs['quadrature_tracking_en'].value = str(0)

    # axi-adrv9009-tx-hpc (buffer capable) channels
    TX1_I_F1 = txdac.find_channel('altvoltage0',True)
    TX1_I_F2 = txdac.find_channel('altvoltage1',True)
    TX1_Q_F1 = txdac.find_channel('altvoltage2',True)
    TX1_Q_F2 = txdac.find_channel('altvoltage3',True)
    TX2_I_F1 = txdac.find_channel('altvoltage4',True)
    TX2_I_F2 = txdac.find_channel('altvoltage5',True)
    TX2_Q_F1 = txdac.find_channel('altvoltage6',True)
    TX2_Q_F2 = txdac.find_channel('altvoltage7',True)
    TX3_I_F1 = txdac.find_channel('altvoltage8',True)
    TX3_I_F2 = txdac.find_channel('altvoltage9',True)
    TX3_Q_F1 = txdac.find_channel('altvoltage10',True)
    TX3_Q_F2 = txdac.find_channel('altvoltage11',True)
    TX4_I_F1 = txdac.find_channel('altvoltage12',True)
    TX4_I_F2 = txdac.find_channel('altvoltage13',True)
    TX4_Q_F1 = txdac.find_channel('altvoltage14',True)
    TX4_Q_F2 = txdac.find_channel('altvoltage15',True)

    # Turn TX tone off
    TX1_I_F1.attrs["frequency"].value = "10000000"
    TX1_I_F1.attrs["scale"].value = "0.0"#"0.031616"#"0.1" = -20dBFS, "0.031616 = -30dBFS = -20dBm"
    TX1_Q_F1.attrs["frequency"].value = "10000000"
    TX1_Q_F1.attrs["scale"].value = "0.0"
    TX1_I_F2.attrs["frequency"].value = "10000000"
    TX1_I_F2.attrs["scale"].value = "0.0"#"0.031616"#"0.1" = -20dBFS, "0.031616 = -30dBFS = -20dBm"
    TX1_Q_F2.attrs["frequency"].value = "10000000"
    TX1_Q_F2.attrs["scale"].value = "0.0"
    TX2_I_F1.attrs["frequency"].value = "10000000"
    TX2_I_F1.attrs["scale"].value = "0.0"
    TX2_Q_F1.attrs["frequency"].value = "10000000"
    TX2_Q_F1.attrs["scale"].value = "0.0"
    TX2_I_F2.attrs["frequency"].value = "10000000"
    TX2_I_F2.attrs["scale"].value = "0.0"
    TX2_Q_F2.attrs["frequency"].value = "10000000"
    TX2_Q_F2.attrs["scale"].value = "0.0"
    TX3_I_F1.attrs["frequency"].value =  "10000000"
    TX3_I_F1.attrs["scale"].value = "0.0"
    TX3_Q_F1.attrs["frequency"].value = "10000000"
    TX3_Q_F1.attrs["scale"].value = "0.0"
    TX3_I_F2.attrs["frequency"].value =  "10000000"
    TX3_I_F2.attrs["scale"].value = "0.0"
    TX3_Q_F2.attrs["frequency"].value = "10000000"
    TX3_Q_F2.attrs["scale"].value = "0.0"
    TX4_I_F1.attrs["frequency"].value =  "10000000"
    TX4_I_F1.attrs["scale"].value = "0.0"
    TX4_Q_F1.attrs["frequency"].value = "10000000"
    TX4_Q_F1.attrs["scale"].value = "0.0"
    TX4_I_F2.attrs["frequency"].value =  "10000000"
    TX4_I_F2.attrs["scale"].value = "0.0"
    TX4_Q_F2.attrs["frequency"].value = "10000000"
    TX4_Q_F2.attrs["scale"].value = "0.0"

    v0_i = rxadc.find_channel("voltage0_i")
    v0_q = rxadc.find_channel("voltage0_q")
    v1_i = rxadc.find_channel("voltage1_i")
    v1_q = rxadc.find_channel("voltage1_q")
    v2_i = rxadc.find_channel("voltage2_i")
    v2_q = rxadc.find_channel("voltage2_q")
    v3_i = rxadc.find_channel("voltage3_i")
    v3_q = rxadc.find_channel("voltage3_q")
    v0_i.enabled = False
    v0_q.enabled = False
    v1_i.enabled = False
    v1_q.enabled = False
    v2_i.enabled = False
    v2_q.enabled = False
    v3_i.enabled = False
    v3_q.enabled = False

    # Enable I/Q channels to be associated with RX buffer
    if(rx_channel.get() == "RX1"):
        v0_i.enabled = True
        v0_q.enabled = True
    elif(rx_channel.get() == "RX2"):
        v1_i.enabled = True
        v1_q.enabled = True
    elif(rx_channel.get() == "RX3"):
        v2_i.enabled = True
        v2_q.enabled = True
    elif(rx_channel.get() == "RX4"):
        v3_i.enabled = True
        v3_q.enabled = True
    else:
        txt1.insert(tk.END, 'Ghinion. Nu-i bine canalu de RX!\n\n')

    #turn radio on
    phy_a.attrs["ensm_mode"]. value = "radio_on"
    phy_b.attrs["ensm_mode"]. value = "radio_on"


#Die temperature
def die_temp():
    
    temp_a = int(phy_a.channels[TMP].attrs["input"].value) / 1000.0
    txt1.insert(tk.END, 'Die Temperature A: ' + str(temp_a) + '\n\n')
    temp_b = int(phy_b.channels[TMP].attrs["input"].value) / 1000.0
    txt1.insert(tk.END, 'Die Temperature B: ' + str(temp_b) + '\n\n')
    root.update_idletasks()
    root.update()

#Get and plot data
def get_plot_data(dev, center_freq):
    
    global save_freq, save_data, save_noise, save_spur_number, save_max_spur, save_LO_leakage, save_IM1, save_IM2, save_IM3
    global save_spur_freq, save_spur_data

    TX_offset = float(tone_offset.get())*1e6
    ylim = [0,0]
    # Create IIO RX Buffer
    rxbuf = iio.Buffer(dev[3], BUFLEN, False) 
    
    # Window function, max value = unity
    window = np.hanning(BUFLEN)
    #window = np.blackman(BUFLEN)
    
    avg_step = 0
    avgband = np.array([])

    save_data = []
    save_freq = []
    save_noise = []
    save_spur_freq = []
    save_spur_data = []
    save_spur_number = []
    save_max_spur = []
    save_LO_leakage = []
    save_IM1 = []
    save_IM2 = []
    save_IM3 = []


    if(gen_en.get() == True):
        ser.write(('POW ' + str(int(tx_pow.get())) + '\r\n').encode('utf-8'))
        ser.write(('OUTP ON' + '\r\n').encode('utf-8'))

    ss = 0

    while(avg_step == 0):

        #acquire data
        for i in range(0, len(center_freq)):
            
            RXLO = center_freq[i]
            TRX_LO_A.attrs["frequency"].value = str(int(RXLO))
            TRX_LO_B.attrs["frequency"].value = str(int(RXLO))

            time.sleep(0.2)
            root.update_idletasks()
            root.update()
            
            #generate test tone
            if(gen_en.get() == True):
               ser.write(('FREQ ' + str(RXLO + TX_offset) + '\r\n').encode('utf-8'))
               #delay before data acquisition
               time.sleep(0.5)
            
            btn_aq.config(bg = "red")
            root.update()
            for k in range(0, int(avg_nr.get())+1):
                #flush operations
                for j in range(5):
                    rxbuf.refill()
                    x = rxbuf.read()
                
                buff = np.frombuffer(x, dtype=np.int16)
                    
                #apply window
                rxvals_i = buff[0::2] * window
                rxvals_q = buff[1::2] * window
                    
                #construct complex IQ data
                rxvals_iq = rxvals_i + (1j * rxvals_q)

                # apply FFT and get amplitude of the IQ data  
                newband = np.abs(np.fft.fft(rxvals_iq)) 
                    
                #apply averaging
                if(k != 0):
                    avgband = (newband) * 1/k + avgband * (k-1) / k;#(newband)/int(avg_nr.get())
                else:
                    avgband = newband

            btn_aq.config(bg = "grey")
            root.update()

            #compute cutoff thresholds        
            low_th = int(((RX0FS - FREQ_BAND)/(RX0FS * 2)) * BUFLEN)
            high_th = int(BUFLEN - (((RX0FS - FREQ_BAND)/(RX0FS * 2)) * BUFLEN))

            #compute frequency bins
            # 0, 1, 2, ..., n/2-1, -n/2,..., -1
            freq_bins = (np.fft.fftfreq(BUFLEN, 1/RX0FS) + (center_freq[0] + FREQ_BAND * i)) /1e6

            # Create tuple such that frequencies and corresponding amplitudes can be manipulated together
            tuple1 = zip(freq_bins, avgband)
            
            # Rearrange such that the center frequency is in the middle.
            tuple1 = sorted(tuple1, key=lambda x: x[0])
            #print(tuple1)
            # Extract the passband of the FIR response
            tuple1 = tuple1[low_th : high_th]
            
            #iq = np.array([nr[1] for nr in tuple1]) / (10 ** (gain_calib/10))
            iq = np.array([nr[1] for nr in tuple1])
            fbin = np.array([nr[0] for nr in tuple1])

            fft_rxvals_iq_dbFS = 10 * np.log10(iq**2) + 20 * np.log10(2/2**(N_BITS-1))- 20 * np.log10(BUFLEN) + 20 * np.log10(1/(1.5**0.5))# IIO
            # Compute in dB, subtract hardware gain to normalize to absolute analog signal level at input

            fft_rxvals_iq_dbm = fft_rxvals_iq_dbFS - 6.7

            noise_f = compute_noise_floor(fft_rxvals_iq_dbm)
            spurs = scipy.signal.find_peaks(fft_rxvals_iq_dbm, ((noise_f + 6), 0), None, 5, 3, None, None, 0.5, None)

            n_bins = fft_rxvals_iq_dbm.size

            freqbin = FREQ_BAND/n_bins
            spurs_fund_th_low = int((FREQ_BAND/2 + TX_offset - 3e6)/freqbin)
            spurs_fund_th_high = int((FREQ_BAND/2 + TX_offset + 3e6)/freqbin)

            IM1_left_th_low = int((FREQ_BAND/2 - TX_offset - 100e3)/freqbin)
            IM1_left_th_high = int((FREQ_BAND/2 - TX_offset + 100e3)/freqbin)

            IM2_left_th_low = int((FREQ_BAND/2 - 2*TX_offset - 100e3)/freqbin)
            IM2_left_th_high = int((FREQ_BAND/2 - 2*TX_offset + 50e3)/freqbin)

            IM3_left_th_low = int((FREQ_BAND/2 - 3*TX_offset - 100e3)/freqbin)
            IM3_left_th_high = int((FREQ_BAND/2 - 3*TX_offset + 100e3)/freqbin)

            LO_leakage = np.max(fft_rxvals_iq_dbm[(int(n_bins/2) - 10) : (int(n_bins/2) + 10)])
            IM1_left = np.max(fft_rxvals_iq_dbm[IM1_left_th_low : IM1_left_th_high])
            IM2_left = np.max(fft_rxvals_iq_dbm[IM2_left_th_low : IM2_left_th_high])
            IM3_left = np.max(fft_rxvals_iq_dbm[IM3_left_th_low : IM3_left_th_high])

            txt1.insert(tk.END, "Center Frequency set to: " + str(RXLO/1e6) + ' MHz\n')
            txt1.insert(tk.END, "RX amplitude : " + str('%04.3f'%(np.max(fft_rxvals_iq_dbm)))+ " dBm\n")
            txt1.insert(tk.END, "LO leakage: " + str('%04.3f'%LO_leakage)+ " dBm\n")
            txt1.insert(tk.END, "IM1 left: " + str('%04.3f'%IM1_left)+ " dBm\n")
            txt1.insert(tk.END, "IM2 left: " + str('%04.3f'%IM2_left)+ " dBm\n")
            txt1.insert(tk.END, "IM3 left: " + str('%04.3f'%IM3_left)+ " dBm\n")
            txt1.insert(tk.END, "Noise floor: " + str('%04.3f'%noise_f)+ " dBm\n")

            spurs_number_band = len(spurs[0])
            spurs_number_save = 0
            for i in range(0, spurs_number_band):
                if spurs[0][i] <= spurs_fund_th_low or spurs_fund_th_high <= spurs[0][i]:
                    save_spur_freq.append((RXLO - (FREQ_BAND/2) + ((spurs[0][i])*freqbin))/1e6)#1875
                    save_spur_data.append(spurs[1]['peak_heights'][i])
                    txt1.insert(tk.END, "Spurs freq: " + str('%04.3f'%save_spur_freq[ss+spurs_number_save]) + " MHz\n" +" Spurs amplitude: " + str('%04.3f'%save_spur_data[ss+spurs_number_save])+ " dBm\n")
                    spurs_number_save += 1
            ss += spurs_number_save

            txt1.see("end")

            save_freq.append(RXLO/1e6)
            save_data.append(np.max(fft_rxvals_iq_dbm))
            save_noise.append(noise_f)
            save_spur_number.append(spurs_number_save)
            save_max_spur.append(np.max(spurs[1]['peak_heights']))
            save_LO_leakage.append(LO_leakage)
            save_IM1.append(IM1_left)
            save_IM2.append(IM2_left)
            save_IM3.append(IM3_left)

            # Plot data
            a.clear()
            a.grid(True)
            a.set_title('Frequency Spectrum')
            a.set_xlabel('Fequency (MHz)')
            a.set_ylabel('Magnitude')
            #a.set_ylim([-140, -100])
            a.plot(fbin, fft_rxvals_iq_dbm)

            canvas.draw()
            root.update_idletasks()
            root.update()

            #txt1.insert(tk.END, "time: " + str(time.time()) + "\n")
            #txt1.see("end")

            if(stop.get() == True):
                break
            
        avg_step += 1

    if(gen_en.get() == True):
        #disable TX
        ser.write(('OUTP OFF' + '\r\n').encode('utf-8'))

    #phy_a.attrs["ensm_mode"]. value = "radio_off"
    #phy_b.attrs["ensm_mode"]. value = "radio_off"

    trace0 = go.Scatter(
        x = save_freq,
        y = save_LO_leakage,
        mode = 'lines+markers',
        name = 'RX LO leakage'
    )

    x_title = 'Frequency (MHz)'

    layout = go.Layout(
        # width=1500, height=750,
        title='Frequency Characteristic', titlefont=dict(family='Arial', size=28),
        xaxis=dict(title=x_title, titlefont=dict(family='Arial', size=28),
                   tickangle=0, tickfont=dict(family='Arial', size=14), dtick=1000, range = [0, 6000]),
        yaxis=dict(title='Magnitude (dBm)', titlefont=dict(family='Arial', size=28),
                   tickfont=dict(family='Arial', size=14), dtick=10, range = [-120, -100]),
        barmode='group',
        legend=dict(
            #orientation="h", x=0.0, y=-0.25,
            font=dict(family='sans-serif', size=18)),
        showlegend = True)

    data = [trace0]
    plotly.offline.plot({"data": data, "layout": layout}, auto_open=False,
        filename = os.path.join(dirname, 'RevB/RX_Logs/Spurs/ADRV9009_BW100M_freq_char_' + rx_channel.get() + '_n20dBm_rx_gain_' + rx_gain.get() + '_track_on' + '.html'))

    data_saving()

def compute_noise_floor(fft_data):
    #global noise_floor

    avg_size = fft_data.size/2
    noise_floor = fft_data[0] / avg_size

    for i in range(1, int(avg_size/2)):
        if(fft_data[i] < (noise_floor + 2)):
            noise_floor += fft_data[i] / avg_size
        else:
            noise_floor += noise_floor / avg_size

    for i in range(int(3*avg_size/2), fft_data.size):
        if(fft_data[i] < (noise_floor + 2)):
            noise_floor += fft_data[i] / avg_size
        else:
            noise_floor += noise_floor / avg_size

    return noise_floor

def data_saving():
    data = {'Frequency(MHz):': save_freq, 'Magnitude(dBm):': save_data, 'Noise Floor (dBm):': save_noise, 'Number of Spurs:': save_spur_number, 'Max Spur (dBm):': save_max_spur, 'LO Leakage (dBm)':save_LO_leakage, 'IM1 (dBm)':save_IM1, 'IM2 (dBm)':save_IM2, 'IM3 (dBm)':save_IM3}

    df = pd.DataFrame(data)
    df.to_csv(os.path.join(dirname, 'RevB/RX_Logs/Spurs/ADRV9009_BW100M_freq_char_' + rx_channel.get() + '_n20dBm_rx_gain_' + rx_gain.get() + '_track_on' + '.csv'), index = False, sep = ',')

    data1 = {'Spur Freq (MHz):': save_spur_freq, 'Spur Data (dBm):':save_spur_data}
    df = pd.DataFrame(data1)
    df.to_csv(os.path.join(dirname, 'RevB/RX_Logs/Spurs/ADRV9009_BW100M_spurs_' + rx_channel.get() + '_n20dBm_rx_gain_' + rx_gain.get() + '_track_on' + '.csv'), index = False, sep = ',')

def check_connection(dev):
    if(dev == ERROR):
        txt1.config(fg = "red")
        txt1.insert('1.0', "No Device Detected.\n\n")
        return ERROR
    else:
        txt1.config(fg = 'black')
        txt1.insert('1.0', "Device Connected.\n\n")
        return SUCCESS

def check_freq(freq):
    if(freq[0][0] == ERROR):
        txt1.config(fg = 'red')
        txt1.delete('1.0', tk.END)
        txt1.insert('1.0', "Please insert valid start/stop frequencies.")
        return ERROR
    else:
        txt1.config(fg = "black")
        txt1.insert(tk.END, "Start frequency set to: " + str(freq[START_FREQ]/1e6) + " MHz\n\n")
        txt1.insert(tk.END, "Stop frequency set to: " + str(freq[STOP_FREQ]/1e6) + " MHz\n\n")
        return SUCCESS

def check_avg_nr():
    try:
        int(avg_nr.get())
        return SUCCESS
    except:
        txt1.config(fg = "red")
        txt1.insert(tk.END, "Please insert valid average number.")
        btn_text.set("Start")
        btn_start.config(bg = "lime green")
        return ERROR

#Sweep Button
def sweep():
    
    sweep_en.set(True)
    
    #Clear GUI Content
    clr_content()

    #Check connection
    if(check_connection(dev) == ERROR):
        return

    # Read and print Die Temperature
    die_temp() 
    
    #compute center frequencies
    freq = comp_freq()

    #Check for valid input frequencies
    if(check_freq(freq) == ERROR):
        return
    
    #Check for valid average number    
    if(check_avg_nr() == ERROR):
        return
    
    if(load_profile.get() == True):
        configure_device()
    time.sleep(0.1)

    setup_device()

    if(gen_en.get() == True):
        connect_generator()
    time.sleep(0.1)

    get_plot_data(dev, freq[CENTER_FREQ])

    if(gen_en.get() == True):
        disconnect_generator()

    sweep_en.set(False)

def stop():
    stop.set(true)

#Create GUI
def gui():
    
    global root, txt1, start_f, stop_f, btn_stop, btn_aq, gen_COM, tx_pow, rx_channel, load_profile, tone_offset, rx_gain, track_en
    global a, canvas, sweep_en, avg_nr, gen_en, stop
    
    root = tk.Tk()
    root.iconbitmap("favicon.ico")
    root.title("ADRV9009_RX_testing (Analog Devices, Inc.)")

    start_f = tk.StringVar()
    stop_f = tk.StringVar()
    avg_nr = tk.StringVar()
    sweep_en = tk.BooleanVar()
    gen_en = tk.BooleanVar()
    gen_COM = tk.StringVar()
    tx_pow = tk.StringVar()
    rx_channel = tk.StringVar()
    load_profile = tk.StringVar()
    tone_offset = tk.StringVar()
    rx_gain = tk.StringVar()
    stop = tk.BooleanVar()
    track_en = tk.BooleanVar()

    #default values
    rx_channel.set("RX1")
    gen_COM.set("13")
    tx_pow.set("-20")
    start_f.set("150")
    stop_f.set("6000")
    avg_nr.set("64")
    sweep_en.set(False)
    gen_en.set(False)
    load_profile.set(False)
    tone_offset.set("10")
    rx_gain.set("30")
    stop.set(False)
    track_en.set(False)

    fr1 = tk.Frame(root)
    fr1.pack(side = tk.LEFT, anchor = 'n', padx = 10, pady = 10)

    fr2 = tk.Frame(fr1)
    fr2.grid(row = 0, column = 0, pady = 10)

    label1 = tk.Label(fr2, text = "Start frequency (MHz): ")
    label1.grid(row = 0, column = 0)


    entry1 = tk.Entry(fr2, textvariable=start_f)
    entry1.grid(row = 0, column = 1)

    label2 = tk.Label(fr2, text = "Stop frequency (MHz): ")
    label2.grid(row = 1, column = 0, pady = (0,5))

    entry2 = tk.Entry(fr2, textvariable=stop_f)
    entry2.grid(row = 1, column = 1)
    
    label3 = tk.Label(fr2, text = "Load Profile: ")
    label3.grid(row = 2, column = 0)

    check1 = tk.Checkbutton(fr2, variable=load_profile)
    check1.grid(row = 2, column = 1)
    
    label4 = tk.Label(fr2, text = "Average: ")
    label4.grid(row = 3, column = 0)

    entry3 = tk.Entry(fr2, textvariable=avg_nr)
    entry3.grid(row = 3, column = 1)
    
    label5 = tk.Label(fr2, text = "Enable Generator: ")
    label5.grid(row = 4, column = 0)

    check2 = tk.Checkbutton(fr2, variable=gen_en)
    check2.grid(row = 4, column = 1)

    label7 = tk.Label(fr2, text = "Generator COM: ")
    label7.grid(row = 6, column = 0)

    entry4 = tk.Entry(fr2, textvariable=gen_COM)
    entry4.grid(row = 6, column = 1)

    label8 = tk.Label(fr2, text = "TX Power [dBm]: ")
    label8.grid(row = 7, column = 0)

    entry5 = tk.Entry(fr2, textvariable=tx_pow)
    entry5.grid(row = 7, column = 1)

    label6 = tk.Label(fr2, text = "Tone Offset [MHz]: ")
    label6.grid(row = 8, column = 0)

    entry7 = tk.Entry(fr2, textvariable=tone_offset)
    entry7.grid(row = 8, column = 1)

    label9 = tk.Label(fr2, text = "RX Channel: ")
    label9.grid(row = 9, column = 0)

    entry6 = tk.Entry(fr2, textvariable=rx_channel)
    entry6.grid(row = 9, column = 1)

    label10 = tk.Label(fr2, text = "RX Gain [dB]: ")
    label10.grid(row = 10, column = 0)

    entry8 = tk.Entry(fr2, textvariable=rx_gain)
    entry8.grid(row = 10, column = 1)

    label11 = tk.Label(fr2, text = "Tracking Enable: ")
    label11.grid(row = 11, column = 0)

    check3 = tk.Checkbutton(fr2, variable=track_en)
    check3.grid(row = 11, column = 1)

    fr3 = tk.Frame(fr1)
    fr3.grid(row = 1, column = 0)

    btn_sweep = tk.Button(fr3, text="Single Sweep", command=sweep)
    btn_sweep.config(width = 13, height = 1,  bg = "orange")
    btn_sweep.grid(row = 0, column = 0, pady = (10,0))

    btn_stop = tk.Button(fr3, text="Stop", command=stop)
    btn_stop.config(width = 13, height = 1, bg = "lime green")
    btn_stop.grid(row = 0, column = 1, pady = (10,0))

    btn_aq = tk.Button(fr3, text="Acq&Proc")
    btn_aq.config(width = 13, height = 1, bg = "grey")
    btn_aq.grid(row = 1, column = 0, pady = (10,0))

    fr4 = tk.Frame(fr1)
    fr4.grid(row = 2, column = 0)
        
    label9 = tk.Label(fr4, text = "Message Log: ")
    label9.grid(row = 0, column = 0)
    
    txt1 = tkscrolled.ScrolledText(fr4, width = 40, height = 20)
    txt1.grid(row = 1, column = 0)
    
    fig = plt.figure()
    DPI = fig.get_dpi()
    fig.set_size_inches(1024.0/float(DPI), 640.0/float(DPI))   
    a = fig.add_subplot(111)
    a.grid(True)
    a.set_title('Frequency Spectrum')
    a.set_xlabel('Fequency (MHz)')
    a.set_ylabel('Magnitude')
    plt.tight_layout()
    plotFrame = tk.Frame(master=root)
    plotFrame.pack(side = tk.LEFT, pady = 10, padx = 10, anchor = 'n')

    toolbarFrame = tk.Frame(master=plotFrame)
    toolbarFrame.pack(anchor = 'w', pady = (0,10))
    
    canvas = FigureCanvasTkAgg(fig, master=plotFrame)
    canvas.get_tk_widget().pack(fill = tk.BOTH)
    canvas.draw()
    root.update_idletasks()

    toolbar = NavigationToolbar2TkAgg(canvas, toolbarFrame)
    toolbar.update()
    
    root.mainloop()

#main function
def main():

    global dev

    #connect device
    dev = connect_device()

    configure_device()
    gui()

main()