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
    import visa
except:
    print("Please install pyvisa")

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

#Device Components index mapping
PHY = 0
TXDAC = 1
RXADC = 2
CTX = 3
    
# channel name index mapping
# for phy:
RX_LO = 0
TX_LO = 1
FIR_EN = 2
TMP = 3
RX_CONFIG = 4
TX_CONFIG = 5
    
# txdac index mapping
TX1_I_F1 = 0 
TX1_I_F2 = 1
TX1_Q_F1 = 2
TX1_Q_F2 = 3
TX1_I_BUF = 4 
TX1_Q_BUF = 5 
    
# rxadc index mapping:
RX_I = 0
RX_Q = 1
    
# buffer length index mapping
BUFLEN = 2 ** 15

# Pluto operational parameters
TXLO =  70e6
TX0BW = 56e6
TX0FS = 61.44e6
RXLO = 70e6
#RX0BW = 56e6
RX0BW = 56e6
RX0FS = 61.44e6

#TX tone frequency 
TXDAC_FREQ = 1e6 #MHz
 
#Center frequencies, start frequency, stop frequency index mapping
CENTER_FREQ = 0
START_FREQ = 1
STOP_FREQ = 2

dirname = os.path.dirname(__file__)

# Clear Plot and Message Log
def clr_content():
    txt1.delete('1.0', tk.END)
    #a.cla()
    #canvas.draw()
    
# Setup Context
def connect_device():
    
    global phy, txdac, rxadc, ctx
    
    try:
        ctx = iio.Context('ip:10.50.1.181')
        #ctx = iio.Context('ip:192.168.2.1')
        #ctx = iio.Context('ip:10.50.1.219')
    except:
        return ERROR
   
    phy = ctx.find_device("ad9361-phy") # Register control
    txdac = ctx.find_device("cf-ad9361-dds-core-lpc") # TX/DAC Core in HDL for DMA (plus DDS)
    rxadc = ctx.find_device("cf-ad9361-lpc") # RX/ADC Core in HDL for DMA
    
    return phy, txdac, rxadc, ctx

def setup_analyzer():
    global inst

    rm = visa.ResourceManager()
    inst = rm.open_resource('USB0::0x0400::0x09C4::DSA1B154600251::INSTR')
    print(inst.query("*IDN?"))
    inst.write(':BAND:RES 3KHz')
    inst.write(':FREQ:CENT 70MHz')
    inst.write(':FREQ:SPAN 10MHz')
    inst.write(':POW:ATT 20')
    inst.write(':DISP:WIN:TRAC:Y:SCAL:RLEV:OFFS 10')
#Compute frequencies for each band
def comp_freq():

    #Frequency band  
    FREQ_BAND = int(step_freq.get())*1e6#20e6 #MHz

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

def load_gain_calib():
    global gain_freq
    gain_freq = []

    with open('gain_calib.csv') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            gain_freq.append(tuple((row['Frequency(MHz):'], row['Magnitude(dB):'])))

def TX1A():
    global txLO

    txLO = phy.find_channel("altvoltage1", True)
    txLO.attrs["frequency"].value = str(int(TXLO))

    tx_ch = phy.find_channel("voltage0", True)
    tx_ch.attrs["rf_port_select"].value = "A"
    return tx_ch

def TX2A():
    global txLO

    txLO = phy.find_channel("altvoltage1", True)
    txLO.attrs["frequency"].value = str(int(TXLO))

    tx_ch = phy.find_channel("voltage1", True)
    tx_ch.attrs["rf_port_select"].value = "A"
    return tx_ch

def TX1B():
    global txLO

    txLO = phy.find_channel("altvoltage1", True)
    txLO.attrs["frequency"].value = str(int(TXLO))

    tx_ch = phy.find_channel("voltage0", True)
    tx_ch.attrs["rf_port_select"].value = "B"
    return tx_ch

def TX2B():
    global txLO

    txLO = phy.find_channel("altvoltage1", True)
    txLO.attrs["frequency"].value = str(int(TXLO))

    tx_ch = phy.find_channel("voltage1", True)
    tx_ch.attrs["rf_port_select"].value = "B"
    return tx_ch

#Transciever settings  
def setup_device(dev):
    
    global tx, rx, dds0, dds1, dds2, dds3, dds4, dds5, dds6, dds7, v0, v1
    

     # Set LO, BW, FS for TX/RX
    rxLO = phy.find_channel("altvoltage0", True)
    rxLO.attrs["frequency"].value = str(int(RXLO))

    rx = phy.find_channel("voltage0")

    rx.attrs["rf_bandwidth"].value = str(int(RX0BW))
    rx.attrs["sampling_frequency"].value = str(int(RX0FS))
    rx.attrs['gain_control_mode'].value = 'manual'
    rx.attrs['hardwaregain'].value = str(0)

    tx_chann = {"TX1A": TX1A, "TX2A": TX2A, "TX1B": TX1B, "TX2B": TX2B}
    func = tx_chann.get(tx_channel.get(), "nothing")
    tx = func()

    tx.attrs["rf_bandwidth"].value = str(int(TX0BW))
    tx.attrs["sampling_frequency"].value = str(int(TX0FS))
    tx.attrs['hardwaregain'].value = str(0)
    
    # cf-ad9361-dds-core-lpc (buffer capable) channels
    dds0 = txdac.find_channel('altvoltage0',True)
    dds1 = txdac.find_channel('altvoltage1',True)
    dds2 = txdac.find_channel('altvoltage2',True)
    dds3 = txdac.find_channel('altvoltage3',True)
    dds4 = txdac.find_channel('altvoltage4',True)
    dds5 = txdac.find_channel('altvoltage5',True)
    dds6 = txdac.find_channel('altvoltage6',True)
    dds7 = txdac.find_channel('altvoltage7',True)

    if(tx_channel.get() == "TX1A" or tx_channel.get() == "TX1B"):
        dds0.attrs["frequency"].value = str(TXDAC_FREQ)
        dds0.attrs["scale"].value = "0.031616"#"0.1" = -20dBFS, "0.031616 = -30dBFS = -20dBm"
        dds0.attrs["phase"].value = "90000"
        dds2.attrs["frequency"].value = str(TXDAC_FREQ)
        dds2.attrs["scale"].value = "0.031616"
        dds2.attrs["phase"].value = "0"
        dds1.attrs["frequency"].value = "10000000"
        dds1.attrs["scale"].value = "0.0"
        dds3.attrs["frequency"].value =  "10000000"
        dds3.attrs["scale"].value = "0.0"
        dds4.attrs["frequency"].value = "10000000"
        dds4.attrs["scale"].value = "0.0"
        dds5.attrs["frequency"].value =  "10000000"
        dds5.attrs["scale"].value = "0.0"
        dds6.attrs["frequency"].value = "10000000"
        dds6.attrs["scale"].value = "0.0"
        dds7.attrs["frequency"].value =  "10000000"
        dds7.attrs["scale"].value = "0.0"

            # Enable I/Q channels to be associated with TX
        dds0.enabled = True
        dds2.enabled = True
        dds1.enabled = False
        dds3.enabled = False
        dds4.enabled = False
        dds5.enabled = False
        dds6.enabled = False
        dds7.enabled = False

    if(tx_channel.get() == "TX2A" or tx_channel.get() == "TX2B"):
        dds4.attrs["frequency"].value = str(TXDAC_FREQ)
        dds4.attrs["scale"].value = "0.031616"#"0.1" = -20dBFS, "0.031616 = -30dBFS = -20dBm"
        dds4.attrs["phase"].value = "90000"
        dds6.attrs["frequency"].value = str(TXDAC_FREQ)
        dds6.attrs["scale"].value = "0.031616"
        dds6.attrs["phase"].value = "0"
        dds0.attrs["frequency"].value = "10000000"
        dds0.attrs["scale"].value = "0.0"
        dds2.attrs["frequency"].value =  "10000000"
        dds2.attrs["scale"].value = "0.0"
        dds1.attrs["frequency"].value = "10000000"
        dds1.attrs["scale"].value = "0.0"
        dds5.attrs["frequency"].value =  "10000000"
        dds5.attrs["scale"].value = "0.0"
        dds3.attrs["frequency"].value = "10000000"
        dds3.attrs["scale"].value = "0.0"
        dds7.attrs["frequency"].value =  "10000000"
        dds7.attrs["scale"].value = "0.0"

            # Enable I/Q channels to be associated with TX
        dds0.enabled = False
        dds2.enabled = False
        dds1.enabled = False
        dds3.enabled = False
        dds4.enabled = True
        dds5.enabled = False
        dds6.enabled = True
        dds7.enabled = False

    # Enable I/Q channels to be associated with RX buffer
    v0 = rxadc.find_channel("voltage0")
    v0.enabled = False
    v1 = rxadc.find_channel("voltage1")
    v1.enabled = False
    v2 = rxadc.find_channel("voltage2")
    v2.enabled = False
    v3 = rxadc.find_channel("voltage3")
    v3.enabled = False

#Die temperature
def die_temp(dev):
    
    temp = int(dev[PHY].channels[TMP].attrs["input"].value) / 1000.0
    txt1.insert(tk.END, 'Die Temperature: ' + str(temp) + '\n\n')
    root.update_idletasks()
    root.update()

#Compute and set HW gain
def set_hw_gain(dev, center_freq):
        
    #set manual hw gain
    rx.attrs['gain_control_mode'].value = 'manual'
    rx.attrs['hardwaregain'].value = str(0)
    
    #print minimum hardware gain
    txt1.insert(tk.END, 'Hardware Gain set to: ' + str(0) + ' dB\n\n')
    root.update_idletasks()
    root.update()
    
    return 0

def data_saving():
    fdata = {'Frequency(MHz):': save_freq, 'TX Power(dsFS):':save_gen, 'TX Power received(dBm):': save_data}
    #fdata = {'Raw Data':data}

    df = pd.DataFrame(fdata)
    df.to_csv(os.path.join(dirname, 'TX_Logs/PowerSweep/ADRV9361_TX1_power_sweep.csv'), index = False, sep = ',')

#    fdata1 = {'Frequency(MHz):': save_freq_noise, 'Noise Floor(dBm):':save_noise}
#    df1 = pd.DataFrame(fdata1)
#    df1.to_csv(os.path.join(dirname, 'TX_Logs/PowerSweep/ADRV9361_TX1_noise_floor.csv'), index = False, sep = ',')

#Get and plot data
def tx_sweep(dev, center_freq):

    global save_freq, save_data, data, savegen, save_gen#, save_noise, save_freq_noise

    data = []
    save_data = []
    save_freq = []
    save_gen = []

    Lo_freq = []
    max_amp = []
    pow_gen = []
#    save_noise = []
#    save_freq_noise = []

    data_trace = []

    ylim = [0,0]
    start_p = int(start_pow.get())
    stop_p = int(stop_pow.get())
    step_p = 5
    #inst.write(':FREQ:SPAN ' + freq_step + 'MHz')

    while(True):
        for i in range(0, len(center_freq)):

            RXLO = center_freq[i]
            TXLO = RXLO
            #rxLO.attrs["frequency"].value = str(int(RXLO))
            txLO.attrs["frequency"].value = str(int(TXLO))

            time.sleep(0.1)
            a_freq = str(int(TXLO/1e6)) 
            txt1.insert(tk.END, 'LO Frequency: ' + str(TXLO/1e6) + ' MHz\n')

            for j in range (0, int(((stop_p - start_p)/step_p))+1):
                tx_power = start_p + j*step_p

                if(tx_channel.get() == "TX1A" or tx_channel.get() == "TX1B"):
                    dds0.attrs["scale"].value = str(10 ** (tx_power/20))
                    dds2.attrs["scale"].value = str(10 ** (tx_power/20))
                if(tx_channel.get() == "TX2A" or tx_channel.get() == "TX2B"):
                    dds4.attrs["scale"].value = str(10 ** (tx_power/20))
                    dds6.attrs["scale"].value = str(10 ** (tx_power/20))

                print(tx_power)
                print(str(10 ** (tx_power/20)))
                time.sleep(0.1)
                inst.write(':FREQ:CENT ' + a_freq + 'MHz')

                #delay before data acquisition
                time.sleep(1.2)

                inst.write(':TRAC:DATA? TRACE1')
                time.sleep(0.1)
                data = inst.read_bytes(9025)
                data_bytes = data[12:]
                data_str = data_bytes.decode('utf-8').split(', ')
            
                for index, item in enumerate(data_str):
                    data_str[index] = float(item)

                Lo_freq.append(RXLO/1e6)
                rx_max = np.max(data_str)
                max_amp.append(rx_max)
                pow_gen.append(tx_power)

                save_freq.append(RXLO/1e6)
                save_data.append(rx_max)
                save_gen.append(tx_power)

                txt1.insert(tk.END, 'Amplitude: \n')
                txt1.insert(tk.END, '    Set: ' + str(tx_power) + ' dBFS\n')
                txt1.insert(tk.END, '    Meas: ' + str(rx_max) + ' dBm\n')
                root.update_idletasks()
                root.update()

            data_trace.append(go.Scatter(
                x = pow_gen, y = max_amp,
                mode = 'lines+markers',
                name = str(RXLO/1e6) + 'MHz'))

            Lo_freq.clear()
            max_amp.clear()
            pow_gen.clear()

#            noise_f = compute_noise_floor(data_str)
#            save_noise.append(noise_f)
#            save_freq_noise.append(str(RXLO/1e6))

            x_title = 'Generated TX Power (dBFS)'
            layout = go.Layout(
                # width=1500, height=750,
                title='TX Power Sweep', titlefont=dict(family='Arial', size=28),
                xaxis=dict(title=x_title, titlefont=dict(family='Arial', size=28),
                           tickangle=0, tickfont=dict(family='Arial', size=14), dtick=5, range = [start_p, stop_p]),
                yaxis=dict(title='Measured TX Power (dBm)', titlefont=dict(family='Arial', size=28),
                           tickfont=dict(family='Arial', size=14), dtick=5, range = [start_p, stop_p + 20]),
                barmode='group',
                legend=dict(
                    #orientation="h", x=0.0, y=-0.25,
                    font=dict(family='sans-serif', size=18)),
                showlegend = True)

            plotly.offline.plot({"data": data_trace, "layout": layout}, auto_open=False,
                filename = os.path.join(dirname, 'TX_Logs/PowerSweep/ADRV9361_TX1_power_sweep.html'))

            if(btn_text.get() == "Start" and sweep_en.get() == False):
                break

        if(btn_text.get() == "Start" and sweep_en.get() == True):
            break

    data_saving()

def compute_noise_floor(fft_data):
    #global noise_floor

    fft_size = len(fft_data)
    avg_size = fft_size/2
    noise_floor = fft_data[0] / avg_size

    for i in range(1, int(avg_size/2)):
        if(fft_data[i] < (noise_floor + 2)):
            noise_floor += fft_data[i] / avg_size
        else:
            noise_floor += noise_floor / avg_size

    for i in range(int(3*avg_size/2), fft_size):
        if(fft_data[i] < (noise_floor + 2)):
            noise_floor += fft_data[i] / avg_size
        else:
            noise_floor += noise_floor / avg_size

    return noise_floor

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
    
    btn_text.set("Start")
    btn_start.config(bg = "lime green")
    
    #Clear GUI Content
    clr_content()

    #connect device
    dev = connect_device()
    
    #Check connection
    if(check_connection(dev) == ERROR):
        return

    # Read and print Die Temperature
    die_temp(dev) 
    
    #compute center frequencies
    freq = comp_freq()

    #Check for valid input frequencies
    if(check_freq(freq) == ERROR):
        return
    
    setup_device(dev)
    setup_analyzer()
    time.sleep(0.1)

    tx_sweep(dev, freq[0])

    sweep_en.set(False)

    del dev


#Start Button
def start():
    
    if(btn_text.get() == "Stop"):
        btn_text.set("Start")
        btn_start.config(bg = "lime green")
        return
    else:
        btn_text.set("Stop")
        btn_start.config(bg = "red")
        
    #Clear GUI Content
    clr_content()

    #connect device
    dev = connect_device()

    #Check connection
    if(check_connection(dev) == ERROR):
        btn_text.set("Start")
        btn_start.config(bg = "lime green")
        return

    # Read and print Die Temperature
    die_temp(dev) 
    
    #compute center frequencies
    freq = comp_freq()
    
    #Check for valid input frequencies
    if(check_freq(freq) == ERROR):
        return
    
    #setup device
    setup_device(dev)
    time.sleep(0.1)


    tx_sweep(dev, freq[0])

    btn_text.set("Start")
    btn_start.config(bg = "lime green")
    
    del dev

#Create GUI
def gui():
    
    global root, txt1, start_f, stop_f, start_pow, stop_pow, btn_text, btn_start, step_freq, sweep_en, tx_channel
    
    root = tk.Tk()
    root.iconbitmap("favicon.ico")
    root.title("Power Supply Noise Sniffer (Analog Devices, Inc.)")

    start_f = tk.StringVar()
    stop_f = tk.StringVar()
    start_pow = tk.StringVar()
    stop_pow = tk.StringVar()
    sweep_en = tk.BooleanVar()
    btn_text = tk.StringVar()
    step_freq = tk.StringVar()
    tx_channel = tk.StringVar()

    #default values
    step_freq.set("40")
    start_f.set("80")
    stop_f.set("3000")
    start_pow.set("-40")
    stop_pow.set("0")
    sweep_en.set(False)
    btn_text.set("Start")
    tx_channel.set("TX1A")

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

    label8 = tk.Label(fr2, text = "Frequency Step [MHz]: ")
    label8.grid(row = 7, column = 0)

    entry5 = tk.Entry(fr2, textvariable=step_freq)
    entry5.grid(row = 7, column = 1)

    label9 = tk.Label(fr2, text = "Start TX Power [dbFS]: ")
    label9.grid(row = 9, column = 0)

    entry6 = tk.Entry(fr2, textvariable=start_pow)
    entry6.grid(row = 9, column = 1)

    label10 = tk.Label(fr2, text = "Stop TX Power [dbFS]: ")
    label10.grid(row = 10, column = 0)

    entry6 = tk.Entry(fr2, textvariable=stop_pow)
    entry6.grid(row = 10, column = 1)

    label11 = tk.Label(fr2, text = "TX Channel: ")
    label11.grid(row = 11, column = 0)

    entry7 = tk.Entry(fr2, textvariable=tx_channel)
    entry7.grid(row = 11, column = 1)

    fr3 = tk.Frame(fr1)
    fr3.grid(row = 1, column = 0)

    btn_start = tk.Button(fr3, textvariable=btn_text, command=start)
    btn_start.config(width = 13, height = 1, bg = "lime green")
    btn_start.grid(row = 0, column = 1, pady = (10,0))
    
    btn_sweep = tk.Button(fr3, text="Single Sweep", command=sweep)
    btn_sweep.config(width = 13, height = 1,  bg = "orange")
    btn_sweep.grid(row = 0, column = 0, pady = (10,0))
 
    fr4 = tk.Frame(fr1)
    fr4.grid(row = 2, column = 0)
        
    label12 = tk.Label(fr4, text = "Message Log: ")
    label12.grid(row = 0, column = 0)
    
    txt1 = tkscrolled.ScrolledText(fr4, width = 40, height = 20)
    txt1.grid(row = 1, column = 0)
    
    root.update_idletasks()
    root.mainloop()

#main function
def main():
    gui()

main()