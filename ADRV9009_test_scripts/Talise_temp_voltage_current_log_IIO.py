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
    print("Please install scipy")

sys.path.append('C:/Users/fhurgoi/Documents/Projects/ThermalChamber/test-equipment-scripts/thermal_chamber_scripts/application')
from chamber import *

#Error
ERROR = -1

#Success
SUCCESS = 0

# channel name index mapping
# for phy:
TMP = 2

# buffer length index mapping
BUFLEN = 2 ** 17

# ADRV9009 operational parameters
TRXLO = 1000e6

TX0BW = 225e6
TX0FS = 245.76e6
RX0BW = 100e6
RX0FS = 122.88e6

FREQ_BAND = 100e6

N_BITS = 16

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
    
    global phy_a, phy_b, txdac, rxadc, ctx, ams, adm1177
    
    try:
        ctx = iio.Context('ip:192.168.1.140')
    except:
        return ERROR

    adm1177 = ctx.find_device("adm1177")
    ams = ctx.find_device("ams") # Register control
    #phy_a = ctx.find_device("adrv9009-phy") # Register control
    #phy_b = ctx.find_device("adrv9009-phy-b")
    #txdac = ctx.find_device("axi-adrv9009-tx-hpc") # TX/DAC Core in HDL for DMA (plus DDS)
    #rxadc = ctx.find_device("axi-adrv9009-rx-hpc") # RX/ADC Core in HDL for DMA
    
    return ctx, ams, adm1177#phy_a, phy_b, txdac, rxadc


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

#Die temperature
def die_temp():
    global temp_a, temp_b, ps_temp, remote_temp, pl_temp

    #temp_a = int(phy_a.channels[TMP].attrs["input"].value) / 1000.0
    #temp_b = int(phy_b.channels[TMP].attrs["input"].value) / 1000.0

    ps_temp = int(ams.channels[0].attrs["raw"].value)/1000.0
    ps_temp_offset = int(ams.channels[0].attrs["offset"].value)/1000.0
    ps_temp_scale = float(ams.channels[0].attrs["scale"].value)
    ps_temp = (ps_temp + ps_temp_offset)*ps_temp_scale

    remote_temp = int(ams.channels[1].attrs["raw"].value)/1000.0
    remote_temp_offset = int(ams.channels[1].attrs["offset"].value)/1000.0
    remote_temp_scale = float(ams.channels[1].attrs["scale"].value)
    remote_temp = (remote_temp + remote_temp_offset)*remote_temp_scale

    pl_temp = int(ams.channels[1].attrs["raw"].value)/1000.0
    pl_temp_offset = int(ams.channels[1].attrs["offset"].value)/1000.0
    pl_temp_scale = float(ams.channels[1].attrs["scale"].value)
    pl_temp = (pl_temp + pl_temp_offset)*pl_temp_scale

    #txt1.insert(tk.END, 'Die Temperature A: ' + str(temp_a) + '\n\n')
    #txt1.insert(tk.END, 'Die Temperature B: ' + str(temp_b) + '\n\n')
    txt1.insert(tk.END, 'FPGA Die Temp PS: ' + str(ps_temp) + '\n\n')
    root.update_idletasks()
    root.update()

def board_power():
    global voltage, current

    voltage = int(adm1177.channels[1].attrs["raw"].value)/1000.0
    voltage_scale = float(adm1177.channels[1].attrs["scale"].value)
    voltage = voltage * voltage_scale

    current = int(adm1177.channels[0].attrs["raw"].value)/1000.0
    current_scale = float(adm1177.channels[0].attrs["scale"].value)
    current = current * current_scale

    txt1.insert(tk.END, 'Board Voltage: ' + str(voltage) + '\n\n')
    txt1.insert(tk.END, 'Board Current: ' + str(current) + '\n\n')
    root.update_idletasks()
    root.update()

#Get and plot data
def get_plot_data(dev):
    
    global die_tmp_a, die_tmp_b, die_tmp_ps, die_tmp_remote, die_tmp_pl, board_v, board_c, ambient_tmp

    ylim = [0,0]

    die_tmp_a = []
    die_tmp_b = []
    die_tmp_ps = []
    die_tmp_remote = []
    die_tmp_pl = []
    ambient_tmp = []
    board_v = []
    board_c = []
    index = []

    inc = 0

    cuptor = Chamber()

    cuptor.upload_routine('temp_profile1.csv')
    cuptor.start_routine()

    time.sleep(60)

    while(True):

        die_temp()
        board_power()

        chamber_tmp = cuptor.get_temperature()

        ambient_tmp.append(chamber_tmp)
        txt1.insert(tk.END, 'Ambient Temp: ' + str(chamber_tmp) + '\n\n')

        #die_tmp_a.append(temp_a)
        #die_tmp_b.append(temp_b)
        die_tmp_ps.append(ps_temp) 
        die_tmp_remote.append(remote_temp)
        die_tmp_pl.append(pl_temp)

        board_v.append(voltage)
        board_c.append(current)

        inc += 1
        index.append(inc)

        # Plot data
        a.clear()
        a.grid(True)
        a.set_title('Temp log')
        a.set_xlabel('Sample')
        a.set_ylabel('degC')

        #a.plot(index, die_tmp_a, label = 'tmp_a')
        #a.plot(index, die_tmp_b, label = 'tmp_b')
        a.plot(index, die_tmp_ps, label = 'tmp_ps')
        a.plot(index, die_tmp_pl, label = 'tmp_pl')
        a.plot(index, die_tmp_remote, label = 'tmp_remote')
        a.plot(index, ambient_tmp, label = 'tmp_ambient')
        a.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

        canvas.draw()
        root.update_idletasks()
        root.update()

        data_saving()

        if(btn_text.get() == "Start"):
            cuptor.stop_device()
            break
        time.sleep(10)


def data_saving():
    data = {'tmp_ps': die_tmp_ps,'tmp_remote': die_tmp_remote, 'tmp_pl': die_tmp_pl, 'tmp_ambient': ambient_tmp, 'voltage': board_v, 'current': board_c}#'tmp_a:': die_tmp_a, 'tmp_b:': die_tmp_b,

    df = pd.DataFrame(data)
    df.to_csv(os.path.join(dirname, 'RevB/TempChamber/tmp_log.csv'), index = False, sep = ',')

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
    die_temp() 
    
    get_plot_data(dev)
    
    btn_text.set("Start")
    btn_start.config(bg = "lime green")
    
    del dev

#Create GUI
def gui():
    
    global root, txt1, start_f, stop_f, btn_text, btn_start, btn_aq, tx_channel, tx_pow, rx_channel, load_profile, tone_offset, rx_gain
    global btn_export, a, canvas, sweep_en, avg_nr, progress_en
    
    root = tk.Tk()
    root.iconbitmap("favicon.ico")
    root.title("Talise temp logger (Analog Devices, Inc.)")

    btn_text = tk.StringVar()

    #default values
    btn_text.set("Start")

    fr1 = tk.Frame(root)
    fr1.pack(side = tk.LEFT, anchor = 'n', padx = 10, pady = 10)

    fr3 = tk.Frame(fr1)
    fr3.grid(row = 1, column = 0)

    btn_start = tk.Button(fr3, textvariable=btn_text, command=start)
    btn_start.config(width = 13, height = 1, bg = "lime green")
    btn_start.grid(row = 0, column = 1, pady = (10,0))
    
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
    a.set_title('Temp Log')
    a.set_xlabel('Samples')
    a.set_ylabel('Temperature [degC]')
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
    gui()

main()