# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import brewer2mpl
import matplotlib as mpl
from scipy import signal

#plot preprocessing
bmap = brewer2mpl.get_map('Set2','qualitative', 7)
colors = bmap.mpl_colors
params = {
    'axes.labelsize': 22,
    'font.size': 20,
    'legend.fontsize': 20,
    'xtick.labelsize': 20,
    'ytick.labelsize': 20,
    'text.usetex': True ,
    'figure.figsize': [7, 5.5], # instead of 4.5, 4.5 now[7,5.5]
    'font.weight': 'bold',
    'axes.labelweight': 'bold',
    'ps.useafm' : True,
    'pdf.use14corefonts':True,
    'pdf.fonttype': 42,
    'ps.fonttype': 42
}
mpl.rcParams.update(params)

def latexplot(timefactor=3.4324,filtered=False):
    #plot
    if filtered == True:
        b, a = signal.butter(8  , 0.025)
        with open('cost0.txt') as f:
            r=f.readlines()
        y=np.array(r[0].strip().split()).astype(np.float)
        x=np.linspace(1,y.shape[0],y.shape[0])*timefactor
        plt.plot(x, y, color=colors[1], alpha=0.9)
        plt.plot(x, signal.filtfilt(b, a, y/y[-1]), color=colors[2], linewidth=3)
        plt.grid(color='.910', linewidth=1.5)
        
        plt.xlabel('Training time (seconds)', fontsize=20)
        plt.ylabel('Episodic cost fraction', fontsize=20)
        plt.legend(['Original','Filtered'])
    else:
        with open('cost0.txt') as f:
            r=f.readlines()
        y=np.array(r[0].strip().split()).astype(np.float)
        x=np.linspace(1,y.shape[0],y.shape[0])*timefactor
        plt.plot(x, y, color=colors[2], linewidth=3)
        plt.grid(color='.910', linewidth=1.5)
        
        plt.xlabel('Training time (seconds)', fontsize=20)
        plt.ylabel('Episodic cost', fontsize=20)
#        plt.legend(['Original'])
    plt.tight_layout()    

def clopcompare():    
    pointnum=9
    testnum=400
    y=np.array(np.loadtxt('clopdatafig5.txt'))
    clerr1=[0 for i in range(int(y.shape[0]/2))]
    operr1=[0 for i in range(int(y.shape[0]/2))]
    
    # calculate error value and get the average by each test
    for i in range(int(y.shape[0]/2)):
        clerr1[i]=abs(y[2*i])
        operr1[i]=abs(y[2*i+1])
    with open('clopbarfig5.txt', 'wt+') as f:
        for k in range(pointnum):
            print(np.mean(clerr1[testnum*k:testnum*(k+1)]), np.std(clerr1[testnum*k:testnum*(k+1)]), np.mean(operr1[testnum*k:testnum*(k+1)]), np.std(operr1[testnum*k:testnum*(k+1)]), k*5, file=f)
    
    # plot performance compare data and success rate
    sind=0
    eind=9
    perfdata=np.transpose(np.loadtxt('clopbarfig5.txt'))
    f5,=plt.plot(perfdata[4][sind:eind],perfdata[0,sind:eind],'orange', linewidth=3)
    f6,=plt.plot(perfdata[4][sind:eind],perfdata[2,sind:eind],'dodgerblue', linewidth=3)
    plt.fill_between(perfdata[4][sind:eind],perfdata[0][sind:eind]-perfdata[1][sind:eind],perfdata[0][sind:eind]+perfdata[1][sind:eind],alpha=0.3,color='orange')
    plt.fill_between(perfdata[4][sind:eind],perfdata[2][sind:eind]-perfdata[3][sind:eind],perfdata[2][sind:eind]+perfdata[3][sind:eind],alpha=0.3,color='dodgerblue')
    plt.xlabel('Std dev of perturbed noise(Percent of max. control)')
    plt.ylabel('Episodic cost')
    plt.legend(handles=[f5,f6],labels=['Closed-loop','Open-loop'],loc='upper left')
    plt.grid(color='.910', linewidth=1.5)
    plt.show()  
 
def mclopcompare():                   
    nstart=0
    nend=50
    pointnum=11
    testnum=400
    y=np.array(np.loadtxt('clopdatafig6.txt'))
    y1=np.array(np.loadtxt('clopdatafig61.txt'))
    clerr1=[0 for i in range(int(y.shape[0]/2))]
    operr1=[0 for i in range(int(y.shape[0]/2))]
    
    # calculate error value and get the average by each test
    for i in range(int(y.shape[0]/2)):
        clerr1[i]=abs(y[2*i])
        operr1[i]=abs(y[2*i+1])
    with open('clopbarfig6.txt', 'wt+') as f:
        for k in range(pointnum):
            print(np.mean(clerr1[testnum*k:testnum*(k+1)]), np.std(clerr1[testnum*k:testnum*(k+1)]), np.mean(operr1[testnum*k:testnum*(k+1)]), np.std(operr1[testnum*k:testnum*(k+1)]), k*0.00002/0.02*100, file=f)
    
    pt = 6
    clerr2=[0 for i in range(int(y1.shape[0]/2))]
    for i in range(int(y1.shape[0]/2)):
        clerr2[i]=abs(y1[2*i])
    with open('clopbarfig61.txt', 'wt+') as f:
        for k in range(pt):
            print(np.mean(clerr2[testnum*k:testnum*(k+1)]), np.std(clerr2[testnum*k:testnum*(k+1)]), k*0.0004/0.02*100, file=f)
    
    
    # plot performance compare data and success rate
    sind=int(nstart/100*(pointnum-1))
    eind=int(nend/100*(pointnum-1))+1
    perfdata=np.transpose(np.loadtxt('clopbarfig6.txt'))
    perfdata1=np.transpose(np.loadtxt('clopbarfig61.txt'))
    # f5,=plt.plot(perfdata[4][sind:eind],perfdata[0][sind:eind],'orange', linewidth=3)
    f5,=plt.plot(perfdata1[2][sind:],perfdata1[0][sind:],'orange', linewidth=3)
    f6,=plt.plot(perfdata[4][sind:eind],perfdata[2][sind:eind],'dodgerblue', linewidth=3)
    # plt.fill_between(perfdata[4][sind:eind],perfdata[0][sind:eind]-perfdata[1][sind:eind],perfdata[0][sind:eind]+perfdata[1][sind:eind],alpha=0.3,color='orange')
    plt.fill_between(perfdata1[2][sind:],perfdata1[0][sind:]-perfdata1[1][sind:],perfdata1[0][sind:]+perfdata1[1][sind:],alpha=0.3,color='orange')
    plt.fill_between(perfdata[4][sind:eind],perfdata[2][sind:eind]-perfdata[3][sind:eind],perfdata[2][sind:eind]+perfdata[3][sind:eind],alpha=0.3,color='dodgerblue')
    plt.xlabel('Std dev of measurement noise(Percent of max. measurement)')
    plt.ylabel('Episodic cost')
    plt.legend(handles=[f5,f6,],labels=['LQG','LQR'],loc='upper right')
    plt.grid(color='.910', linewidth=1.5)
    plt.show() 
    
if __name__=='__main__':
    clopcompare()
    mclopcompare()

