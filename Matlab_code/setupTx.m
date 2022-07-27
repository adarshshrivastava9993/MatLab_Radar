
function [sWav,sTx,sAntPlat,sRad,fs,prf] = setupTx(maxrange,range_res,pd,pfa,nint,sAnt,apos,avel,fc)

propspeed = physconst('LightSpeed');    
pulsebw = propspeed/(2*range_res);      
fs = 2.1*pulsebw;                      
time_bandwidth_product = 20;           
prf = propspeed/(2*maxrange);           
sWav = phased.LinearFMWaveform('PRF',prf,'SampleRate',fs,...
                               'PulseWidth',time_bandwidth_product/pulsebw,...
                               'SweepBandwidth',pulsebw,...
                               'SweepInterval','Symmetric');
%% Transmitter
snr = shnidman(pd,pfa,nint,2);               
antG = phased.ArrayGain('SensorArray',sAnt); 
txG = 20;                                   
lambda = propspeed/fc;
Pt = radareqpow(lambda,maxrange,snr,sWav.PulseWidth,...
                'Gain',txG+step(antG,fc,0),'Loss',25); 
sTx = phased.Transmitter('Gain',txG,'PeakPower',Pt,'InUseOutputPort',true); 
%% Antenna Platform
sAntPlat = phased.Platform('InitialPosition',apos,'Velocity',avel);
%% Radiator 
sRad = phased.Radiator('Sensor',sAnt,'OperatingFrequency',fc,'CombineRadiatedSignals',true);
