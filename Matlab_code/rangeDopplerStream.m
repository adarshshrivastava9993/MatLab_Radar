%Project Title :-  "Designing of Object Detection System Using  Radio Technology for Ultra-Wide-band RADAR Communication Applications"
% Designing of Radar System with The help of antenna
%Guided By:- Dr. Pallabi Sarkar (Assistant Professor SEEE,VIT BHOPAL UNIVERSITY).
%Date : 29.01.2022

maxrange  = 8000; 
range_res = 50;   

pd        = 0.9;  
pfa       = 1e-6; 
nint      = 48;   


apos = [0; 0; 0];                  
avel = [30*cos(30);30*sin(30);0];  
fc   = 3e9;                        
sAnt = lowProfileArray('FrequencyRange',[2/3*fc 4/3*fc],'ViewArray',false);
[sWav,sTx,sAntPlat,sRad,fs,prf] = setupTx(maxrange,range_res,pd,pfa,nint,sAnt,apos,avel,fc);


tgtRCS = [1.2 1.1 1.05];
tgtpos = [500 200 200; 0 0 0; 0 0 0];
tgtvel = [100 20 10; 0 0 0; 0 0 0];    % m/s
[sTgt,sTgtMotion,sChan] = setupTheater(tgtRCS,tgtpos,tgtvel,fc,fs);


nf          = 0;                                   
fast_time   = 0:1/fs:1/prf-1/fs;                   
range_gates = physconst('LightSpeed')*fast_time/2; 
pulses      = zeros(numel(fast_time),nint);        
intpulses   = zeros(numel(fast_time),1);
[sCol,sRx,sRD,sMFilt,sTVG,threshold] = setupRx(nint,nf,pfa,maxrange,range_gates,sWav,sAnt,fc);

rsig = zeros(336,3);
ang  = zeros(2,3);
for m = 1:2000
    [s,tx_status] = step(sTx,step(sWav));               
    [apos,avel]   = step(sAntPlat,1/prf);               
    for n = 1:3                                        
        [tpos,tvel]   = step(sTgtMotion{n},1/prf);      
        [~, ang(:,n)] = rangeangle(tpos,apos);          
        tsig          = step(sRad,s,ang(:,n));          
        tsig          = step(sChan{n},tsig,apos,tpos,avel,tvel); 
        rsig(:,n)     = step(sTgt{n},tsig,true);      
    end
    rsig = step(sCol,rsig,ang);                         
    rsig = sum(rsig,2);                                
    nn   = mod(m-1,nint)+1;
    pulses(:,nn) = step(sRx,rsig,~(tx_status>0));       
    pulses(:,nn) = step(sTVG,pulses(:,nn));             
    [rdmap,rgrid,sgrid] = step(sRD,pulses,sMFilt.Coefficients); 
    pulses(:,nn) = step(sMFilt,pulses(:,nn));           
    if nn == nint
        intpulses  = pulsint(pulses,'noncoherent');    
        [pmax,detect] = findpeaks(intpulses,'MinPeakHeight',sqrt(threshold)); 
        tgtrange   = range_gates(detect-(numel(sMFilt.Coefficients)-1));      
    end
    viewSignals
end