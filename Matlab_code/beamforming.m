
fc    = 3e9;                      
sAnt  = lowProfileArray('FrequencyRange',[2/3*fc 4/3*fc],'ViewArray',false); 
[sSAnt, sSAntPlat]= splitSubArrays(sAnt);
for i=1:4,
    sCol{i}  = phased.Collector('Sensor',sSAnt{i},'OperatingFrequency',fc); 
    sBF{i}   = phased.PhaseShiftBeamformer('OperatingFrequency',fc,...
                        'SensorArray',sSAnt{i},'DirectionSource','Input port',...
                        'WeightsOutputPort',true); 
    sResp{i} = phased.ArrayResponse('SensorArray',sSAnt{i},'WeightsInputPort',true);
end

sTgtMotion = phased.Platform('InitialPosition',[10e4; 0; 5e4],...
                             'Velocity',[-400; -200; -40]);
s=sin(2*pi*5*(0:.01:1))';   


N = 500;
scanAz = -180:180;
for ii=1:N
    [AzEl,face,antpos,tgtpos,range] = updatePos(sSAntPlat,sTgtMotion,1/N,ii);
    viewTrajectories                               
    y = []; resp = []; chan = 0;                   
    for k=find(face),
        x = step(sCol{k},s,AzEl);                  
        noise = 0.01*(randn(size(x)));             
        chan = chan + 1;
        [y(:,chan),w] = step(sBF{k},x+noise,AzEl); 
        ang = [scanAz;ones(size(scanAz))*AzEl(2)];
        resp(:,chan) = step(sResp{k},fc,ang,w);    
    end
    y = mean(y,2);                                
    resp = mean(resp,2);
    plotBeam                                       
end


