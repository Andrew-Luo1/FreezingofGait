Data01 = csvread('11-06-2017_Freezing_01.csv',1);

time1   = Data01(:,1);
data1   = Data01(:,7);       % data1 is parameter values for dataset 1;

freqCuttoff1    = 20;   % value obtained from literature
freqCuttoff2    = 20;   % for the new thing
samplingFreq    = 100;  % IMU sampling frequency
coefficienct    = 4;    % artificially magnifying the sampling frequency

[b,a] = butter(6,freqCuttoff1/(samplingFreq * coefficienct));

data1_1 = (data1.^2)/2 + data1;


%transforming - butterworth
data1_2 = filter(b,a,data1_1);


%transforming - weighted moving average
elapsed = 1/samplingFreq;
tauUS = 1/freqCuttoff2;
ampFactor = exp( (-1)*elapsed/tauUS );
data1_3 = [];
data1_3(1) = data1_1(1);

for i = 2:length(data1_1)
    data1_3(i) = (1 - ampFactor)*data1_1(i) + (ampFactor)*data1_3(i-1);
end


%transforming - Butterworth
data1_4 = Butterworth( data1_1, 0.01, 20 );

%plotting
figure(1)
plot(time1, data1)
figure(2)
plot(time1, data1_2)
figure(3)
plot(time1, data1_3)
figure(4)
plot(time1, data1_4)

%Butterworth - code from https://www.codeproject.com/Tips/1092012/A-Butterworth-Filter-in-Csharp
function outData = Butterworth( inData, deltaTimeInSec, cutOff )
    
    samplingRate = 1 / deltaTimeInSec;
    dF2 = length(inData) - 1;   % data range
    Dat2 = zeros(1, dF2+4);
    outData = inData;
    
    for i = 1:dF2
        Dat2(2 + i) = inData(i);
    end
    
    Dat2(1) = inData(1);
    Dat2(2) = inData(1);
    Dat2(dF2+3) = inData(dF2+1);
    Dat2(dF2+4) = inData(dF2+1);
    
    wc = tan( cutOff * pi() / samplingRate );       % radians or degrees?
    k1 = 1.414213562 * wc;
    k2 = wc * wc;
    a = k2 / (1 + k1 + k2);
    b = 2 * a;
    c = a;
    k3 = b / k2;
    d = -2 * a + k3;
    e = 1 - (2 * a) - k3;
    
    DatYt = zeros(1, dF2+4);
    DatYt(1) = inData(1);
    DatYt(2) = inData(1);
    for j = 3:(dF2+2)
        DatYt(j) = a*Dat2(j) + b*Dat2(j-1) + c*Dat2(j-2) + d*DatYt(j-1) + e*DatYt(j-2);
    end
    DatYt(dF2+3) = DatYt(dF2+2);
    DatYt(dF2+4) = DatYt(dF2+2);
    
    DatZt = zeros(1, dF2+2);
    DatZt(dF2+1) = DatYt(dF2+3);
    DatZt(dF2+2) = DatYt(dF2+4);
    for k = dF2:-1:1
        DatZt(k) = a*DatYt(k+2) + b*DatYt(k+3) + c*DatYt(k+4) + d*DatZt(k+1) + e*DatZt(k+2);
    end
    
    for l = 1:dF2
        outData(l) = DatYt(l);
    end
end