clear all;
close all;

values = load("6-6-2-new.csv");
dax = movmean(values(:,2),3);
day = movmean(values(:,3),3);
daz = movmean(values(:,4),3);
time = values(:,1);

fgz = filter(values(:,7));
figure
plot(fgz);

x = length(dax);
for i=1:x
    angle(i) = (acos(day(i)/ sqrt((dax(i)*dax(i)) + (day(i)*day(i)) + (daz(i)*daz(i)))) - pi/2);
end

% Code for angle calibration
cali = mean(angle(1,1:40));
angle = angle - cali;


figure(2);
plot(time, angle)

hold on

%find indicies where angle is near zero. 
j = 1;  %iterator for the near zero angle indexe s. 
q = 1; %iterator for stride time
angInd = 0;

arduinoTime = zeros(1,4);

for i=2:x
    if (abs(angle(i)) < 0.02) %This changes based on sampling rate.
        angInd(j) = i;
        j = j+1;
        delayTime = time(i) - time(i-1);
        arduinoTime(4) = arduinoTime(3); %lil switcharoo. 
        arduinoTime(3) = arduinoTime(2);
        arduinoTime(2) = arduinoTime(1); 
        arduinoTime(1) = time(i);
        ignoreClose = arduinoTime(1)-arduinoTime(2);
        
        if(arduinoTime(1)-arduinoTime(4) < (delayTime * 5)) %10 was arbitrarily chosen. 
            
            for t = 1:4
                strideIndex(q) = i; %for visualization purposes. 
                strideTime(q) = ((arduinoTime(1)-arduinoTime(4))/4);
                q = q+1;
            end
            continue;
        end

        if(ignoreClose < delayTime * 6 )% sampling rate dependent. 
            continue;               
        else  
            strideTime(q) = arduinoTime(1)-arduinoTime(2);
            strideIndex(q) = i;
            q = q+1;        
        end
            
    end
end

%feature 1: Gait length -> need to calculate velocity and average velocity 
%feature 2: Number of times you reach near-zero velocity in a given window
%of time( you decide window length)

plot(time, 0.02);
hold on;
plot(time, -0.02);
hold on;
for i=1:length(angInd)
    plot(time(angInd(i)), angle(angInd(i)), 'r*')
    hold on
end

for i=1:length(strideTime)
    plot(time(strideIndex(i)), angle(strideIndex(i)), 'g*')
    hold on
end

%plot every instance the angle becomes zero. 


% Velocity
% Find horizontal accleration
% Find time difference
% Use initial velocity plus accleration times time difference
gz = values(:,7);


vel = zeros(474, 1);          %gait length parameter variables
initialTime = 0;
sumVel = 0;
nVel = 0;
prevTime = 0;                   %isStep method variables
zeroAngleTime = zeros(5, 1);
gaitLength = zeros(474, 1);    %gait length (f1) variable 
maxVel = 0;                    %max velocity (f2) variable
maxVelArr = zeros(474, 1);
maxVelInd = zeros(22, 1); 

svmArr = zeros(474, 2);       %input variable for SVM with gait length (col1) and max velocity (col2)
index = 1;
for k = 2:474
    if (abs(angle(k))<0.02)
        zeroAngleTime = isStep(time(k), time(k)-prevTime, zeroAngleTime);
        if(zeroAngleTime(5))
            vel(k) = 0;
            if (initialTime==0)
                initialTime = time(k);
                continue;
            end
            
            if (maxVel==0) 
                continue;
            end
            
            gaitLength(k) = (sumVel/nVel)*(time(k)-initialTime);            
            maxVelArr(k-1) = maxVel;
            maxVelInd(index) = k-1;
            svmArr(index, 1) = (sumVel/nVel)*(time(k)-initialTime);
            svmArr(index, 2) = maxVel;
            
            initialTime = time(k);
            sumVel = 0;                                                                                                                                                                                  
            nVel = 0;
            maxVel = 0;
            index = index + 1;
        else
            % angle var is the angle between the x' and x axes
            nax = dax(k) - 16384*cos(angle(k)); %400 is calibration for constant error
            nay = day(k) + 16384*sin(angle(k));
            hay = nay*cos(angle(k)) + nax*sin(angle(k));
            vel(k) = vel(k-1) + hay*(time(k)-time(k-1));

            %update velocity sum and count
            if (vel(k)<maxVel)
               maxVel = vel(k);
            end
            sumVel = sumVel + vel(k);
            nVel = nVel + 1;     
        end
    else
        % angle var is the angle between the x' and x axes
        nax = dax(k) - 16384*cos(angle(k)); %400 is calibration for constant error
        nay = day(k) + 16384*sin(angle(k));
        hay = nay*cos(angle(k)) + nax*sin(angle(k));
        vel(k) = (vel(k-1) + hay*(time(k)-time(k-1)));

        %update velocity sum and count
            if (vel(k)<maxVel)
               maxVel = vel(k);
            end
            sumVel = sumVel + vel(k);
            nVel = nVel + 1;  
    end
    %update prevTime
    prevTime = time(k);
end

svmArrSize = 0;
for i=1:474
    if (svmArr(i, 2)~=0)
        svmArrSize = svmArrSize + 1;
    end
end
svmArrNew = zeros(svmArrSize, 2);
for i=1:svmArrSize
   svmArrNew(i, 1) = svmArr(i, 1);
   svmArrNew(i, 2) = svmArr(i, 2);
end
y = [1; 1; 1; 1; 1; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
SVM(svmArrNew, y, 1, 20);

%{
figure
plot(angle);
for i=1:22
    hold on;
    plot(maxVelInd(i), angle(maxVelInd(i)), 'rx');
end
%}

%figure
%plot(vel, 'x');

% plot f1 vs. f2 graph
% 'x' is walking
% 'o' is freezing
pos = find(y==1);
neg = find(y==0);
figure
plot(svmArrNew(pos,1), svmArrNew(pos,2), 'k+', 'LineWidth', 1, 'MarkerSize', 7);
hold on
plot(svmArrNew(neg,1), svmArrNew(neg,2), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 7);
    
function result = filter(data)
% Filter data using simple moving average filter
    samplingFreq = 100; % IMU sampling frequency
    freqCuttoff2 = 40;  % for the new thing; can be modified
    elapsed = 1/samplingFreq;
    tauUS = 1/freqCuttoff2;
    ampFactor = exp((-1)*elapsed/tauUS);
    data = (data.^2)/2 + data;
    result = zeros(length(data));   
    result(1) = data(1);
    for i = 2:length(data)
        result(i) = (1 - ampFactor)*data(i) + (ampFactor)*result(i-1);
    end
end

function result = isStep(time, gapTime, zeroAngleTime)
    zeroAngleTime(4) = zeroAngleTime(3);
    zeroAngleTime(3) = zeroAngleTime(2);
    zeroAngleTime(2) = zeroAngleTime(1);
    zeroAngleTime(1) = time; 
    if ((zeroAngleTime(1)-zeroAngleTime(4))<gapTime*10)     %identify shuffle
        zeroAngleTime(5) = true;
    elseif ((zeroAngleTime(1)-zeroAngleTime(2))>gapTime*6)  %identify walking
        zeroAngleTime(5) = true;
    else
        zeroAngleTime(5) = false;    
    end
    result = zeroAngleTime;
end

function SVM( X, y, C, max_iter )
%Graph the SVM, provide coefficients for the decision boundary. Max_iter
%can be around 20. X is a two column array containing feature values. y is
%a vector storing the class information. (0 or 1; freezing or normal)
    model = svmtrain(X, y, C, @linearKernel, 1e-3, max_iter);
    w = model.w;
    b = model.b;
    xp = linspace(min(X(:,1)), max(X(:,1)), 100);
    yp = - (w(1)*xp + b)/w(2);
    plotData(X, y);
    hold on;
    plot(xp, yp, '-b'); 
    hold off
    model.w
    model.b
end

% issue
% Several consecutive zero angles because of cluster
% current solution: ignore points when maxVel is zero


