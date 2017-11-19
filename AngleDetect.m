clear all;
close all;

values = load("6-6-2-new.csv");
dax = movmean(values(:,2),3);
day = movmean(values(:,3),3);
daz = movmean(values(:,4),3);
time = values(:,1);

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
        
        if(arduinoTime(1)-arduinoTime(4) < (delayTime * 10)) %10 was arbitrarily chosen. 
            
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
gaitLength = zeros(474, 1);    %gait length variable
maxVel = 0;
maxVelArr = zeros(474, 1);
maxVelInd = zeros(474, 1); 
for k = 2:474
    if (abs(angle(k))<0.05)
        zeroAngleTime = isStep(time(k), time(k)-prevTime, zeroAngleTime);
        if(zeroAngleTime(5))
            vel(k) = 0;
            if (initialTime==0)
                initialTime = time(k);
                continue;
            end
            gaitLength(k) = (sumVel/nVel)*(time(k)-initialTime);
            initialTime = time(k);
            sumVel = 0;                                                                                                                                                                                  
            nVel = 0;
            maxVelArr(k-1) = maxVel;
            maxVelInd(k-1) = k-1;
            maxVel = 0;
        else
            % angle var is the angle between the x' and x axes
            nax = dax(k) - 16384*cos(angle(k)); %400 is calibration for constant error
            nay = day(k) + 16384*sin(angle(k));
            hay = nay*cos(angle(k)) + nax*sin(angle(k));
            vel(k) = vel(k-1) + hay*(time(k)-time(k-1));

            %update velocity sum and count
            if (vel(k)>maxVel)
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
        vel(k) = vel(k-1) + hay*(time(k)-time(k-1));

        %update velocity sum and count
            if (vel(k)>maxVel)
               maxVel = vel(k);
            end
            sumVel = sumVel + vel(k);
            nVel = nVel + 1;  
    end
    %update prevTime
    prevTime = time(k);
end

figure
plot(angle, 'x');
 
figure
plot(vel, 'x');
% k=0;
% for k = 1:328
%     if (maxVelInd(k)~=0)
%         hold on;
%         plot(maxVelInd(k), maxVelArr(k), 'rx');
%     end 
% end
% hold on;
% plot(198, maxVel(198), 'rx')
% hold on;
% plot(255, maxVel(255), 'rx')
% hold on;
% plot(260, maxVel(260), 'rx')
% hold on;
% plot(261, maxVel(261), 'rx')
% hold on;
% plot(303, maxVel(303), 'rx')
% hold on;
% plot(304, maxVel(304), 'rx')
%hold on;
%hold on;
%plot(time, gaitLength, 'o');

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
