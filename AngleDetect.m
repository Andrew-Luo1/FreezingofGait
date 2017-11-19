%Some quick matlab code to calculate angles
clear all;
values = load("11-06-2017_Freezing_01.csv");
%values = load("healthGait2.csv");
dax = movmean(values(:,2),3);
day = movmean(values(:,3),3);
daz = movmean(values(:,4),3);
time = values(:,1);

x = length(dax);
for i=1:x
    angle(i) = (acos(day(i)/ sqrt((dax(i)*dax(i)) + (day(i)*day(i)) + (daz(i)*daz(i)))) - pi/2) * -1;
end

figure(2);
plot(time, angle)

hold on

%find indicies where angle is near zero. 
j = 1;  %iterator for the near zero angle indexes. 
q = 1; %iterator for stride time
angInd = 0;

arduinoTime = zeros(1,4);

for i=1:x
    if (abs(angle(i)) < 0.01) %This changes based on sampling rate.
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


