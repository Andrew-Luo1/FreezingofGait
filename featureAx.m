pts=zeros(1,5);
diffs=zeros(1,4);
ind=1;
isDecrease=true;

for i = 1:328    
   if index==5
       pts(1)=pts(2);
       pts(2)=pts(3);
       pts(3)=pts(4);
       pts(4)=pts(5);
       pts(5)=ax(i);
   end
   
   if ax(i)<=10000
     pts(index)=ax(i);
     index=index+1;
   end
   
   diffs(1)=pts(2)-pts(1);
   diffs(2)=pts(3)-pts(2);
   diffs(3)=pts(4)-pts(3);
   diffs(4)=pts(5)-pts(4);
   for j = 1:4
       if diffs(j)<=0
           isDecrease=false;
       end
   end
   
   if isDecrease==true
       for k = 1:4
          if true
              %CALCULATE SLOPE
              %COMPARE SLOPE TO CRITICAL VALUE 
              %DETERMINE IF IS THE LOWEST TROUGH
              %FIND LOWEST TROUGH TIMESTAMP
              %RECORD TIMESTAMP TO CALCULATE DIFFERENCE IN TIME
          end
       end
   end
end