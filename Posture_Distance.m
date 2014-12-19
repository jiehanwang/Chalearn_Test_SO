function [maxWeight,maxLengthAll] = Posture_Distance(xx,x,yy,y)
    if x==0 || y==0  %No key frames in both
		error('beep');
    end
    
	if x>15 || y>15 %Number of key frames>15 will be consider as abnormal.
		error('beep');
	end
    
    for i=1:x
		maxLength(i) = 1;
    end
    
    %% Calculate the maximum weight and its order.
	for i=1:x
		maxMatch = 0.0;
        for j=1:y
			tempMatch = 10-abs(xx(i)-yy(j));%Histogram(xx(i),yy(j));
            if tempMatch>maxMatch
				maxMatch = tempMatch;
				order_temp(i)= j;
            end
        end
		weight_temp(i) = maxMatch;
	end
    
    count = 1;
    for i=1:x
        if weight_temp(i)>9
            weight(count) = weight_temp(i);
            order(count) = order_temp(i);
            count = count +1;
        end
    end
    x=count-1;
    
    %% Obtain the maxLength for each Index
	for i=2:x
        for j=i-1:-1:1
            if order(i) > order(j)
				maxLength(i) = maxLength(j)+1;
				break;
            end
        end
	end
    
    %% Get the max length of all
	maxLengthAll = 0;
	for i=1:x
        if maxLength(i) > maxLengthAll
			maxLengthAll = maxLength(i);
        end
	end
    
    groupNo = ones(1,maxLengthAll);
    routeNo = 1;
    for i=1:maxLengthAll
        for j=1:x
            if maxLength(j) == i
				%group((i-1)*x + groupNo(i)+1)=j;
                group(i,groupNo(i)) = j;
				groupNo(i) = groupNo(i) + 1;
            end
        end
		routeNo = routeNo * groupNo(i);
    end
    
    step=1;
    for i=1:maxLengthAll
        for j=1:routeNo
			jChange = (j-1)/step;
			jFloor = floor(jChange);
            groIndex = mod(jFloor,groupNo(i)-1);
			routeAll(j,i) = group(i,groIndex+1);
        end
		step = groupNo(i);
    end
    
    %% 
    maxWeight = 0.0;
    for i=1:routeNo
		useful = 1;
		weightTempSum = 0.0;
        for j=1:maxLengthAll-1
			former = routeAll(i,j);
			latter = routeAll(i,j+1);
			weightTempSum = weightTempSum + weight(former);
            if former>latter || order(former)>order(latter)
				useful = 0;
            end
        end
        if useful == 1
			weightTempSum = weightTempSum + weight(maxLengthAll);
            if weightTempSum > maxWeight
				maxWeight = weightTempSum;
            end
        end
    end
    
    %%
    maxWeight = maxWeight / maxLengthAll;
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
			
		
	
    
    
	