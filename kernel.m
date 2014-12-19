function outKernel = kernel(SY1,SY2,testID)

if (nargin<1)
    error('Not enough inputs');
end

number_sets1 = length(SY1);

if (isempty(SY2)~=1)
    number_sets2 = length(SY2);
    trainFlag = 0;    %1: traing. 0:test
else
    SY2 = SY1;
    number_sets2 = length(SY2);
     trainFlag = 1;
end

outKernel = zeros(number_sets1,number_sets2,1);
%%
for tmpC1 = 1:number_sets1
    if trainFlag == 0
        fprintf('Test kernel--%d------%d/%d\n', testID, tmpC1,number_sets1);
    else
        fprintf('Training kernel--%d------%d/%d\n', testID, tmpC1,number_sets1);
    end
    Y1 = SY1{tmpC1};
    for tmpC2 = 1:number_sets2
        Y2 = SY2{tmpC2};
        if(isempty(Y1)~=1 && isempty(Y2)~=1)

            % CTC
            tmpMatrix = Y1'*Y2;
            outKernel(tmpC1,tmpC2) = sum(sum(tmpMatrix.^2));

        else
            outKernel(tmpC1,tmpC2) = 0;
        end
    end
end