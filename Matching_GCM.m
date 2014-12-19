clear all;
clc;
addpath(genpath('D:\iCode\GitHub\libsvm\matlab'));
dir = 'output\';
nClass = 20;
nSample = 17;
totalN = 1;
nSub = 10;

% Readin the data
for i=1:nClass
    fprintf('Readin Data: %d / %d\n', i, nClass);
    nTrain = 1;
    for s=1:nSample
        fileName = sprintf('%s%02d\\%02d_Sample000%02d.txt',dir, i, i,s+2);
        data = importdata(fileName, ' ', 1);
        
        Data{totalN} = construct_subspace((data.data)',nSub);
        Label(totalN,1) = i;
        totalN = totalN + 1;
    end
end


for testID = 1:nSample
    testData = cell(nClass,1);
    trainData = cell((nSample-1)*nClass, 1);
    testLabel = zeros(nClass,1);
    trainLabel = zeros((nSample-1)*nClass, 1);
    obt = 1;
    testN = 1;
    trainN = 1;

    for i=1:nClass
        for s=1:nSample
            if s == testID
                testData{testN} = Data{obt};
                testLabel(testN,1) = Label(obt,1);
                testN = testN + 1;
                obt = obt+1;
            else
                trainData{trainN} = Data{obt};
                trainLabel(trainN,1) = Label(obt,1);
                trainN = trainN + 1;
                obt = obt+1;
            end
        end
    end
    
    trainN = trainN - 1;
    testN = testN - 1;

    TrainKernel = kernel(trainData,[],testID);
    ValKernel = kernel(trainData,testData,testID);

    TTrainKernel = [(1:trainN)',TrainKernel];
    VValKernel = [(1:testN)',ValKernel'];

    model_precomputed = svmtrain(trainLabel, TTrainKernel, '-t 4');
    [predict_label_P1, accuracy_P1, dec_values_P1] = svmpredict(testLabel, VValKernel, model_precomputed);
    accrant(testID) = accuracy_P1(1)/100;
end

for i=1:nSample
    fprintf('%f\n', accrant(i));
end



