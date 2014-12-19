clear all;
clc;
dir = 'output\';
nClass = 20;
nSample = 5;

% Readin the data
for i=1:nClass
    fprintf('Readin Data: %d / %d\n', i, nClass);
    nTrain = 1;
    for s=1:nSample
        fileName = sprintf('%s%02d\\%02d_Sample0000%d.txt',dir, i, i,s+2);
        D(i).info{s} = importdata(fileName, ' ', 1);
    end
end

testSample = 1;
for testSample = 1:5
    correctRate(testSample) = 0;
    tic;
    for i = 1: nClass
        fprintf('Compare: P%d------%d\n', testSample, i);
        for j = 1: nClass
            dist(i,j) = 0;
            for g=1:nSample
                if g~=testSample
                    dist(i,j) = dist(i,j) + dtw_fast(D(j).info{1,g}.data, D(i).info{1,testSample}.data);
                end
            end
            dist(i,j) = dist(i,j)/4;
        end
        [sA,index] = sort(dist(i,:));

        if index(1) == i
            correctRate(testSample) = correctRate(testSample) + 1;
        end
    end
    toc;
end

for testSample = 1:nSample
    fprintf('P%d %f\n',testSample, correctRate(testSample)/nClass);
end
