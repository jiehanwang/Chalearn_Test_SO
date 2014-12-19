clear all;
clc;
meaning_ID = importdata('meaning_ID.txt');

for k=1:20
    if k<10
        folderName=['output\0' num2str(k)];
    else
        folderName=['output\' num2str(k)];
    end
    mkdir(folderName);
end

myDir = 'data\';
DIRS = dir([myDir, '*.zip']);
n=length(DIRS);
for i=1:n
    if ~DIRS(i).isdir
        fileName=DIRS(i).name;
        fileRouteName = [myDir fileName];
        ChanFeatureExtra(fileRouteName, meaning_ID);
    end
end