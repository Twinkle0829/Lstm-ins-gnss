


%% **********************************
% Sequence-to-Sequence Regression Using Deep Learning (LSTM)
% The HRV-SLEEP datacontains 45 columns,and the columns correspond to the following:
% Column 1-44: features
% Column 45: values
% Author: Cheng Kang, kangkangsome@gmail.com, 2019/07/11

% http://www.dpmi.tu-graz.ac.at/~schloegl/
%% **********************************

%function [net, idxConstant, mu, sig] = lstm_regression(XTrain, YTrain)
%function [net,Yp,Yt] = lstm_regression(trainnum,numnn,lstmdata)
load lstmdata.mat
numnn=1000;
trainnum=500;
for i=1:size(lstmdata,1)
ddata.X{1,i}=lstmdata(i,1:13)';
ddata.Y{1,i}=lstmdata(i,14:16)';
end

%k=rand(1,16000);
%[m,n]=sort(k);
%numnn=10000;
for i = 1:numnn
%XTrain{i}=ddata.X{1,n(i)};
%YTrain{i}=ddata.Y{1,n(i)};   
XTrain{i}=ddata.X{1,i};
YTrain{i}=ddata.Y{1,i};  
end
for i = 1:trainnum
% XTest{i}=ddata.X{1,n(i+numnn)};
% YTest{i}=ddata.Y{1,n(i+numnn)};   
  XTest{i}=ddata.X{1,i+numnn};
 YTest{i}=ddata.Y{1,i+numnn}; 
end

TrainData_length = size(XTrain);
mu=0;
sig=0;
for l = 1:TrainData_length(2)
    mu = mu + mean([XTrain{l}], 1);
    sig = sig + std([XTrain{l}], 0, 1);
end
%%

mu = mu / TrainData_length(2);
sig = sig / TrainData_length(2);

for i = 1:numel(XTrain)
    XTrain{i} = (XTrain{i} - mu) ./ sig;
end

% avoid more horizontal lines
% thr = 150;
% for i = 1:numel(YTrain)
%     YTrain{i}(YTrain{i} > thr) = thr;
% end

% Prepare Data for Padding

%% Define Network Architecture
numResponses = size(YTrain{1},1);
featureDimension = size(XTrain{1},1);
numHiddenUnits =64;
%gm = fitgmdist£¨dataToFit'£¬2£¬'Options'£¬options£¬'Replicates'£¬2£¬'Start'£¬'plus'£©;
layers = [ ...
    sequenceInputLayer(featureDimension)
    lstmLayer(numHiddenUnits,'OutputMode','sequence')
    fullyConnectedLayer(100)
    fullyConnectedLayer(numResponses)
    softmaxLayer
    regressionLayer];

maxEpochs = 100;
miniBatchSize = 30;

options = trainingOptions('adam', ...
    'MaxEpochs',maxEpochs, ...
    'MiniBatchSize',miniBatchSize, ...
    'InitialLearnRate',0.01, ...
    'LearnRateDropFactor',0.1, ...
    'LearnRateDropPeriod',60, ...
    'GradientThreshold',1, ...
    'SequenceLength','longest', ...
    'Shuffle','never', ...
    'Plots','training-progress',...
    'Verbose',0);

% Train the Network
net = trainNetwork(XTrain,YTrain,layers,options);

for i = 1:numel(XTest)
    XTest{i} = (XTest{i} - mu) ./ sig;
end
YPred = predict(net, XTest, 'MiniBatchSize',1);
Ypred=YPred';

for i = 1:size(Ypred,2)
Yt(i,:)=YTest{i};
Yp(i,:)=double(Ypred{i});
end
%Lp=Yp(:,2);
%Ep=Yp(:,1);
Hp=Yp(:,3);
%Lt=Yt(:,2);
%Et=Yt(:,1);
Ht=Yt(:,3);
plot(Hp,'DisplayName','Hp');hold on;plot(Ht,'DisplayName','Ht');hold off;


%% Example Functions
% The function prepareDataTrain extracts the data from filenamePredictors and returns the cell arrays XTrain and YTrain which contain the training predictor and response sequences, respectively.
% The data contains zip-compressed text files with 26 columns of numbers, separated by spaces. Each row is a snapshot of data taken during a single operational cycle, and each column is a different variable. The columns correspond to the following:
   
%end
