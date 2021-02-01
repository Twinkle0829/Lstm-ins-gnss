


%% **********************************
% Sequence-to-Sequence Regression Using Deep Learning (LSTM)
% The HRV-SLEEP datacontains 45 columns,and the columns correspond to the following:
% Column 1-44: features
% Column 45: values
% Author: Cheng Kang, kangkangsome@gmail.com, 2019/07/11

% http://www.dpmi.tu-graz.ac.at/~schloegl/
%% **********************************

function [net,tr] = lstm_resss(XTr, YTr)

%% DenumResponses = size(YTr,1);
featureDimension = size(XTr,1);
numHiddenUnits = 128;


numResponses = size(YTr,1);
featureDimension = size(XTr,1);
load layers_1
layers = [ ...
    sequenceInputLayer(featureDimension)
    lstmLayer(numHiddenUnits,'OutputMode','sequence')
    fullyConnectedLayer(100) % fullyConnectedLayer(50) 
    dropoutLayer(0.5)
    fullyConnectedLayer(numResponses)
    regressionLayer];

maxEpochs =350;
miniBatchSize =64;

options = trainingOptions('adam', ...
    'MaxEpochs',maxEpochs, ...
    'MiniBatchSize',miniBatchSize, ...
    'InitialLearnRate',0.001, ...
    'LearnRateDropFactor',0.000000001, ...
    'LearnRateDropPeriod',150, ...
    'GradientThreshold',1, ...
    'SequenceLength','longest', ...
    'Shuffle','never', ... %
    'Plots','training-progress',...
    'Verbose',0);

% Train the Network
[net,tr] = trainNetwork(XTr,YTr,layers,options);

%[net2,tr] = trainNetwork(XTr,YTr,layers_1,options);
%% Example Functions
% The function prepareDataTrain extracts the data from filenamePredictors and returns the cell arrays XTrain and YTrain which contain the training predictor and response sequences, respectively.
% The data contains zip-compressed text files with 26 columns of numbers, separated by spaces. Each row is a snapshot of data taken during a single operational cycle, and each column is a different variable. The columns correspond to the following:
   
%end
