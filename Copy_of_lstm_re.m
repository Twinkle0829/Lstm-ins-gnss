
%function[imudata,gnsstest,gnsspre] = lstm_re(trainnum,testnum)
Data= importdata('lstm_data.mat');
inputdata=Data(:,2:end-3);
outputdata=Data(:,end-2:end);
trainnum=5000;
testnum=1000;
numm=trainnum+testnum;
%k=rand(1,numm);
%[m,n]=sort(k);
%ѵ������
input_train=inputdata(1:trainnum,:)';
output_train=outputdata(1:trainnum,:)';   
[inputd,inputpstr]=mapminmax(input_train);          %�����һ��
[outputd,outputpstr]=mapminmax(output_train);  
XTrain =inputd;
YTrain =outputd;
%��������
net = lstm_resss(inputd, outputd);
test_input=inputdata(trainnum:numm,:)';
test_output=outputdata(trainnum:numm,:)';
aa=test_output;
%inputn_test=mapminmax('apply',input_test,inputps); %����inputps��ʽ���й�һ��
inputest=mapminmax('apply',test_input,inputpstr);          %�����һ��
%[outputesn,outputpns]=mapminmax(test_output);    %�����һ��
tn_sim1 = predict(net,inputest);
  % tn_sim = elmpredict(inputn_test,IW,B,LW,TF,TYPE);
T_sim1=mapminmax('reverse',tn_sim1,outputpstr);
Yt11=aa(1,:)'; Yp11=T_sim1(1,:)';
Yt21=aa(2,:)'; Yp21=T_sim1(2,:)';
Yt31=aa(3,:)'; Yp31=T_sim1(3,:)';
%imudata=test_input;
%gnsstest=test_output;
 plot(Yt11,'DisplayName','Yt1');hold on;plot(Yp11,'DisplayName','Yp1');hold off;
%%

















