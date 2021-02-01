function tstlstm=lstm_ddata change(lstm_ddata ,l)

imu = importddata ('nav_ekinox.mat');
dddata  =  importddata ('nav_ekinox.mat');
gnss = importddata ('nav_ekinox.mat');
%l= size(ddata .t);

for i=1:l
tstlstm(i,1)=dddata .t(i,1);
tstlstm(i,2)=ddata .wb(i,1);
tstlstm(i,3)=ddata .wb(i,2);
tstlstm(i,4)=ddata .wb(i,3);
tstlstm(i,5)=ddata .fb(i,1);
tstlstm(i,6)=ddata .fb(i,2);
tstlstm(i,7)=ddata .fb(i,3);
tstlstm(i,8)=ddata .roll_e(i,1);
tstlstm(i,9)=ddata .yaw_e(i,1);
tstlstm(i,10)=ddata .pitch_e(i,1);
tstlstm(i,11)=ddata .vel(i,1);
tstlstm(i,12)=ddata .vel(i,2);
tstlstm(i,13)=ddata .vel(i,3);
tstlstm(i,14)=ddata .gnsslat(i,1);
tstlstm(i,15)=ddata .gnsslon(i,1);
tstlstm(i,16)=ddata .h(1,i)-ddata .gnssh(i,1);
end

%load tstlstm
%j=150;
%for i=1:150
%ddata a.xtrain{1,i}=tstlstm(i+1,1:13)';
%ddata a.ytrain{1,i}=tstlstm(i+1,14:16)';
%ddata a.train{1,i}=tstlstm(i+1,:)';
%end

%for i=150:(size(tstlstm,1)-1)
%ddata a.xtest{1,i-149}=tstlstm(i+1,1:13)';
%ddata a.ytest{1,i-149}=tstlstm(i+1,14:16)';
%ddata a.ttest{1,i-149}=tstlstm(i+1,:)';
%end
%save ddata a.mat ddata a
load lstm_data.mat
l=size(lstm_data,1);

lstm.t=lstm_data(:,1);
lstm.wb=lstm_data(:,2:4);
lstm.fb=lstm_data(:,5:7);
lstm.roll=lstm_data(:,8);
lstm.pitch=lstm_data(:,9);
lstm.yaw=lstm_data(:,10);
lstm.vel=lstm_data(:,11:13);
lstm.lat=lstm_data(:);