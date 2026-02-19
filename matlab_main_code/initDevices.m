function [ S11Re, S11Im, t11] = initDevices( PNA,pointsNum )

t11=0;
ComplexDataBlockNum = ceil(pointsNum*2*20/512);
fprintf(PNA,'INITiate:IMMediate;*wai');
S11 = readBlock(PNA,ComplexDataBlockNum,'CALCulate1:MEAS1:DATA:SDATA?');
S11Re = S11(1:2:end-1);
S11Im = S11(2:2:end);
% S11Am = sqrt(S11im.^2+S11re.^2);
% S11Ph = atan(S11im./S11re);
DataBlockNum = ceil(pointsNum*20/512);
t11 = readBlock(PNA,DataBlockNum,'CALCulate1:X?');

% pause(3)

end

