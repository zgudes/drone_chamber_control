function [ data ] = readBlock( PNA,n,STR )
fprintf(PNA,STR);
data='';
for j=1:n
   data=[data,fscanf(PNA)];
end
data=str2num(data);
end

