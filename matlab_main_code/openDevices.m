function [PNA] = openDevices()

PNA = instrfind('Type', 'visa-usb', 'RsrcName', 'USB0::0x2A8D::0x2C01::MY57181148::0::INSTR', 'Tag', '');

if isempty(PNA)
    PNA = visa('KEYSIGHT', 'USB0::0x2A8D::0x2C01::MY57181148::0::INSTR');
else
    fclose(PNA);
    PNA = PNA(1);
end
fopen(PNA);

% fprintf(PNA,'SYSTem:FPReset');

pause(0.2)

end