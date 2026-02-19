function [ ] = configDevices( PNA, twind, power, BW, fCarrier, pointsNum)

fprintf(PNA, 'SYSTem:FPReset');
fprintf(PNA, 'CALCulate1:PARameter:DEFine:EXT ''Meas1'',''S11''');
fprintf(PNA, 'DISPlay:WINDow1:STATE ON');
fprintf(PNA, 'DISPlay:WINDow1:TRACe1:FEED ''Meas1''');
fprintf(PNA, 'CALCulate1:MEAS1:FORM MLIN');
fprintf(PNA, 'INITiate:CONTinuous OFF');

fprintf(PNA, 'SENS1:SWE:TYPE CW');
fprintf(PNA, ['SENS1:SWE:POIN ', num2str(pointsNum)]);

fprintf(PNA, ['SENS1:FREQ:FIXED ', num2str(fCarrier)]);
fprintf(PNA, 'SENS1:SWE:DWEL:AUTO OFF');
fprintf(PNA, ['SOURce:POWer ', num2str(power)]);
fprintf(PNA, ['SENS1:BWID ', num2str(BW), 'KHZ']);
fprintf(PNA, ['SENS1:SWE:TIME ', num2str(twind), ' s']);

% fprintf(PNA,'TRIG:SOUR INT');
fprintf(PNA, 'SENSe1:SWEep:MODE SINGle');
pause(1);

pause(twind);
pause(0.5);

end

