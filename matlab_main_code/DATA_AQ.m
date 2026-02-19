function [] = DATA_AQ(speed, angles, twind)
if nargin < 3 || isempty(twind)
    twind = 10;  % ברירת מחדל כמו שהיה
end

%function [] = DATA_AQ(speed,angles)

delete(instrfindall)
pointsNum = 90001;
v = 90001;
fCarrier = [3.35e9 10e9];
%twind = 10;
t11 = linspace(0,twind,pointsNum);
power = [5];
BW = [30];
[PNA] = openDevices();
Sample = 1:1;
Foil = '00000000';%edit_name
nummotor=4;
hovering = ([num2str(angles(3)) 'deg_Horizontal_' num2str(angles(1)) 'deg_vertical']);
for kf = 1:length(fCarrier)
    for k1 = 1:length(Sample)
        clear Sig
            configDevices( PNA,twind,power,BW,fCarrier(kf),pointsNum)
            [ S11Re, S11Im] = initDevices( PNA,pointsNum );
            pause(.1)
            Sig = [ S11Re(1:v)', S11Im(1:v)' ]; 
            pause(.1)
            save(['StageCOM_' hovering '_speed_' num2str(speed) '_Rotors_Foil_' Foil '_num_motors_active_' num2str(nummotor) '_Freq_' num2str(fCarrier(kf)/1e9,'%03.03f')...
                '__Sample_' num2str(Sample(k1),'%.0f') '.mat'],'Sig');
            % save(['FS__Freq_' num2str(fCarrier(kf)/1e9,'%03.03f')...
            %     '__Sample_' num2str(Sample(k1),'%.0f') '.mat'],'Sig');
            % save(['Ref_DJI_Flying_Freq_' num2str(fCarrier(kf)/1e9,'%03.03f')...
            %     '__Sample_' num2str(Sample(k1),'%.0f') '.mat'],'Sig');
            pause(6)

        s = abs(complex(S11Re,S11Im));
        s_fft = fft(s);
        s_fft_abs = abs(s_fft);
        fmax = 1/abs(t11(1)-t11(2));
        f = linspace(0,fmax,length(s_fft));
        
        figure(2)
        hold on
        plot(f,s_fft_abs,'-')
        grid on
%         axis([0 fmax/2 0 0.25])
        axis([0 2000 0 0.08])
        set(gca,'FontSize', 18);
        xlabel('FFT frequencies, Hz')
        ylabel('Amplitude')
%         title('Carrier 12 GHz')
        pause(5)

    end

end
end
