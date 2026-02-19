function [] = DATA_AQ_Parallel(speed, angles, twind, conveyor_position)
% DATA_AQ_Parallel
% Modified for background execution:
% 1. No "delete(instrfindall)" (Safe for Drone)
% 2. No Plotting (Safe for Background Worker)
% 3. Added Initial Pause (Sync with Drone)
% 4. Added Optional Conveyor Position parameter for filename

    % --- INPUT HANDLING ---
    if nargin < 3 || isempty(twind)
        twind = 10;
    end
    
    % Check if conveyor_position was passed
    conveyorString = '';
    if nargin >= 4 && ~isempty(conveyor_position)
        % Create the suffix string: "_ConveyorPos<Value>"
        conveyorString = ['_ConveyorPos' num2str(conveyor_position)];
    end
    
    % --- SYNC PAUSE ---
    % Wait 3 seconds for the drone (started simultaneously) 
    % to reach its target angle before we begin recording.
    pause(3.0); 
    
    % --- SETUP ---
    pointsNum = 90001;
    v = 90001;
    fCarrier = [3.35e9];
    power = [5];
    BW = [30];
    Foil = '111111111111';
    nummotor = 4;
    hovering = ([num2str(angles(3)) 'deg_Horizontal_' num2str(angles(1)) 'deg_vertical']);
    
    % Open PNA (Ensure openDevices doesn't use instrreset/delete)
    [PNA] = openDevices(); 
    Sample = 1:1;
    
    try
        for kf = 1:length(fCarrier)
            for k1 = 1:length(Sample)
                
                % Configure & Init
                configDevices(PNA, twind, power, BW, fCarrier(kf), pointsNum);
                [S11Re, S11Im] = initDevices(PNA, pointsNum);
                
                pause(.1);
                Sig = [S11Re(1:v)', S11Im(1:v)']; 
                pause(.1);
                
                % Save Data
                % Added conveyorString to the end of the filename
                filename = ['StageCOM_' hovering 'speed' num2str(speed) ...
                            'Rotors_Foil' Foil 'num_motors_active' num2str(nummotor) ...
                            'Freq' num2str(fCarrier(kf)/1e9,'%03.03f') ...
                            '_Sample' num2str(Sample(k1),'%.0f') ...
                            conveyorString '.mat'];
                            
                save(filename, 'Sig');
                
                % Processing time buffer
                pause(6);
                
                % --- NO PLOTTING IN BACKGROUND ---
                % Plotting commands removed to prevent crashes.
                % If you need live plots, you must perform PNA in the main thread.
            end
        end
    catch ME
        warning('Error during PNA Acquisition: %s', ME.message);
    end
    
    % Clean up ONLY the PNA object
    % Do NOT use delete(instrfindall)
    try
        fclose(PNA);
        delete(PNA);
    catch
        % Handle cases where PNA is struct or not open
    end
end