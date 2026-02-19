function Run_AutoMeas_Parallel()
% Run_AutoMeas_Parallel
% Executing Drone Flight and PNA Acquisition simultaneously.
clc;
%% ===================== Measurement Table =====================
% [ roll, pitch, yaw, measTime, conveyor_increment ]
measTbl = [ ...
      0,   -30,   0,   10,   0;
      0,   -30,   0.001,   10,   0;
      0,   -30,   0.002,   10,   0;
      0,     0,   0,   10,   0;
      0,     0,   0.001,   10,   0;
      0,     0,   0.002,   10,   0;
      0,    30,   0,   10,   0;
      0,    30,   0.001,   10,   0;
      0,    30,   0.002,   10,   0;
%       0,   -30,   0,   10,   2;
%       0,   -30,   0.001,   10,   0;
%       0,   -30,   0.002,   10,   0;
% %       0,     0,   0,   10,   0;
%       0,     0,   0.001,   10,   0;
%       0,     0,   0.002,   10,   0;
%       0,    30,   0,   10,   0;
%       0,    30,   0.001,   10,   0;
%       0,    30,   0.002,   10,   0;
%       0,   -30,   0,   10,   2;
%       0,   -30,   0.001,   10,   0;
%       0,   -30,   0.002,   10,   0;
%       0,     0,   0,   10,   0;
%       0,     0,   0.001,   10,   0;
%       0,     0,   0.002,   10,   0;
%       0,    30,   0,   10,   0;
%       0,    30,   0.001,   10,   0;
%       0,    30,   0.002,   10,   0;
      ];

%% ===================== General Options =====================
tolDeg          = 10;
maxRetries      = 3; 
attHold_s       = 25.0;  
speedForDAQ     = 0;

% --- INITIALIZE ACCUMULATOR ---
currentConveyorPos = 0; % Starts at 0

fprintf("=== AUTO PARALLEL RUN | total=%d | Hold=%.1fs ===\n", size(measTbl,1), attHold_s);

%% ===================== Run =====================
for k = 1:size(measTbl,1)
    rollDeg   = measTbl(k,1);
    pitchDeg  = measTbl(k,2);
    yawDeg    = measTbl(k,3);
    twind_s   = measTbl(k,4);
    
    % Get the INCREMENT from the table
    conveyorIncrement = measTbl(k,5);
    
    % Update the GLOBAL position (Accumulate)
    currentConveyorPos = currentConveyorPos + conveyorIncrement;
    
    angles    = [pitchDeg, rollDeg, yawDeg];
    fprintf("\n=== MEAS %d/%d (Parallel) ===\n", k, size(measTbl,1));
    fprintf("    Conveyor Status: Move by %.2f | Current Total Pos: %.2f\n", conveyorIncrement, currentConveyorPos);

    %% Stage 1: Conveyor (Sequential)
    % We actuate based on the INCREMENT (only if we need to move)
    if conveyorIncrement ~= 0
        fprintf("[1] Moving Conveyor by increment: %.2f\n", conveyorIncrement);
        if exist("servo_control_gui.m","file") == 2
            servo_control_gui(conveyorIncrement);
        end
    else
        fprintf("[1] Conveyor skip (No movement required)\n");
    end
    
    %% Stage 2 & 3: Parallel Execution
    fprintf("[2+3] Starting Parallel Ops: Drone Hold + PNA Record\n");
    ok = false;
    for a = 1:maxRetries
        
        % --- A. Start PNA in Background ---
        % UPDATED: We pass 'currentConveyorPos' (the accumulated value) to DATA_AQ
        fprintf("      -> Launching PNA background worker (Pos: %.2f)...\n", currentConveyorPos);
        
        % parfeval inputs: (Function, NumOutputs, speed, angles, twind, conveyor_pos)
        futurePNA = parfeval(@DATA_AQ_Parallel, 0, speedForDAQ, angles, twind_s, currentConveyorPos);
        
        % --- B. Start Drone in Main Thread ---
        fprintf("      -> Commanding Drone (Hold %ds)...\n", attHold_s);
        st = mav_roll_pitch_yaw_guidednogps(rollDeg, pitchDeg, yawDeg, attHold_s);
        
        % --- C. Sync and Check ---
        try
            fprintf("      -> Waiting for PNA to finalize...\n");
            fetchOutputs(futurePNA); 
            fprintf("      -> PNA Finished.\n");
        catch err
            fprintf("      ❌ PNA Failed: %s\n", err.message);
        end
        
        % Check Drone Attitude
        ok = attitudeWithinTol(st, tolDeg);
        
        if ok
            fprintf("    ✅ Drone Attitude confirmed within tolerance.\n");
            break;
        else
            fprintf("    ⚠️ Attitude Out of Tolerance. Retrying...\n");
        end
    end
    if ~ok
        fprintf("    ❌ Failed to reach attitude after retries.\n");
    end
    
    fprintf("✅ MEAS %d Done\n", k);
end
fprintf("\n=== RUN FINISHED ===\n");
end

function ok = attitudeWithinTol(st, tolDeg)
    ok = false;
    if isfield(st,"attitudeErrorDeg")
        e = st.attitudeErrorDeg;
        if isfield(e,"roll") && isfield(e,"pitch") && isfield(e,"yaw")
            ok = (abs(e.roll) <= tolDeg) && (abs(e.pitch) <= tolDeg) && (abs(e.yaw) <= tolDeg);
            return;
        end
    end
    if isfield(st,"attitudeObserved")
        ok = logical(st.attitudeObserved);
    end
end