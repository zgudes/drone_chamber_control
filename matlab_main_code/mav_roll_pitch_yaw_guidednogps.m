function status = mav_roll_pitch_yaw_guidednogps(rollDeg, pitchDeg, yawDeg, duration_s)
% MAV_ROLL_PITCH_YAW_GUIDEDNOGPS
% Commands fixed roll/pitch/yaw in GUIDED_NOGPS using SET_ATTITUDE_TARGET,
% then disarms (force-disarm if needed), and RETURNS a checked status struct.
%
% Inputs:
%   rollDeg, pitchDeg, yawDeg  - desired angles [deg]
%   duration_s                 - hold duration [s]
%
% Output (struct "status"):
%   .connected, .autopilotFound
%   .armCmdSent, .armedObserved
%   .attitudeCmdSent, .attitudeObserved, .attitudeErrorDeg (struct)
%   .disarmCmdSent, .disarmedObserved
%   .forceDisarmUsed
%   .success
%   .errorMessage

    %% ---- Defaults / knobs you can change inside the function ----
    thrustCmd     = 0.10;   % thrust command (0..1)
    thrustLimit   = 0.35;   % clamp (script-side safety)
    thrustCmd     = min(thrustCmd, thrustLimit);

    cmdRate_Hz    = 20;     % command rate
    LocalPort     = 14555;  % Mission Planner UDPCI must match

    % How close must the measured attitude be to call it "ok"?
    tolDeg_roll  = 5;
    tolDeg_pitch = 5;
    tolDeg_yaw   = 10; % yaw can be weak/loose depending on setup, so wider tolerance

    %% ---- Status struct (everything initialized "false") ----
    status = struct();
    status.connected        = false;
    status.autopilotFound   = false;

    status.armCmdSent       = false;
    status.armedObserved    = false;

    status.attitudeCmdSent  = false;
    status.attitudeObserved = false;
    status.attitudeErrorDeg = struct('roll', NaN, 'pitch', NaN, 'yaw', NaN);

    status.disarmCmdSent    = false;
    status.disarmedObserved = false;
    status.forceDisarmUsed  = false;

    status.success          = false;
    status.errorMessage     = "";

    %% ---- MAVLink Setup ----
    dialect = mavlinkdialect("common.xml", 2);
    mav = mavlinkio(dialect, ...
        SystemID=255, ...
        ComponentID=1, ...
        ComponentType="MAV_TYPE_GCS", ...
        AutopilotType="MAV_AUTOPILOT_INVALID");

    cleanupObj = onCleanup(@() try_disconnect(mav)); %#ok<NASGU>

    try
        %% ---- Connect ----
        connect(mav, "UDP", "LocalPort", LocalPort);
        status.connected = true;
        fprintf("Listening on UDP 0.0.0.0:%d...\n", LocalPort);

        %% ---- Wait autopilot ----
        apClient = waitForAutopilotClient(mav, 7.0);
        if isempty(apClient)
            error("No autopilot heartbeat. Check Mission Planner UDPCI settings.");
        end
        status.autopilotFound = true;
        fprintf("Autopilot found: SysID %d, CompID %d\n", apClient.SystemID, apClient.ComponentID);

        % Subscribers for checking states
        hbSub  = mavlinksub(mav, "HEARTBEAT");
        attSub = mavlinksub(mav, "ATTITUDE");  % roll/pitch/yaw feedback (rad)

        %% ---- ARM ----
        disp("Sending ARM command...");
        status.armCmdSent = true;
        mavcmd(mav, dialect, apClient, 400, 1);  % MAV_CMD_COMPONENT_ARM_DISARM, arm
        pause(0.7);

        status.armedObserved = waitArmedState(hbSub, true, 5.0);
        if ~status.armedObserved
            warning("ARM command sent but ARMED state not observed in HEARTBEAT.");
        end

        %% ---- Set mode GUIDED_NOGPS ----
        disp("Setting mode to GUIDED_NOGPS...");
        mavcmd(mav, dialect, apClient, 176, 1, 20); % MAV_CMD_DO_SET_MODE, custom_mode=20
        pause(0.7);

        %% ---- Build SET_ATTITUDE_TARGET ----
        disp("Starting attitude commands (roll/pitch/yaw)...");
        assignin('base','ATT_CMD_STARTED_T', tic);   % publish start time
        disp("ATT_CMD_STARTED_T set");              % optional debug print

        rollRad  = deg2rad(rollDeg);
        pitchRad = deg2rad(pitchDeg);
        yawRad   = deg2rad(yawDeg);

        q = eulerToQuaternion(rollRad, pitchRad, yawRad);

        attMsg = createmsg(dialect, "SET_ATTITUDE_TARGET");
        attMsg.Payload.target_system    = uint8(apClient.SystemID);
        attMsg.Payload.target_component = uint8(apClient.ComponentID);

        % Ignore body rates, USE attitude+thrust
        attMsg.Payload.type_mask        = uint8(7);

        attMsg.Payload.q               = single(q);
        attMsg.Payload.body_roll_rate  = single(0);
        attMsg.Payload.body_pitch_rate = single(0);
        attMsg.Payload.body_yaw_rate   = single(0);
        attMsg.Payload.thrust          = single(thrustCmd);

        if isfield(attMsg.Payload, "thrust_body")
            attMsg.Payload.thrust_body = single([0 0 0]);
        end

        %% ---- Command loop ----
        status.attitudeCmdSent = true;
        dt     = 1/cmdRate_Hz;
        tStart = tic;
        k      = 0;

        lastMeasDeg = struct('roll', NaN, 'pitch', NaN, 'yaw', NaN);

        while toc(tStart) < duration_s
            k = k + 1;
            attMsg.Payload.time_boot_ms = uint32(toc(tStart)*1000);
            sendmsg(mav, attMsg, apClient);

            % grab latest attitude feedback (if available)
            msgs = latestmsgs(attSub, 1);
            if ~isempty(msgs)
                m = msgs(end);
                % ATTITUDE message fields: roll, pitch, yaw (rad) in most MAVLink dialects
                if isfield(m.Payload, "roll") && isfield(m.Payload, "pitch") && isfield(m.Payload, "yaw")
                    lastMeasDeg.roll  = rad2deg(double(m.Payload.roll));
                    lastMeasDeg.pitch = rad2deg(double(m.Payload.pitch));
                    lastMeasDeg.yaw   = rad2deg(double(m.Payload.yaw));
                end
            end

            if mod(k, round(cmdRate_Hz)) == 0
                fprintf("t=%.1fs: cmd R=%.1f P=%.1f Y=%.1f deg | thrust=%.2f\n", ...
                    toc(tStart), rollDeg, pitchDeg, yawDeg, thrustCmd);
            end

            pause(dt);
        end

        disp("Finished sending attitude commands.");

        %% ---- Evaluate attitude tracking (checked status) ----
        % Use wrap-around safe yaw error
        status.attitudeErrorDeg.roll  = angleDiffDeg(lastMeasDeg.roll,  rollDeg);
        status.attitudeErrorDeg.pitch = angleDiffDeg(lastMeasDeg.pitch, pitchDeg);
        status.attitudeErrorDeg.yaw   = angleDiffDeg(lastMeasDeg.yaw,   yawDeg);

        status.attitudeObserved = ...
            isfinite(status.attitudeErrorDeg.roll)  && ...
            isfinite(status.attitudeErrorDeg.pitch) && ...
            isfinite(status.attitudeErrorDeg.yaw)   && ...
            abs(status.attitudeErrorDeg.roll)  <= tolDeg_roll  && ...
            abs(status.attitudeErrorDeg.pitch) <= tolDeg_pitch && ...
            abs(status.attitudeErrorDeg.yaw)   <= tolDeg_yaw;

        fprintf("Measured attitude (deg): R=%.1f P=%.1f Y=%.1f | Errors: dR=%.1f dP=%.1f dY=%.1f\n", ...
            lastMeasDeg.roll, lastMeasDeg.pitch, lastMeasDeg.yaw, ...
            status.attitudeErrorDeg.roll, status.attitudeErrorDeg.pitch, status.attitudeErrorDeg.yaw);

        if ~status.attitudeObserved
            warning("Attitude not within tolerance (this may be fine on a bench / yaw may be loose).");
        end

        %% ---- Safety tail: thrust=0 briefly ----
        disp("Sending thrust=0 tail (0.5s)...");
        attMsg.Payload.thrust = single(0);
        tTail = tic;
        while toc(tTail) < 0.5
            attMsg.Payload.time_boot_ms = uint32((duration_s + toc(tTail))*1000);
            sendmsg(mav, attMsg, apClient);
            pause(dt);
        end

        %% ---- DISARM (normal then force) ----
        disp("Sending DISARM command...");
        status.disarmCmdSent = true;
        mavcmd(mav, dialect, apClient, 400, 0); % disarm
        pause(0.5);

        status.disarmedObserved = waitArmedState(hbSub, false, 3.0);

        if ~status.disarmedObserved
            warning("Disarm not observed. Trying FORCE DISARM (param2=21196)...");
            status.forceDisarmUsed = true;
            mavcmd(mav, dialect, apClient, 400, 0, 21196);
            pause(0.5);
            status.disarmedObserved = waitArmedState(hbSub, false, 3.0);
        end

        if status.disarmedObserved
            disp("✅ Disarmed successfully.");
        else
            warning("Still ARMED after disarm attempts.");
        end

        %% ---- Overall success ----
        % You asked: return checked status after every command; we do that.
        % For "success", require: autopilot found + armed observed + disarmed observed.
        status.success = status.autopilotFound && status.armedObserved && status.disarmedObserved;

    catch ME
        status.errorMessage = string(ME.message);
        fprintf(2, "❌ Error: %s\n", ME.message);
    end
end

% =======================================================
%                 Local helper functions
% =======================================================

function ok = waitArmedState(hbSub, wantArmed, timeoutSec)
    t0 = tic;
    ok = false;
    while toc(t0) < timeoutSec
        msgs = latestmsgs(hbSub, 8);
        for m = msgs
            if isfield(m.Payload, "base_mode")
                base_mode = double(m.Payload.base_mode);
                isArmed = bitand(base_mode, 128) ~= 0; % MAV_MODE_FLAG_SAFETY_ARMED = 128
                if isArmed == wantArmed
                    ok = true;
                    return;
                end
            end
        end
        pause(0.05);
    end
end

function apClient = waitForAutopilotClient(mav, waitSec)
    hbSub = mavlinksub(mav, "HEARTBEAT");
    t0 = tic; apClient = [];
    while toc(t0) < waitSec
        msgs = latestmsgs(hbSub, 16);
        if ~isempty(msgs)
            for m = msgs
                if ~(m.SystemID==255 && m.ComponentID==1)
                    apClient = mavlinkclient(mav, m.SystemID, m.ComponentID);
                    return;
                end
            end
        end
        pause(0.05);
    end
end

function try_disconnect(mav)
    try
        disconnect(mav);
        disp("Disconnected MAVLink.");
    catch
    end
end

function q = eulerToQuaternion(roll, pitch, yaw)
    % Convert ZYX euler (roll, pitch, yaw) to quaternion [w x y z] as single
    cy = cos(yaw*0.5);  sy = sin(yaw*0.5);
    cp = cos(pitch*0.5); sp = sin(pitch*0.5);
    cr = cos(roll*0.5);  sr = sin(roll*0.5);

    w = cr*cp*cy + sr*sp*sy;
    x = sr*cp*cy - cr*sp*sy;
    y = cr*sp*cy + sr*cp*sy;
    z = cr*cp*sy - sr*sp*cy;

    q = single([w, x, y, z]);
end

function d = angleDiffDeg(measDeg, cmdDeg)
    % Smallest signed difference meas-cmd in degrees, handling wrap-around.
    if ~isfinite(measDeg) || ~isfinite(cmdDeg)
        d = NaN;
        return;
    end
    d = mod((measDeg - cmdDeg) + 180, 360) - 180;
end
