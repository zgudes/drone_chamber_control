function ctx = mav_att_stream_start(rollDeg, pitchDeg, yawDeg, thrustCmd, cmdRate_Hz)
% Starts continuous SET_ATTITUDE_TARGET streaming using a timer (same MATLAB session).
% Returns ctx which you must pass to mav_att_stream_stop(ctx).

    if nargin < 4 || isempty(thrustCmd),  thrustCmd = 0.10; end
    if nargin < 5 || isempty(cmdRate_Hz), cmdRate_Hz = 20;  end

    % Clamp thrust for safety
    thrustCmd = min(thrustCmd, 0.35);

    ctx = struct();
    ctx.cmdRate_Hz = cmdRate_Hz;
    ctx.dt = 1/cmdRate_Hz;

    % ---- MAVLink setup ----
    ctx.dialect = mavlinkdialect("common.xml", 2);
    ctx.mav = mavlinkio(ctx.dialect, ...
        SystemID=255, ...
        ComponentID=1, ...
        ComponentType="MAV_TYPE_GCS", ...
        AutopilotType="MAV_AUTOPILOT_INVALID");

    ctx.cleanupMav = onCleanup(@() try_disconnect(ctx.mav)); %#ok<NASGU>

    LocalPort = 14555;
    connect(ctx.mav,"UDP","LocalPort",LocalPort);
    fprintf("Listening on UDP 0.0.0.0:%d...\n", LocalPort);

    ctx.apClient = waitForAutopilotClient(ctx.mav, 7.0);
    if isempty(ctx.apClient)
        error("No autopilot heartbeat. Check Mission Planner UDPCI settings.");
    end
    fprintf("Autopilot found: SysID %d, CompID %d\n", ctx.apClient.SystemID, ctx.apClient.ComponentID);

    ctx.hbSub = mavlinksub(ctx.mav, "HEARTBEAT");

    % ---- ARM ----
    disp("Sending ARM command...");
    mavcmd(ctx.mav, ctx.dialect, ctx.apClient, 400, 1);
    pause(0.7);

    % ---- Set mode GUIDED_NOGPS ----
    disp("Setting mode to GUIDED_NOGPS...");
    mavcmd(ctx.mav, ctx.dialect, ctx.apClient, 176, 1, 20);
    pause(0.7);

    % ---- Build attitude command ----
    disp("Starting attitude commands (roll/pitch/yaw)...");

    rollRad  = deg2rad(rollDeg);
    pitchRad = deg2rad(pitchDeg);
    yawRad   = deg2rad(yawDeg);
    q = eulerToQuaternion(rollRad, pitchRad, yawRad);

    ctx.attMsg = createmsg(ctx.dialect, "SET_ATTITUDE_TARGET");
    ctx.attMsg.Payload.target_system    = uint8(ctx.apClient.SystemID);
    ctx.attMsg.Payload.target_component = uint8(ctx.apClient.ComponentID);

    % Ignore body rates (bits 0,1,2 = 1), use attitude+thrust
    ctx.attMsg.Payload.type_mask        = uint8(7);
    ctx.attMsg.Payload.q               = single(q);
    ctx.attMsg.Payload.body_roll_rate  = single(0);
    ctx.attMsg.Payload.body_pitch_rate = single(0);
    ctx.attMsg.Payload.body_yaw_rate   = single(0);
    ctx.attMsg.Payload.thrust          = single(thrustCmd);

    if isfield(ctx.attMsg.Payload, "thrust_body")
        ctx.attMsg.Payload.thrust_body = single([0 0 0]);
    end

    ctx.tStart = tic;
    ctx.k = 0;

    % ---- Timer that streams commands ----
    % IMPORTANT: TimerFcn is a nested function that can update ctx via guidata-like behavior
    t = timer( ...
        'ExecutionMode','fixedSpacing', ...
        'Period', ctx.dt, ...
        'BusyMode','drop', ...
        'TasksToExecute', Inf);

    % store ctx in timer UserData so TimerFcn can modify it safely
    t.UserData = ctx;
    t.TimerFcn = @att_tick;

    ctx.timer = t;

    start(ctx.timer);
end

function att_tick(t, ~)
    ctx = t.UserData;

    try
        ctx.k = ctx.k + 1;
        ctx.attMsg.Payload.time_boot_ms = uint32(toc(ctx.tStart)*1000);
        sendmsg(ctx.mav, ctx.attMsg, ctx.apClient);

        if mod(ctx.k, round(ctx.cmdRate_Hz)) == 0
            fprintf("t=%.1fs: streaming attitude | thrust=%.2f\n", ...
                toc(ctx.tStart), double(ctx.attMsg.Payload.thrust));
        end
    catch ME
        fprintf(2, "att_tick error: %s\n", ME.message);
    end

    % write back updated ctx
    t.UserData = ctx;
end

% ---- helpers ----
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
    cy = cos(yaw*0.5);   sy = sin(yaw*0.5);
    cp = cos(pitch*0.5); sp = sin(pitch*0.5);
    cr = cos(roll*0.5);  sr = sin(roll*0.5);

    w = cr*cp*cy + sr*sp*sy;
    x = sr*cp*cy - cr*sp*sy;
    y = cr*sp*cy + sr*cp*sy;
    z = cr*cp*sy - sr*sp*cy;

    q = single([w, x, y, z]);
end
