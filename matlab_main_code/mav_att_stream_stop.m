function mav_att_stream_stop(ctx)
% Stops the timer streaming, sends thrust=0 tail, disarms (force if needed), disconnects.

    if isempty(ctx) || ~isfield(ctx,'mav')
        return;
    end

    % Stop timer first
    if isfield(ctx,'timer') && isa(ctx.timer,'timer')
        try
            stop(ctx.timer);
        catch
        end
        try
            delete(ctx.timer);
        catch
        end
    end

    % Thrust=0 tail (0.5s)
    disp("Sending thrust=0 tail (0.5s)...");
    try
        ctx.attMsg.Payload.thrust = single(0);
        dt = 1/max(10, ctx.cmdRate_Hz);
        t0 = tic;
        while toc(t0) < 0.5
            ctx.attMsg.Payload.time_boot_ms = uint32(toc(t0)*1000);
            sendmsg(ctx.mav, ctx.attMsg, ctx.apClient);
            pause(dt);
        end
    catch
    end

    % Disarm
    disp("Sending DISARM command...");
    try
        mavcmd(ctx.mav, ctx.dialect, ctx.apClient, 400, 0);
        pause(0.4);
    catch
    end

    % Force disarm as backup
    disp("Force DISARM (param2=21196) backup...");
    try
        mavcmd(ctx.mav, ctx.dialect, ctx.apClient, 400, 0, 21196);
        pause(0.2);
    catch
    end

    % Disconnect
    try
        disconnect(ctx.mav);
        disp("Disconnected MAVLink.");
    catch
    end
end
