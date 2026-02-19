function mavcmd(mav, dialect, apClient, commandID, varargin)
% MAVCMD  Sends a general MAVLink COMMAND_LONG message.
%
% Usage:
%   mavcmd(mav, dialect, apClient, commandID, p1, p2, p3, p4, p5, p6, p7)
%
%   Any missing parameters (p1-p7) will be set to 0.

% --- 1. Parse Parameters (p1-p7) ---
% Set all 7 params to 0 by default
params = zeros(1, 7, 'single');

% Overwrite with user-provided values
n_params = min(7, numel(varargin)); % Don't allow more than 7
for i = 1:n_params
    if ~isempty(varargin{i}) % Allow empty [] to mean 0
        params(i) = single(varargin{i});
    end
end

% --- 2. Create COMMAND_LONG message ---
msg = createmsg(dialect, "COMMAND_LONG");

% --- 3. Set Target ---
msg.Payload.target_system    = apClient.SystemID;
msg.Payload.target_component = apClient.ComponentID;

% --- 4. Set Command ID ---
msg.Payload.command = uint16(commandID);

% --- 5. Set Command Parameters ---
msg.Payload.param1 = params(1);
msg.Payload.param2 = params(2);
msg.Payload.param3 = params(3);
msg.Payload.param4 = params(4);
msg.Payload.param5 = params(5);
msg.Payload.param6 = params(6);
msg.Payload.param7 = params(7);

% --- 6. Send Message ---
sendmsg(mav, msg, apClient);

% Print a confirmation to the console
fprintf('Sent Command ID %d with params: [%g, %g, %g, %g, %g, %g, %g]\n', ...
    commandID, params(1), params(2), params(3), params(4), ...
    params(5), params(6), params(7));
end