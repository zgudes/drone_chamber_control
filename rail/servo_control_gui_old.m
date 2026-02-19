function servo_control_gui
% Simple GUI to control a servo driven platform over a COM port
% Run in MATLAB: servo_control_gui

%% ===== PROTOCOL AND PORT SETTINGS =====
COM_PORT_DEFAULT   = 'COM7';       % Default COM port
BAUDRATE           = 19200;         % Serial baud rate
TIMEOUT_S          = 2;            % Serial timeout in seconds

CMD_SPEED_PREFIX   = 'Speed_';      % Speed command prefix, example: Speed_35
CMD_FORWARD        = 'Start_Forvard'; % Forward command, exactly as in device firmware
CMD_REVERSE        = 'Start_Reverse'; % Reverse or return command
CMD_STOP           = 'Stop';        % Emergency stop command, change if your controller uses another word

KICK_COUNT         = 5;             % Repeated forward triggers to overcome a possible stall
NEWLINE            = sprintf('\n'); % Line ending sent with every command

SPEED_MIN = 20;                     % Allowed speed range required by your controller
SPEED_MAX = 50;

%% ===== APPLICATION STATE =====
state.s = [];                       % Serial object handle
state.connected = false;            % Connection flag

%% ===== USER INTERFACE =====
fig = figure('Name','Servo Controller','NumberTitle','off',...
    'MenuBar','none','ToolBar','none','Position',[200 200 520 300],...
    'CloseRequestFcn',@onClose);

uicontrol(fig,'Style','text','String','Port','HorizontalAlignment','left',...
    'Position',[20 255 80 20]);

portList = availablePorts();
hPort = uicontrol(fig,'Style','popupmenu','String',portList, ...
    'Position',[20 230 160 25], 'Value', pickDefaultIndex(portList, COM_PORT_DEFAULT));

uicontrol(fig,'Style','pushbutton','String','Refresh ports',...
    'Position',[190 230 120 25],'Callback',@onRefreshPorts);

hConnect = uicontrol(fig,'Style','togglebutton','String','Connect',...
    'Position',[320 230 80 25],'Callback',@onToggleConnect);

hDisconnect = uicontrol(fig,'Style','pushbutton','String','Disconnect',...
    'Position',[410 230 90 25],'Callback',@onDisconnect,'Enable','off');

uicontrol(fig,'Style','text','String','Speed','HorizontalAlignment','left',...
    'Position',[20 190 80 18]);

hSpeed = uicontrol(fig,'Style','slider','Min',SPEED_MIN,'Max',SPEED_MAX,'Value',SPEED_MIN,...
    'SliderStep',[1/(SPEED_MAX-SPEED_MIN) 5/(SPEED_MAX-SPEED_MIN)],...
    'Position',[20 170 250 20],'Callback',@onSpeedChange,'Enable','off');

% FIX 1: removed stray ']' at the end of this line
hSpeedEdit = uicontrol(fig,'Style','edit','String',num2str(SPEED_MIN),...
    'Position',[280 167 60 24],'Callback',@onSpeedEdit,'Enable','off');

uicontrol(fig,'Style','text','String','Direction','HorizontalAlignment','left',...
    'Position',[20 140 80 18]);

hDirGroup = uibuttongroup(fig,'Position',[0.04 0.33 0.42 0.16],...
    'BorderType','none','Visible','on');
hForward = uicontrol(hDirGroup,'Style','radiobutton','String','Forward',...
    'Position',[10 10 120 22],'Value',1,'Enable','off');
hReverse = uicontrol(hDirGroup,'Style','radiobutton','String','Return',...
    'Position',[130 10 120 22],'Value',0,'Enable','off');

hStart = uicontrol(fig,'Style','pushbutton','String','Start',...
    'Position',[20 100 120 30],'Callback',@onStart,'Enable','off');

hStop = uicontrol(fig,'Style','pushbutton','String','STOP NOW',...
    'Position',[150 100 120 30],'Callback',@onStop,'ForegroundColor',[1 0 0],...
    'FontWeight','bold','Enable','off');

uicontrol(fig,'Style','text','String','Kick count','HorizontalAlignment','left',...
    'Position',[290 108 80 18]);
hKick = uicontrol(fig,'Style','edit','String',num2str(KICK_COUNT),...
    'Position',[370 105 60 24],'Enable','off');

hLog = uicontrol(fig,'Style','listbox','Position',[20 20 480 70],'Max',2,'Min',0);
logMsg('App ready');

%% ===== CALLBACKS =====
    function onRefreshPorts(~,~)
        % Refresh the list of available COM ports
        plist = availablePorts();
        set(hPort,'String',plist,'Value',pickDefaultIndex(plist, COM_PORT_DEFAULT));
        logMsg('Ports refreshed');
    end

    function onToggleConnect(src,~)
        % Connect or disconnect when the toggle button changes state
        if get(src,'Value') == 1
            doConnect();
        else
            onDisconnect();
        end
    end

    function doConnect()
        % Open the serial port and enable controls
        if state.connected, return; end
        ports = get(hPort,'String');
        idx   = get(hPort,'Value');
        if isempty(ports)
            logMsg('No ports found');
            set(hConnect,'Value',0);
            return
        end
        portName = ports{idx};
        try
            state.s = opencom(portName, BAUDRATE, TIMEOUT_S);
            state.connected = true;
            set([hDisconnect hSpeed hSpeedEdit hForward hReverse hStart hStop hKick],'Enable','on');
            set(hConnect,'String','Connected','Enable','off');
            logMsg(sprintf('Connected to %s', portName));
        catch ME
            logMsg(['Connect error: ' ME.message]);
            set(hConnect,'Value',0);
        end
    end

    function onDisconnect(~,~)
        % Close the serial port and disable controls
        if state.connected
            try
                closecom(state.s);
            catch ME
                logMsg(['Disconnect error: ' ME.message]);
            end
        end
        state.s = [];
        state.connected = false;
        set([hDisconnect hSpeed hSpeedEdit hForward hReverse hStart hStop hKick],'Enable','off');
        set(hConnect,'String','Connect','Enable','on','Value',0);
        logMsg('Disconnected');
    end

    function onSpeedChange(~,~)
        % Slider moved, clamp to integer range and update the edit box
        v = round(get(hSpeed,'Value'));
        v = max(SPEED_MIN, min(SPEED_MAX, v));
        set(hSpeed,'Value',v);
        set(hSpeedEdit,'String',num2str(v));
    end

    function onSpeedEdit(~,~)
        % Numeric edit changed, parse and update the slider
        v = str2double(get(hSpeedEdit,'String'));
        if isnan(v), v = SPEED_MIN; end
        v = round(max(SPEED_MIN, min(SPEED_MAX, v)));
        set(hSpeed,'Value',v);
        set(hSpeedEdit,'String',num2str(v));
    end

    function onStart(~,~)
        % Send speed first, then a motion command based on the chosen direction
        if ~state.connected
            logMsg('Not connected');
            return
        end
        % Temporarily disable buttons while sending commands
        set([hStart hStop hDisconnect],'Enable','off');
        c = onCleanup(@() set([hStart hStop hDisconnect],'Enable','on')); %#ok<NASGU>
        try
            spd = round(get(hSpeed,'Value'));

            % Build and send Speed_<v>\n as pure char
            spdLine = sprintf('%s%d%s', CMD_SPEED_PREFIX, spd, NEWLINE);
            sendLine(state.s, spdLine);
            pause(0.05);

            % Start motion in the selected direction
            if get(hForward,'Value') == 1
                logMsg(sprintf('Start forward at %d', spd));
                k = str2double(get(hKick,'String'));
                if isnan(k) || k < 1, k = 1; end
                k = min(max(round(k),1),20);
                for i = 1:k
                    sendLine(state.s, [CMD_FORWARD NEWLINE]);
                    pause(0.1);
                end
            else
                logMsg(sprintf('Start return at %d', spd));
                sendLine(state.s, [CMD_REVERSE NEWLINE]);
            end
        catch ME
            logMsg(['Start error: ' ME.message]);
        end
    end

    function onStop(~,~)
        % Emergency stop, send several stop commands in quick succession
        if ~state.connected
            logMsg('Not connected');
            return
        end
        try
            for i = 1:3
                sendLine(state.s, [CMD_STOP NEWLINE]);
                pause(0.03);
            end
            logMsg('Stop sent');
        catch ME
            logMsg(['Stop error: ' ME.message]);
        end
    end

    function onClose(~,~)
        % Try to stop the motor and close the port when the window is closed
        try
            if state.connected && ~isempty(state.s)
                try
                    for i = 1:2
                        sendLine(state.s, [CMD_STOP NEWLINE]);
                        pause(0.02);
                    end
                catch
                end
                closecom(state.s);
            end
        catch
        end
        delete(fig);
    end

    function logMsg(t)
        % Append a time stamped line to the log list box
        ts = datestr(now,'HH:MM:SS');
        cur = get(hLog,'String');
        if ischar(cur), cur = {cur}; end
        set(hLog,'String',[cur; {sprintf('[%s] %s',ts,t)}]);
        drawnow;
    end
end

%% ===== SERIAL HELPERS =====
function s = opencom(portName, baudrate, timeout_s)
% Open a serial port and return the handle
% Uses the classic serial interface for broad MATLAB version support
    if nargin < 1 || isempty(portName), portName = 'COM18'; end
    if nargin < 2 || isempty(baudrate), baudrate = 19200; end
    if nargin < 3 || isempty(timeout_s), timeout_s = 30; end

    s = serial(portName);              %#ok<SERIAL> classic serial object
    set(s,'BaudRate',baudrate);
    set(s,'Timeout',timeout_s);        % Property name is Timeout
    fopen(s);
end

function err = closecom(s)
% Safely close and delete a serial object
    err = 0;
    try
        if ~isempty(s) && strcmp(get(s,'Status'),'open')
            fclose(s);
        end
    catch
        err = 1;
    end
    try
        delete(s);
    catch
        err = err + 1;
    end
end

function sendLine(s, strline)
% Send a single line, newline must be included by the caller
    fprintf(s, '%s', strline);
end

function list = availablePorts()
% Return a cell array of available COM port names
    try
        list = serialportlist("available");
        if isempty(list)
            list = serialportlist; %#ok<SERIALPORTLIST>
        end
        if isrow(list), list = list(:); end
        list = cellstr(list);
    catch
        % Fallback scan, attempt to open common COM numbers
        list = {};
        for k = 1:32
            try
                s = serial(['COM' num2str(k)]); %#ok<SERIAL>
                fopen(s);
                fclose(s);
                delete(s);
                list{end+1} = ['COM' num2str(k)]; %#ok<AGROW>
            catch
            end
        end
        if isempty(list)
            list = { 'COM18' };
        end
    end
end

function idx = pickDefaultIndex(list, def)
% Select the default item index in a popup menu
    idx = 1;
    for k = 1:numel(list)
        if strcmpi(list{k}, def)
            idx = k;
            return
        end
    end
end

%% ===== OPTIONAL ONE SHOT WRAPPERS =====
function motor_forward(speed)
% One shot forward start with speed set in advance
    s = opencom('COM18',19200,30);
    fprintf(s, 'Speed_%d\n', round(speed));
    pause(0.05);
    for i = 1:5
        fprintf(s, 'Start_Forvard\n');
        pause(0.1);
    end
    closecom(s);
end

function motor_return()
% One shot reverse
    s = opencom('COM18',19200,30);
    fprintf(s, 'Start_Reverse\n');
    closecom(s);
end

function motor_stop()
% One shot emergency stop
    s = opencom('COM18',19200,30);
    fprintf(s, 'Stop\n'); % Change if your device uses a different word
    closecom(s);
end
