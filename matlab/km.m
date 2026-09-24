%% ========================================================================
% build_simulink_model.m
%
% Creates and simulates:
%
%   1. laser_cnc_openloop.slx
%      Actual 3-axis CNC laser marking machine - OPEN LOOP
%
%   2. laser_cnc_camera_pid.slx
%      Proposed camera-feedback CLOSED LOOP system
%
% X and Y are motorized.
% Z-axis is manually adjusted for laser focusing.
%
% Requires:
%   - MATLAB
%   - Simulink
%   - Control System Toolbox
%% ========================================================================

clear;
clc;
close all;

%% ========================================================================
% SYSTEM PARAMETERS
%% ========================================================================

% ---------------- Pulley ----------------
% Assumed pulley diameter = 20 mm
pulleyDiameter = 20e-3;       % m
r = pulleyDiameter/2;          % m

% ---------------- Mechanical parameters ----------------
Jm  = 5.4e-6;                 % Motor rotor inertia [kg.m^2]
Jp  = 2.0e-6;                 % Pulley inertia [kg.m^2]
M   = 0.70;                   % Carriage mass [kg]
b   = 1.0e-3;                 % Viscous damping [N.m.s/rad]

% Equivalent rotational inertia
Jeq = Jm + Jp + M*r^2;

% ---------------- Motor electrical parameters ----------------
R_w = 1.6;                    % Winding resistance [Ohm]
L_w = 3.8e-3;                 % Winding inductance [H]

% ---------------- Motor constants ----------------
Kt = 0.28;                    % Torque constant [N.m/A]
Kb = 0.28;                    % Back EMF constant [V.s/rad]

% ---------------- Position reference ----------------
ref = 0.050;                  % 50 mm [m]

% ---------------- PID parameters ----------------
Kp = 9.0;
Ki = 6.0;
Kd = 0.16;

% ---------------- Laser ----------------
laserPWM = 0.6;               % PWM duty cycle
laserMaxPower = 5;            % Maximum optical power [W]

% ---------------- Manual Z focus ----------------
Zfocus = 8.0;                 % mm

%% ========================================================================
% DISPLAY PARAMETERS
%% ========================================================================

fprintf('\n');
fprintf('====================================================\n');
fprintf('       CNC LASER MARKING MACHINE SIMULATION\n');
fprintf('====================================================\n');
fprintf('Pulley radius       : %.2f mm\n',r*1000);
fprintf('Equivalent inertia  : %.6e kg.m^2\n',Jeq);
fprintf('Carriage mass       : %.2f kg\n',M);
fprintf('Reference position  : %.2f mm\n',ref*1000);
fprintf('PID Kp              : %.2f\n',Kp);
fprintf('PID Ki              : %.2f\n',Ki);
fprintf('PID Kd              : %.2f\n',Kd);
fprintf('Laser PWM           : %.2f\n',laserPWM);
fprintf('Manual Z focus      : %.2f mm\n',Zfocus);
fprintf('====================================================\n');
fprintf('\n');

%% ========================================================================
% MODEL 1
% ACTUAL CNC MACHINE - OPEN LOOP
%% ========================================================================

m1 = 'laser_cnc_openloop';

% Close model if already open
if bdIsLoaded(m1)
    close_system(m1,0);
end

% Delete existing file if present
if exist([m1 '.slx'],'file')
    delete([m1 '.slx']);
end

% Create new model
new_system(m1);
open_system(m1);

%% ------------------------------------------------------------------------
% PC / G-CODE POSITION COMMAND
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Sources/Step', ...
    [m1 '/PC G-code Position Command'], ...
    'Time','0.05', ...
    'Before','0', ...
    'After',num2str(ref), ...
    'Position',[20 100 90 140]);

%% ------------------------------------------------------------------------
% X AXIS
%% ------------------------------------------------------------------------

xsub = [m1 '/X Axis - A4988 NEMA17 Belt'];

add_block( ...
    'built-in/Subsystem', ...
    xsub, ...
    'Position',[160 60 360 130]);

build_axis(xsub,R_w,L_w,Kt,Kb,Jeq,b,r);

%% ------------------------------------------------------------------------
% Y AXIS
%% ------------------------------------------------------------------------

ysub = [m1 '/Y Axis - A4988 NEMA17 Belt'];

add_block( ...
    'built-in/Subsystem', ...
    ysub, ...
    'Position',[160 180 360 250]);

build_axis(ysub,R_w,L_w,Kt,Kb,Jeq,b,r);

%% ------------------------------------------------------------------------
% XY POSITION MUX
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Signal Routing/Mux', ...
    [m1 '/XY Position'], ...
    'Inputs','2', ...
    'Position',[410 95 440 215]);

%% ------------------------------------------------------------------------
% XY POSITION SCOPE
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Sinks/Scope', ...
    [m1 '/XY Position Scope'], ...
    'Position',[490 105 530 145]);

%% ------------------------------------------------------------------------
% XY POSITION TO WORKSPACE
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Sinks/To Workspace', ...
    [m1 '/xy_open'], ...
    'VariableName','xy_open', ...
    'SaveFormat','Structure With Time', ...
    'Position',[490 175 590 210]);

%% ------------------------------------------------------------------------
% LASER PWM
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Sources/Constant', ...
    [m1 '/Laser PWM Duty'], ...
    'Value',num2str(laserPWM), ...
    'Position',[160 300 240 330]);

%% ------------------------------------------------------------------------
% LASER DRIVER
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Math Operations/Gain', ...
    [m1 '/Laser Driver'], ...
    'Gain',num2str(laserMaxPower), ...
    'Position',[290 295 390 335]);

%% ------------------------------------------------------------------------
% LASER OPTICAL POWER
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Sinks/Scope', ...
    [m1 '/Laser Optical Power W'], ...
    'Position',[440 295 480 335]);

%% ------------------------------------------------------------------------
% MANUAL Z AXIS
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Sources/Constant', ...
    [m1 '/Manual Z Focus mm'], ...
    'Value',num2str(Zfocus), ...
    'Position',[160 390 280 420]);

add_block( ...
    'simulink/Sinks/Terminator', ...
    [m1 '/Z Terminator'], ...
    'Position',[330 398 360 412]);

%% ========================================================================
% MODEL 1 CONNECTIONS
%% ========================================================================

% PC command -> X axis
add_line( ...
    m1, ...
    'PC G-code Position Command/1', ...
    'X Axis - A4988 NEMA17 Belt/1', ...
    'autorouting','on');

% PC command -> Y axis
add_line( ...
    m1, ...
    'PC G-code Position Command/1', ...
    'Y Axis - A4988 NEMA17 Belt/1', ...
    'autorouting','on');

% X -> Mux
add_line( ...
    m1, ...
    'X Axis - A4988 NEMA17 Belt/1', ...
    'XY Position/1', ...
    'autorouting','on');

% Y -> Mux
add_line( ...
    m1, ...
    'Y Axis - A4988 NEMA17 Belt/1', ...
    'XY Position/2', ...
    'autorouting','on');

% Mux -> Scope
add_line( ...
    m1, ...
    'XY Position/1', ...
    'XY Position Scope/1', ...
    'autorouting','on');

% Mux -> Workspace
add_line( ...
    m1, ...
    'XY Position/1', ...
    'xy_open/1', ...
    'autorouting','on');

% PWM -> laser driver
add_line( ...
    m1, ...
    'Laser PWM Duty/1', ...
    'Laser Driver/1', ...
    'autorouting','on');

% Laser driver -> optical power
add_line( ...
    m1, ...
    'Laser Driver/1', ...
    'Laser Optical Power W/1', ...
    'autorouting','on');

% Z -> terminator
add_line( ...
    m1, ...
    'Manual Z Focus mm/1', ...
    'Z Terminator/1', ...
    'autorouting','on');

%% ------------------------------------------------------------------------
% OPEN LOOP ANNOTATION
%% ------------------------------------------------------------------------

set_param(m1,'ModelBrowserVisibility','off');

%% ------------------------------------------------------------------------
% OPEN LOOP SIMULATION SETTINGS
%% ------------------------------------------------------------------------

set_param( ...
    m1, ...
    'StopTime','0.5', ...
    'Solver','ode23t');

%% ------------------------------------------------------------------------
% SAVE OPEN LOOP MODEL
%% ------------------------------------------------------------------------

save_system(m1,[m1 '.slx']);

fprintf('Open-loop model created successfully.\n');

%% ========================================================================
% MODEL 2
% PROPOSED CLOSED LOOP CAMERA + PID
%% ========================================================================

m2 = 'laser_cnc_camera_pid';

% Close model if already open
if bdIsLoaded(m2)
    close_system(m2,0);
end

% Delete old file
if exist([m2 '.slx'],'file')
    delete([m2 '.slx']);
end

% Create model
new_system(m2);
open_system(m2);

%% ------------------------------------------------------------------------
% TARGET ARTWORK / REFERENCE
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Sources/Step', ...
    [m2 '/Target Artwork Reference'], ...
    'Time','0.05', ...
    'Before','0', ...
    'After',num2str(ref), ...
    'Position',[20 100 90 140]);

%% ------------------------------------------------------------------------
% SUMMING JUNCTION
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Math Operations/Sum', ...
    [m2 '/Position Error'], ...
    'Inputs','+-', ...
    'Position',[120 105 150 135]);

%% ------------------------------------------------------------------------
% PID CONTROLLER
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Continuous/PID Controller', ...
    [m2 '/PC Brain - Vision PID'], ...
    'P',num2str(Kp), ...
    'I',num2str(Ki), ...
    'D',num2str(Kd), ...
    'Position',[180 90 300 150]);

%% ------------------------------------------------------------------------
% X AXIS PLANT
%% ------------------------------------------------------------------------

axsub = [m2 '/GRBL A4988 NEMA17 Belt X Axis'];

add_block( ...
    'built-in/Subsystem', ...
    axsub, ...
    'Position',[330 90 500 155]);

build_axis(axsub,R_w,L_w,Kt,Kb,Jeq,b,r);

%% ------------------------------------------------------------------------
% POSITION SCOPE
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Sinks/Scope', ...
    [m2 '/Laser Head Position'], ...
    'Position',[540 95 580 135]);

%% ------------------------------------------------------------------------
% POSITION TO WORKSPACE
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Sinks/To Workspace', ...
    [m2 '/x_closed'], ...
    'VariableName','x_closed', ...
    'SaveFormat','Structure With Time', ...
    'Position',[540 165 640 200]);

%% ------------------------------------------------------------------------
% CAMERA FRAME DELAY
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Continuous/Transport Delay', ...
    [m2 '/Overhead Camera Frame Delay'], ...
    'DelayTime','0.02', ...
    'Position',[390 245 510 285]);

%% ------------------------------------------------------------------------
% CAMERA CALIBRATION
%% ------------------------------------------------------------------------

add_block( ...
    'simulink/Math Operations/Gain', ...
    [m2 '/Camera Calibration'], ...
    'Gain','1', ...
    'Position',[300 245 360 285]);

%% ========================================================================
% MODEL 2 CONNECTIONS
%% ========================================================================

% Reference -> summing junction
add_line( ...
    m2, ...
    'Target Artwork Reference/1', ...
    'Position Error/1', ...
    'autorouting','on');

% Error -> PID
add_line( ...
    m2, ...
    'Position Error/1', ...
    'PC Brain - Vision PID/1', ...
    'autorouting','on');

% PID -> CNC axis
add_line( ...
    m2, ...
    'PC Brain - Vision PID/1', ...
    'GRBL A4988 NEMA17 Belt X Axis/1', ...
    'autorouting','on');

% CNC axis -> scope
add_line( ...
    m2, ...
    'GRBL A4988 NEMA17 Belt X Axis/1', ...
    'Laser Head Position/1', ...
    'autorouting','on');

% CNC axis -> workspace
add_line( ...
    m2, ...
    'GRBL A4988 NEMA17 Belt X Axis/1', ...
    'x_closed/1', ...
    'autorouting','on');

% CNC position -> camera
add_line( ...
    m2, ...
    'GRBL A4988 NEMA17 Belt X Axis/1', ...
    'Overhead Camera Frame Delay/1', ...
    'autorouting','on');

% Camera -> calibration
add_line( ...
    m2, ...
    'Overhead Camera Frame Delay/1', ...
    'Camera Calibration/1', ...
    'autorouting','on');

% Camera feedback -> negative input
add_line( ...
    m2, ...
    'Camera Calibration/1', ...
    'Position Error/2', ...
    'autorouting','on');

%% ------------------------------------------------------------------------
% CLOSED LOOP SIMULATION SETTINGS
%% ------------------------------------------------------------------------

set_param( ...
    m2, ...
    'StopTime','0.6', ...
    'Solver','ode23t');

%% ------------------------------------------------------------------------
% SAVE CLOSED LOOP MODEL
%% ------------------------------------------------------------------------

save_system(m2,[m2 '.slx']);

fprintf('Closed-loop model created successfully.\n');

%% ========================================================================
% RUN SIMULATIONS
%% ========================================================================

fprintf('\nRunning open-loop simulation...\n');

sim(m1);

fprintf('Open-loop simulation completed.\n');

fprintf('\nRunning closed-loop simulation...\n');

sim(m2);

fprintf('Closed-loop simulation completed.\n');

%% ========================================================================
% PLOT CLOSED LOOP RESPONSE
%% ========================================================================

if exist('x_closed','var')

    figure('Color','w');

    plot( ...
        x_closed.time*1000, ...
        x_closed.signals.values*1000, ...
        'LineWidth',2);

    hold on;

    yline( ...
        ref*1000, ...
        '--', ...
        'Reference');

    grid on;

    xlabel('Time (ms)');
    ylabel('Position (mm)');

    title( ...
        'Camera + PID Closed-Loop CNC Position Response');

    legend( ...
        'Closed-loop position', ...
        'Reference', ...
        'Location','southeast');

end

%% ========================================================================
% FINAL MESSAGE
%% ========================================================================

fprintf('\n');
fprintf('====================================================\n');
fprintf('             SIMULATION COMPLETE\n');
fprintf('====================================================\n');
fprintf('Created files:\n');
fprintf('  %s.slx\n',m1);
fprintf('  %s.slx\n',m2);
fprintf('====================================================\n');


%% ========================================================================
% LOCAL FUNCTION
%
% Creates one motorized CNC axis:
%
% Position command
%       |
%       v
%   A4988 driver
%       |
%       v
%   NEMA17 motor
%       |
%       v
%   Angular velocity
%       |
%       v
%   Angular position
%       |
%       v
%   Pulley/Belt
%       |
%       v
%   Linear carriage position
%% ========================================================================

function build_axis(path,R_w,L_w,Kt,Kb,Jeq,b,r)

    %% --------------------------------------------------------------------
    % A4988 DRIVER
    %% --------------------------------------------------------------------

    add_block( ...
        'simulink/Math Operations/Gain', ...
        [path '/A4988 Driver'], ...
        'Gain','1', ...
        'Position',[20 40 70 80]);

    %% --------------------------------------------------------------------
    % STEPPER MOTOR ELECTROMECHANICAL MODEL
    %
    % omega(s)/V(s) =
    %
    %             Kt
    % --------------------------------
    % (Ls+R)(Jeq*s+b) + Kt*Kb
    %
    % Expanded denominator:
    %
    % L*Jeq*s^2
    % +(L*b + R*Jeq)*s
    % +(R*b + Kt*Kb)
    %% --------------------------------------------------------------------

    num = sprintf('[%.12g]',Kt);

    den = sprintf( ...
        '[%.12g %.12g %.12g]', ...
        L_w*Jeq, ...
        L_w*b + R_w*Jeq, ...
        R_w*b + Kt*Kb);

    add_block( ...
        'simulink/Continuous/Transfer Fcn', ...
        [path '/NEMA17 V to Omega'], ...
        'Numerator',num, ...
        'Denominator',den, ...
        'Position',[100 30 230 90]);

    %% --------------------------------------------------------------------
    % ANGULAR VELOCITY -> ANGULAR POSITION
    %% --------------------------------------------------------------------

    add_block( ...
        'simulink/Continuous/Integrator', ...
        [path '/Omega to Theta'], ...
        'Position',[260 45 290 75]);

    %% --------------------------------------------------------------------
    % PULLEY CONVERSION
    %
    % x = r * theta
    %% --------------------------------------------------------------------

    add_block( ...
        'simulink/Math Operations/Gain', ...
        [path '/Pulley Radius Theta to X'], ...
        'Gain',sprintf('%.12g',r), ...
        'Position',[320 45 380 75]);

    %% --------------------------------------------------------------------
    % INPUT PORT
    %% --------------------------------------------------------------------

    add_block( ...
        'simulink/Ports & Subsystems/In1', ...
        [path '/Position Command'], ...
        'Position',[-20 50 0 70]);

    %% --------------------------------------------------------------------
    % OUTPUT PORT
    %% --------------------------------------------------------------------

    add_block( ...
        'simulink/Ports & Subsystems/Out1', ...
        [path '/Carriage Position'], ...
        'Position',[420 50 440 70]);

    %% ====================================================================
    % INTERNAL CONNECTIONS
    %% ====================================================================

    add_line( ...
        path, ...
        'Position Command/1', ...
        'A4988 Driver/1', ...
        'autorouting','on');

    add_line( ...
        path, ...
        'A4988 Driver/1', ...
        'NEMA17 V to Omega/1', ...
        'autorouting','on');

    add_line( ...
        path, ...
        'NEMA17 V to Omega/1', ...
        'Omega to Theta/1', ...
        'autorouting','on');

    add_line( ...
        path, ...
        'Omega to Theta/1', ...
        'Pulley Radius Theta to X/1', ...
        'autorouting','on');

    add_line( ...
        path, ...
        'Pulley Radius Theta to X/1', ...
        'Carriage Position/1', ...
        'autorouting','on');

end