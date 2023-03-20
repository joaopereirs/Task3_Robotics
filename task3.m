%% INITIALISE PC/LIBRARY PARAMETERS

clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% CONTROL TABLE ADDRESSES (same for all 4x XM430-W350-T Servos)

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table addresses are different for each Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;
ADDR_MIN_POS                 = 52;
ADDR_MAX_POS                 = 48;
ADDR_MAX_VEL                 = 44;
ADDR_DRIVE_MODE              = 10;
ADDR_PROF_ACC                = 108;
ADDR_PROF_VEL                = 112;


%% SERVO-COMMON SETTINGS

PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel
BAUDRATE                    = 115200;
DEVICENAME                  = 'COM6';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0;            % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4095;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% SERVO-SPECIFIC IDENTIFICATION (RO5 in room 303)

DXL_ID_1 = 11;            % Dynamixel ID: 1
DXL_ID_2 = 12;
DXL_ID_3 = 13;
DXL_ID_4 = 14;
DXL_ID_5 = 15;

%% PORT STRUCTURE INITIALISATION

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result

%% VARIABLE INITIALISATION 1 

dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position
dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position

%% PORT OPENING
 
% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end

%% SERVO BAUD RATE

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end


%% SET MOTION LIMITS
%SERVO-COMMON MOTION LIMITS %Between 0 and 4096%

MIN_POS_1 = 569;
MIN_POS_2 = 1024;
MIN_POS_3 = 800;
MIN_POS_4 = 900;
MIN_POS_5 = 0;

MAX_POS_1 = 3527;
MAX_POS_2 = 2731;
MAX_POS_3 = 2958;
MAX_POS_4 = 3413;
MAX_POS_5 = 4095;

% Set max position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_MAX_POS, MAX_POS_1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_MAX_POS, MAX_POS_2);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_MAX_POS, MAX_POS_3);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_MAX_POS, MAX_POS_4);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_MAX_POS, MAX_POS_5);

% Set min position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_MIN_POS, MIN_POS_1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_MIN_POS, MIN_POS_2);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_MIN_POS, MIN_POS_3);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_MIN_POS, MIN_POS_4);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_MIN_POS, MIN_POS_5);

%Set drive modes
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_DRIVE_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_DRIVE_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_DRIVE_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_DRIVE_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_DRIVE_MODE, 4);

%Set profile accelerations
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PROF_ACC, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PROF_ACC, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PROF_ACC, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PROF_ACC, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PROF_ACC, 1000);

%Set profile velocities
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PROF_VEL, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PROF_VEL, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PROF_VEL, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PROF_VEL, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PROF_VEL, 1000);
%% 
%read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PROF_ACC);
drivemode1 = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_DRIVE_MODE);
drivemode1

%% ENABLE SERVO-SPECIFIC TORQUE

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

%% HUB COMMS VERIFICATION

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

%%  

%MOTION 0
move(0,0.1,0.1,90,1000,1000);
% %OPEN GRIPPER
open();

%MOTION 1

%VERTICAL!!!
move(0.075*((0.075*2.5)+0.475),0,0.02,0,1000,1000);
move(0.075*((0.075*2.5)+0.475),0,-0.04,0,1000,1000);
close();
move(0.075*((0.075*2.5)+0.475),0,0.02,0,1000,1000);


%MOTION 2
move(0,0.18,0.1,45,1000,1000);

%VERTICAL!!!
move(0,0.1*((0.1*2.5)+0.475),0.02,0,1000,1000);
% %MOTION 3
move(0,0.1*((0.1*2.5)+0.475),-0.04,0,1000,500);

%(0.075*0.28)-0.06

%%
%OPEN GRIPPER
open();
% %MOTION 4
%%
%CLOSE GRIPPER

move(0,0.1*((0.1*2.5)+0.475),-0.04,0,1000,500);

close();

move(0,0.1*((0.1*2.5)+0.475),0.02,0,1000,1000);

%% 
%
move(0,0.205*((0.205*2.5)+0.475),0,0,1000,1000);
open();
move(0,0.1,0.1,90,1000,1000);

%% flip
move(0,0.205*((0.205*2.5)+0.475)-0.01,-0.015,70,1000,1000);
close();
move(0,0.205*((0.205*2.5)+0.475)-0.01,0.1,70,1000,1000);
 
move(0,0.1,0.1,90,1000,1000); %fin neutral pos
move(0,0.205*((0.205*2.5)+0.475)-0.029,-0.02,-20,1000,1000);

open();

%% flip inverse

move(0,0.205*((0.205*2.5)+0.475)-0.029,-0.02,-20,1000,1000);
close();
move(0,0.205*((0.205*2.5)+0.475)-0.029,0.02,-20,1000,1000);

move(0,0.1,0.1,90,1000,1000); %fin neutral pos

move(0,0.205*((0.205*2.5)+0.475)-0.01,-0.015,70,1000,1000);

open();

%%
move(0,0.1,0.1,90,1000,1000);
close();

%% clap

open();
close_more();


%% z testing

move(0,0.225,0,65,1000,1000);
close(); 

open();
move(0,0.2,0,65,1000,1000); 
close();

open();
move(0,0.175,0,65,1000,1000);
close();

open();
move(0,0.15,0,65,1000,1000);
close();

open();
move(0,0.125,0,65,1000,1000);
close();

open();
move(0,0.1,0,65,1000,1000);
close();

open();
move(0,0.075,0,65,1000,1000);
close();
%% angle testing

open();
move(0,0.21*((0.205*2.5)+0.475),0,10,1000,1000);
close();
open();
move(0,0.21*((0.205*2.5)+0.475),0,50,1000,1000);
close();
open();
move(0,0.21*((0.205*2.5)+0.475),0,70,1000,1000);
close();
open();
move(0,0.21*((0.205*2.5)+0.475),0,-20,1000,1000);

%% task 2a


%MOTION 0
move(0,0.1,0.1,95,1000,1000);
% %OPEN GRIPPER
open();


%First block
move(0,0.225,0.01,5,1000,1000);
move(0,0.225,-0.01,5,1000,1000);
close();
move(0,0.225,0.01,5,1000,1000);
move(0,0.1,0.01,5,1000,1000);
move(0,0.1,0,5,1000,1000);
open();
move(0,0.1,0.01,5,1000,1000);



%second block
move(0.2,0.075,0.01,5,1000,1000);
move(0.2,0.075,0,5,1000,1000);
close();
move(0.2,0.075,0.01,5,1000,1000);
move(0.125,0.125,0.01,5,1000,1000);
move(0.125,0.125,0,5,1000,1000);
open();
move(0.125,0.125,0.01,5,1000,1000);


%third block
move(-0.15,0.15,0.01,5,1000,1000);
move(-0.15,0.15,0,5,1000,1000);
close();
move(-0.15,0.15,0.01,10,1000,1000);


move(-0.1,0,0.01,5,1000,1000);
move(-0.1,0,0,5,1000,1000);
open();
move(-0.1,0,0.01,5,1000,1000);
move(0,0.1,0.1,5,1000,1000);



%% task 2b 
 
open();
move(0,0.202,0,5,1000,1000);
close();
open();
% close();
% move(0,0.1,0.1,90,1000,1000);
% move(0,0.1,0,-25,1000,1000);
% open();
% move(0,0.1,0.1,90,1000,1000);

%% Task 2b (edited for different robot)
%MOTION 0 
move(0,0.1,0.1,90,1000,1000);
% %OPEN GRIPPER
open();

% block 1
for i = 1:2
    move(0,0.22,-0.035,65,1000,1000);
    close();
    move(0,0.225,0.01,65,1000,1000);
 
    move(0,0.1,0.1,90,1000,1000); % neutral position
  
    move(-0.001,0.1,0.005,-25,1000,1000);
   
    move(-0.001,0.1,-0.04,-25,1000,1000);
    open();
    move(-0.001,0.1,0.01,-25,1000,1000);
    move(0,0.1,0.1,90,1000,1000);
    move(0,0.1,0.02,5,1000,1000);
    move(0,0.09,0,5,1000,1000);%might be an error with y
    close();
    move(0,0.1,0.02,5,1000,1000);
    move(0,0.225,0.01,5,1000,1000);
    move(-0.005,0.216,-0.01,5,1000,1000);
    open();
    move(-0.001,0.212,0.01,5,1000,1000);
    move(0,0.1,0.1,90,1000,1000); % neutral position
    
end

 
%block 2
open();
move(0,0.1,0.1,90,1000,1000);
move(0.2,0.075,0.02,5,1000,1000);
move(0.2,0.075,0,5,1000,10000);
close();
move(0.2,0.075,0.02,5,1000,1000);
move(0,0.1,0.02,5,1000,1000);
move(0,0.095,0,5,1000,1000);
open();
move(0,0.1,0.02,5,1000,1000);
move(0,0.1,-0.02,-25,1000,1000);
close();
move(0,0.1,0.02,5,1000,1000);
move(0,0.1,0.1,90,1000,1000);

move(0.176,0.064,0.02,65,1000,1000);
move(0.176,0.064,-0.04,65,1000,10000);
open();



%block 3
open();
move(0,0.1,0.1,90,1000,1000);
move(-0.15,0.15,0.03,5,1000,1000);
move(-0.15,0.15,0,5,1000,1000);
close();
move(-0.15,0.15,0.03,5,1000,1000);
move(0.001,0.095,0,5,1000,1000);
open();
move(0,0.1,0.02,5,1000,1000);
move(0,0.1,-0.02,-25,1000,1000);
close();
move(0,0.1,0.02,5,1000,1000);
move(0,0.1,0.1,90,1000,1000);
move(-0.133,0.128,0.02,65,1000,1000);
move(-0.133,0.128,-0.04,65,1000,1000);
open();
move(-0.15,0.15,0.02,65,1000,1000);
move(0,0.1,0.1,90,1000,1000);



%% task 2c (edited for different robot)
open();
move(0,0.1,0.1,90,1000,1000);
% %OPEN GRIPPER
open();
% Flipping
move(0,0.225,0.02,5,1000,1000);
move(0,0.2225,0,5,1000,1000);
close();
move(0,0.225,0.02,5,1000,1000);
move(0,0.1,0.02,5,1000,1000);
move(0,0.1,0,5,1000,1000);
open();
move(0,0.1,0.02,5,1000,1000);
move(0,0.1,0,-25,1000,1000);
close();
move(0,0.1,0.02,5,1000,1000);
move(0,0.1,0.1,90,1000,1000);

move(0,0.225,0.02,65,1000,1000);
move(0,0.225,0,65,1000,10000);
open();

%%
% move block 1
move(-0.001,0.216,0.01,5,1000,1000);
move(-0.001,0.216,-0.01,5,1000,1000);
close();
move(-0.001,0.216,0.02,5,1000,1000);
move(0,0.1,0.02,5,1000,1000);
move(0,0.086,-0.04,5,1000,1000);
open();
move(0,0.1,0.02,5,1000,1000)

%Move block 2

move(0,0.1,0.1,90,1000,1000);
move(0.2,0.075,0.02,5,1000,1000);
move(0.2,0.075,0,5,1000,10000);
close();
move(0.2,0.075,0.02,5,1000,1000);
move(0,0.093,0.02,5,1000,1000);
move(0,0.093,0,5,1000,1000);
open();
move(0,0.1,0.02,5,1000,1000);


%move block 3

move(0,0.1,0.1,90,1000,1000);
move(-0.15,0.15,0.03,5,1000,1000);
move(-0.15,0.15,0,5,1000,1000);
close();
move(-0.15,0.15,0.03,5,1000,1000);
move(0.001,0.093,0.012,5,1000,1000);
open();
move(0,0.1,0.02,5,1000,1000);

move(0,0.1,0.1,90,1000,1000);

%% Pen tests
move(0,0.1,0.1,90,1000,1000);
open();
close_more();

%%

move(0,0.1,0.1,90,1000,1000);
move(-0.15, 0.15, -0.02, 88, 1000, 1000)
move(-0.15, 0.15, -0.032, 88, 5000, 1000)
move(-0.12, 0.12, -0.034, 82, 5000, 5000)
move(-0.1, 0.12, -0.034, 78, 1000, 1000)
move(0,0.1,0.1,90,1000,1000);


%% pen tests with gripper

move(0,0.1,0.1,90,1000,1000);
open();
move(0.0375,0.1875,0.002,90,1000,1000);
close_more();

move(0.0375,0.1875,0.02,90,1000,1000);
move(0,0.1,0.1,90,1000,1000);
%%
move(-0.06, 0.2, -0.004, 90, 1000, 1000)
pause(1);
move(-0.09, 0.205, -0.004, 90, 1000, 1000)
move(-0.11, 0.204, -0.004, 90, 1000, 1000)
move(-0.13, 0.203, -0.004, 90, 1000, 1000)
move(-0.14, 0.204, -0.004, 90, 1000, 1000)
pause(1);
move(-0.14, 0.205, -0.004, 90, 1000, 1000)
move(-0.138, 0.19, -0.004, 90, 1000, 1000)
move(-0.134, 0.15, -0.006, 90, 1000, 1000)
move(-0.135, 0.125, -0.006, 90, 1000, 1000)
pause(1);
move(-0.126, 0.136, -0.008, 90, 1000, 1000)
move(-0.116, 0.146, -0.008, 90, 1000, 1000)
move(-0.104, 0.156, -0.007, 90, 1000, 1000)%
move(-0.094, 0.166, -0.006, 90, 1000, 1000)%
move(-0.084, 0.176, -0.006, 90, 1000, 1000)
move(-0.074, 0.186, -0.005, 90, 1000, 1000)
move(-0.065, 0.195, -0.004, 90, 1000, 1000)
move(-0.06, 0.2, -0.004, 90, 1000, 1000)
pause(1);
move(0,0.1,0.1,90,1000,1000);

%% 

move(0,0.1,0.1,90,1000,1000);
LineInterpolation(-0.06, 0.2, -0.14, 0.2, -0.002, 90, 1000, 1000, 0.2);
LineInterpolation(-0.14, 0.2, -0.14, 0.125, -0.002, 90, 1000, 1000, 0.2);
LineInterpolation(-0.14, 0.125, -0.06, 0.2, -0.002, 90, 1000, 1000, 0.2);
move(0,0.1,0.1,90,1000,1000);

%%
move(0,0.1,0.1,90,1000,1000);
pause(1);
CurveInterpolation(-0.06, 0.17, -0.14, 0.17, 0.005, 90, 0.04, 1000, 1000, 0.001)
pause(1);
move(0,0.1,0.1,90,1000,1000);

%% task 3
move(0,0.1,0.1,90,1000,1000);
open();

%%
%pick up pen
move(0,0.1,0.1,90,1000,1000);
open();
move(0.0675, 0.1575, 0.002, 90, 2000, 2000);
move(0.0675, 0.1875, 0.002, 90, 2000, 2000);
close_more();
move(0.0675, 0.1875, 0.02, 90, 2000, 2000);
move(0,0.1,0.1,95,1000,2000);

%%
% Draw shape
move(0,0.1,0.1,95,1000,1000);


move_drawing(-0.085, 0.225, 0.015, 95, 1000, 1000, 1);
pause(1);
LineInterpolation(-0.085, 0.225, -0.165, 0.2225, 0, 95, 1000, 1000, 0.1);
LineInterpolation(-0.165, 0.2225, -0.165, 0.15, 0, 95, 1000, 1000, 0.1);
LineInterpolation(-0.165, 0.15, -0.088, 0.225, 0, 95, 1000, 1000, 0.1);
CurveInterpolation(-0.085, 0.225, -0.085, 0.15, 0, 95, 0.04, 1000, 1000, 0.0002)
pause(1);
move(0,0.1,0.1,90,2000,2000);

%%
move(0,0.1,0.1,90,1000,1000);
move(-0.1,0.1,0.01,95,1000,1000)

 
%% DISABLE TORQUE FOR ALL SERVOS
torqueoff_all();
%% HUB COMMS RE-VERIFICATION

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end



%% FORWARD KINEMATICS

function output2 = ForwardKin(theta1,theta2,theta3,theta4)
    n=1;
    figure;
    hold on;
    for j = 1:n
%         theta1 = theta1_array(j); 
%         theta2 = theta2_array(j); 
%         theta3 = theta3_array(j); 
%         theta4 = theta4_array(j); 
        alpha = [0, pi/2, 0, 0, 0]; 
        a = [0, 0, 0.13, 0.124, 0.11]; %0.126 for original gripper, 0.11 for our gripper
        d = [0, 0, 0, 0, 0]; 
        theta = [theta1, theta2, theta3, theta4, 0]; 
        T = eye(4);
        T_list = zeros(4, 4, length(alpha));
    
        % Create transformation matrices for each joint
        for i = 1:length(alpha)
            Ti = [cos(theta(i)), -sin(theta(i)),  0, a(i);
                sin(theta(i))*cos(alpha(i)),  cos(theta(i))*cos(alpha(i)), -sin(alpha(i)), -sin(alpha(i))*d(i);
                sin(theta(i))*sin(alpha(i)),  cos(theta(i))*sin(alpha(i)), cos(alpha(i)), cos(alpha(i))*d(i);
                0,             0,                          0,                         1];
            T = T * Ti;
            T_list(:, :, i) = T;
        end
    
        %Figure plotting
        plot3(0,0,0);
        quiver3(0, 0, 0, 1, 0, 0, 'color','r');
        quiver3(0, 0, 0, 0, 1, 0, 'color','b');
        quiver3(0, 0, 0, 0, 0, 1, 'color','g');
        pos_old = [0,0,0];
        for i = 1:length(alpha)
            T_current = T_list(:, :, i);
            quiver3(T_current(1,4), T_current(2,4), T_current(3,4), T_current(1,1), T_current(2,1), T_current(3,1), 'color', 'r'); %x-axis
            quiver3(T_current(1,4), T_current(2,4), T_current(3,4), T_current(1,2), T_current(2,2), T_current(3,2), 'color', 'b'); %y-axis
            quiver3(T_current(1,4), T_current(2,4), T_current(3,4), T_current(1,3), T_current(2,3), T_current(3,3), 'color', 'g'); %z-axis
            line([pos_old(1) T_current(1,4)],[pos_old(2) T_current(2,4)],[pos_old(3) T_current(3,4)], 'color', 'k')
            pos_old = [T_current(1,4), T_current(2,4), T_current(3,4)];
        end
        q = findobj(gca, 'Type', 'Quiver');
        set(q, 'AutoScaleFactor', 0.05);
        l = findobj(gca, 'Type', 'Line');
        axis equal;
        grid on;
        view(3);
        axis([-0.3 0.5 -0.3 0.5 -0.3 0.5])
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        title('3D frames according to a DH table');
        pause(1);
        if j < n
            delete(q);
            delete(l);
        end

    end

    output2 = [T(1,4), T(2,4), T(3,4)];
end

%% MOVE FUNCTION
function move(X,Y,Z,ANGLE,DURATION,END_DURATION)

%X_error = -0.02*X + 0.019;

    if Z<0.0001 && Z > -0.0001
    Z = Z_offset(sqrt(X^2+Y^2), ANGLE);
    end
%     if ANGLE >62 && ANGLE < 67
%         Z = Z - 0.035;
%     end
%     if ANGLE < -22 && ANGLE > -27
%         Z = Z + 0.014;
%     end
thetatry_gen = InverseKin(X,Y,Z,deg2rad(ANGLE));
deg_thetatry_gen = rad2deg(thetatry_gen);
deg_servo_gen = zeros(4,1);
deg_servo_gen(1) = (deg_thetatry_gen(1)*+1) + 90  ;
deg_servo_gen(2) = (deg_thetatry_gen(2)*-1) + 270 ;
deg_servo_gen(3) = (deg_thetatry_gen(3)*-1) + 90  ;
deg_servo_gen(4) = (deg_thetatry_gen(4)*-1) + 180 ;

act_servo_gen = (deg_servo_gen/360) * 4096;


write4ByteTxRx(0, 2, 11, 116, act_servo_gen(1));
write4ByteTxRx(0, 2, 12, 116, act_servo_gen(2));
write4ByteTxRx(0, 2, 13, 116, act_servo_gen(3));
write4ByteTxRx(0, 2, 14, 116, act_servo_gen(4));

write4ByteTxRx(0, 2, 11, 112, DURATION);
write4ByteTxRx(0, 2, 12, 112, DURATION);
write4ByteTxRx(0, 2, 13, 112, DURATION);
write4ByteTxRx(0, 2, 14, 112, END_DURATION);


pause(DURATION/1000); %keep it at 1000 for block tasks

end

%% SHANICE SHUT IT PLEASE
function close()
write4ByteTxRx(0, 2, 15, 116, 2300);
pause(1)
end

%% Tighter close
function close_more()
write4ByteTxRx(0, 2, 15, 116, 2600);
pause(1)
end

%% OPEN
function open()
write4ByteTxRx(0, 2, 15, 116, 1200);
pause(1)
end

%% Straight line interpolation

function LineInterpolation(x_initial, y_initial, x_final, y_final, z, angle, duration, end_duration, pause_time)
xinterval = (x_final - x_initial)/10;
yinterval = (y_final - y_initial)/10;
time = 0.2;
for i = 0:1:10

    move_drawing(x_initial + xinterval*i, y_initial+yinterval*i, z, angle, duration, end_duration, pause_time);
    
    if i == 0
        pause(1);
    end
end
pause(1);
end

%% Curve interpolation

function CurveInterpolation(x_initial, y_initial, x_final, y_final, z, angle, radius, duration, end_duration, pause_time)
    xc = (x_initial + x_final)/2; % x-axis of circle center
    yc = ((y_initial + y_final)/2)-radius; %y-axis of circle center

    % Calculate the angle of the semicircle
    theta = linspace(0,pi,15);

    y_temp = radius*cos(theta) + xc;
    x_temp = -radius*sin(theta) + yc; %% change radius between negativre and positive to switch side
    x_temp = x_temp + (x_initial - x_temp(1));
    y_temp = y_temp + (y_initial - y_temp(1));

    for i = 1:1:length(x_temp)
        move_drawing(x_temp(i),y_temp(i),z,angle,duration,end_duration, duration/3800);
        if i == 1
            pause(4);
        end
    end

end
%% move function for drawing task

function move_drawing(X,Y,Z,ANGLE,DURATION,END_DURATION, pause_time)

%X_error = -0.02*X + 0.019;

    if Z<0.0001 && Z > -0.0001
    Z = Z_offset(sqrt(X^2+Y^2), ANGLE);
    end
%     if ANGLE >62 && ANGLE < 67
%         Z = Z - 0.035;
%     end
%     if ANGLE < -22 && ANGLE > -27
%         Z = Z + 0.014;
%     end
thetatry_gen = InverseKin(X,Y,Z,deg2rad(ANGLE));
deg_thetatry_gen = rad2deg(thetatry_gen);
deg_servo_gen = zeros(4,1);
deg_servo_gen(1) = (deg_thetatry_gen(1)*+1) + 90  ;
deg_servo_gen(2) = (deg_thetatry_gen(2)*-1) + 270 ;
deg_servo_gen(3) = (deg_thetatry_gen(3)*-1) + 90  ;
deg_servo_gen(4) = (deg_thetatry_gen(4)*-1) + 180 ;

act_servo_gen = (deg_servo_gen/360) * 4096;


write4ByteTxRx(0, 2, 11, 116, act_servo_gen(1));
write4ByteTxRx(0, 2, 12, 116, act_servo_gen(2));
write4ByteTxRx(0, 2, 13, 116, act_servo_gen(3));
write4ByteTxRx(0, 2, 14, 116, act_servo_gen(4));

write4ByteTxRx(0, 2, 11, 112, DURATION);
write4ByteTxRx(0, 2, 12, 112, DURATION);
write4ByteTxRx(0, 2, 13, 112, DURATION);
write4ByteTxRx(0, 2, 14, 112, END_DURATION);


pause(pause_time); %keep it at 1000 for block tasks

end

%% INVERSE KINEMATICS
function output = InverseKin(x,y,z,b)
    a3 = 0.13;
    a4 = 0.124;
    %a5 = 0.126; 
    a5 = 0.11;

    x_y = sqrt(x^2+y^2);
    x_y = x_y - OffsetRejection(x_y,rad2deg(b));
    
    b = b - deg2rad(5);
    %Finding the new coordinate frames
    if -pi/2-deg2rad(1) < b && b <= 0 - deg2rad(1)
        z_new = a5*abs(cos(b))+z;
        x_y_new = x_y + a5*abs(sin(b));
   

    elseif b<= pi/2 + deg2rad(1) && b > 0 - deg2rad(1)
        z_new = a5*cos(b)+z;
        x_y_new = x_y - a5*sin(b);
    

    elseif pi/2 + deg2rad(1) < b && b <= pi
        z_new = z - a5*cos(b);
        x_y_new = x_y - a5*sin(b);
    end

    b = b + deg2rad(5);
    
    %Finding theta 1
    theta1 = atan2(y,x);

    
    %Finding theta 3
    cos3 = (x_y_new^2+z_new^2-a3^2-a4^2)/(2*a3*a4);
    sin3 = -sqrt(1-cos3^2);
    theta3 = atan2(sin3, cos3);
    
    
    %Finding theta 2
    k1 = a3 + a4*cos3;
    k2 = a4*sin3;
    x_temp = x_y_new*k1+z_new*k2;
    y_temp = z_new*k1-x_y_new*k2;
    theta2 = atan2(y_temp, x_temp);
    
    

    %Finding theta 4
    theta4 = (b - theta2 - theta3-pi/2);

    theta = [theta1, theta2, theta3, theta4];
    output = theta;
end

%% RADIAL OFFSET FUNCTION
function offset = OffsetRejection(yx_before, b)
    if b > 4 && b < 6
        if yx_before > 0.05 && yx_before < 0.1125
            offset = 0.025;
        
        elseif yx_before >= 0.1125 && yx_before < 0.1375
            offset = 0.03;
         
        elseif yx_before >= 0.1375 && yx_before < 0.1625
            offset = 0.03;
        elseif yx_before >= 0.1625
            offset = 0.03;
        end
    elseif b > 94 && b < 96
           offset = 0.02; %uncomment this for drawing task
    
    else
        if yx_before > 0.05 && yx_before < 0.1875
            offset = 0.017;
        
        elseif yx_before >= 0.1875 && yx_before < 0.2125
            offset = 0.014;
         
        elseif yx_before >= 0.2125
            offset = 0.023;
        end
    end
end

%% Z OFFSET FUNCTION

function z_pickup = Z_offset(yx, b)
    if b > 4 && b < 6
        if yx >= 0 &&  yx < 0.06
            z_pickup = -0.045;
        elseif yx >= 0.06 && yx < 0.08
            z_pickup = -0.04; %Z = 0.04 for 0 degrees
        elseif yx >= 0.08 && yx < 0.11
            z_pickup = -0.035;
        elseif yx >= 0.11 && yx < 0.13
            z_pickup = -0.03;
        elseif yx >= 0.13 && yx < 0.16
            z_pickup = -0.02;
        elseif yx >= 0.16 && yx < 0.2175 %%%change made here!!!!
            z_pickup = -0.002;
        elseif yx >= 0.2175 
            z_pickup = 0.01; %0 for 0 degrees
        end
    
    else 
        if yx >= 0 &&  yx < 0.0875
            z_pickup = -0.008;
        elseif yx >= 0.0875 && yx < 0.1125
            z_pickup = 0; %Z = 0.04 for 0 degrees
        elseif yx >= 0.1125 && yx < 0.1375
            z_pickup = 0.001;
        elseif yx >= 0.1375 && yx < 0.1625
            z_pickup = 0.002;
        elseif yx >= 0.1625 && yx < 0.1875
            z_pickup = 0.004;
        elseif yx >= 0.1875 && yx < 0.2175 %%%change made here!!!!
            z_pickup = 0.006;
        elseif yx >= 0.2175 
            z_pickup = 0.006; %0 for 0 degrees
        end
    end
end
%% ALL TORQUE OFF
function torqueoff_all()
write1ByteTxRx(0, 2, 11, 64, 0);
write1ByteTxRx(0, 2, 12, 64, 0);
write1ByteTxRx(0, 2, 13, 64, 0);
write1ByteTxRx(0, 2, 14, 64, 0);
write1ByteTxRx(0, 2, 15, 64, 0);
end