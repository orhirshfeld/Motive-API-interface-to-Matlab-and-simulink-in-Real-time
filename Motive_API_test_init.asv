%% Init simulink
%% loading files (project and code libary)

% TT_Tools_demo(project_file)
%
% This function demonstrates basic functionality of the Tracking Tools API
% from Natural Point (to be used with Optitrack). Before use it is
% essential to have calibrated cameras and created the desirable
% trackable/s and save these in tta project file. The project file is then
% passed to the function and the library loaded if need be before
% attempting to plot any data
%
% Input - project_file - a string containing path and filename for the
%               project file that is to be used (default value assigned if
%               not assigned)
%
% Written by Glen Lichtwark, University of Queensland, Australia
% Last updated: 22nd Jan 2010
% Please acknowledge in any academic papers which may utilise this code

clc; clear all; close all; %clear all

project_file = 'D:\or hirshfeld\onedrive\work control lab aero summer 2014 technion\MOTIVE_api\motive15.ttp'; % change if necessary
%addpath('C:\Users\sorhirsh\OneDrive\work control lab aero summer 2014 technion\'); % path where you can find reltime block for simulink

% load the NPTrackingTools library if it is not already loaded
if ~libisloaded('NPTrackingTools')
    
    addpath('C:\Program Files\OptiTrack\Motive\lib'); % change if necessary
    addpath('C:\Program Files\OptiTrack\Motive\inc'); % change if necessary

   [notfound,warnings]=loadlibrary('NPTrackingToolsx64','NPTrackingTools.h','alias','NPTrackingTools')
end

% to see all function write the code: libfunctionsview NPTrackingTools

%% initilzing

% libfunctionsview NPTrackingTools --> use this to see available functions

% initialise cameras
calllib('NPTrackingTools', 'TT_Initialize') %you should see red leds on the camera turns on
%%
% load the project file which sets up cameras correctly
calllib('NPTrackingTools', 'TT_LoadProject', project_file)

%% test
X = 0;Y = 0;Z = 0;
qx = 0;qy = 0;qz = 0;qw = 0;
yaw = 0;pitch = 0;roll = 0;
TrackableNum=0

calllib('NPTrackingTools', 'TT_UpdateSingleFrame')
T = calllib('NPTrackingTools', 'TT_FrameTimeStamp')
[X,Y,Z,qx,qy,qz,qw,yaw,pitch,roll] = calllib('NPTrackingTools', 'TT_TrackableLocation',TrackableNum,X,Y,Z,qx,qy,qz,qw,yaw,pitch,roll);

%% run simlink
sim('car_model.slx'); % call simulink model

%% shutdown
%calllib('NPTrackingTools', 'TT_FinalCleanup')
calllib('NPTrackingTools', 'TT_Shutdown') % exit tracking tools
unloadlibrary NPTrackingTools

