function [x_cam,y_cam,z_cam,x_tank,y_tank,z_tank] = Camera_Refraction(x_io,y_io,y_is,z_is,z_free,d_o,d_s)
% Camera_Refraction finds the three dimensional position of a point based on
% the views from two cameras with refraction taken into account.
% 
% Outputs- [x_cam,y_cam,x_tank,y_tank,z] are the three dimensional coordinates in the real world.
% Units are millimeters. [x_cam,y_cam] have the image center as the origin.
% [x_tank,y_tank] have the tank corner as the origin. [z_cam] is measured from
% mid depth. [z_tank] is depth from the free surface.
%
% Inputs- (x_io,y_io,x_is,z_is) are the coordinates in the image frame in pixels
% from the overhead camera (x_io,y_io) and side camera (x_is,y_is)
%
% Camera parameters needed: 
% f_o, f_s are the focal lnegths of the overhead and side cameras in pixels
% c_xo,c_yo,c_xs,c_zs are the coordinates of the centers of projection of the two camera in pixels
%
% Experimental setup parameters:
% d_o,d_s are the distances from the outer pupil to the interface of the
% two cameras
% z_free is the distance from the side camera axis to the free surface in
% millimeters

%Optical parameters
n=4/3;

%% Load camera calibration parameters found using Matlab image processing toolbox
load cameraParams_overhead
load cameraParams_side
%%

% Camera Parameters from Matlab image processing toolbox
c_xo=cameraParams_overhead.PrincipalPoint(1); %pixels
c_yo=cameraParams_overhead.PrincipalPoint(2); %pixels
f_o=mean(cameraParams_overhead.FocalLength); %pixels

c_ys=cameraParams_side.PrincipalPoint(1); %pixels
c_zs=cameraParams_side.PrincipalPoint(2); %pixels
f_s=mean(cameraParams_side.FocalLength); %pixels

% % Setup Parameters
% d_o=1349.2; %millimeters
% d_s=205.91; %millimeters
% z_free=438.15; %millimeters

tank_zerox=395.4; %pixels
tank_zeroy=-62.65; %pixels

% find the radius from the image center to the pixels
ri_o=sqrt((x_io-c_xo)^2+(y_io-c_yo)^2);
ri_s=sqrt((y_is-c_ys)^2+(z_is-c_zs)^2);
ri_tank=sqrt((tank_zerox-c_xo)^2+(tank_zeroy-c_yo)^2);

% vector angle to the pixels
theta_o=atan2(y_io-c_yo,x_io-c_xo);
theta_s=atan2(z_is-c_zs,y_is-c_ys);
theta_tank=atan2(tank_zeroy-c_yo,tank_zerox-c_xo);

%distance from overhead camera axis to tank corner
rw_tank = (d_o/f_o)*ri_tank;

syms rw_o_sym

%Solve the two equations together
eq1 = (d_o/f_o)*ri_o +...
    (z_free-((d_s/f_s*ri_s)+((1778-abs(rw_tank*cos(theta_tank)))-(rw_o_sym*cos(theta_o)))/sqrt((f_s*n/ri_s)^2 + n^2 -1))*sin(theta_s))...
    /sqrt((f_o*n/ri_o)^2 + n^2 -1)...
    == rw_o_sym;

rw_o = vpasolve(eq1,rw_o_sym);
rw_o = double(rw_o);

%solve for the side camera radius
x_w=(1778-abs(rw_tank*cos(theta_tank)))-(rw_o*cos(theta_o));
rw_s = (d_s/f_s*ri_s)+x_w/sqrt((f_s*n/ri_s)^2 + n^2 -1);

%Camera frame
x_cam=rw_o*cos(theta_o);
y_cam=rw_o*sin(theta_o);
z_cam=rw_s*sin(theta_s);

%Tank Frame
x_tank=rw_o*cos(theta_o)+abs(rw_tank*cos(theta_tank));
y_tank=rw_o*sin(theta_o)+abs(rw_tank*sin(theta_tank));
z_tank=z_free-rw_s*sin(theta_s);


end

