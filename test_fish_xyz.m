%Click on the position of the fish from the overhead and side views, solve
%for x, y, and z coordinates
clc, clear variables, close all

%% Load camera calibration parameters found using Matlab image processing toolbox
load cameraParams_overhead
load cameraParams_side
%%

% %select overhead image file to get the path
[imname, impath] = uigetfile('.tif', 'Pick overhead image file');

T_o=imread([impath,imname]);
T_o=undistortImage(T_o,cameraParams_overhead);
imagesc(T_o);
axis ij
title('Overhead Image')
I=getframe(gcf);
H=impoint;
pos_o=getPosition(H);
close all

% select side image file to get the path
[imname_side, impath_side] = uigetfile('.tif', 'Pick side image file');

T_cam=imread([impath_side,imname_side]);
T_cam=undistortImage(T_cam,cameraParams_side);
imagesc(T_cam);
title('Side Image')
I=getframe(gcf);
H_side=impoint;
pos_s=getPosition(H_side);
% close all

for dd=1%:length(pos_o)
    [world_outputs(dd,1),world_outputs(dd,2),world_outputs(dd,3),world_outputs(dd,4),world_outputs(dd,5),world_outputs(dd,6)] = Camera_Refraction(pos_o(dd,1),size(T_o,1)-pos_o(dd,2),pos_s(dd,1),size(T_cam,1)-pos_s(dd,2));
end
x_tank=world_outputs(:,4);
y_tank=world_outputs(:,5);
z_tank=world_outputs(:,6);