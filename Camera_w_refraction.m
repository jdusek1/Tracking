%Jeff Dusek
%11/21/16

%This code will be used to carry out camera calibration for robot tracking
%in the water lab taking into account the refraction due to the fluid. The
%method will be based on Treibitz et al. (2008). This method assumes that
%the refraction interface is perpendicular to the camera axis, and the
%refraction from the glass window will be neglected as in the paper.
%Variable names will follow the paper as well.

clc, clear variables, close all

%% Load camera calibration parameters found using Matlab image processing toolbox
load cameraParams_overhead
load cameraParams_side
%%

%Basic parameters
n=4/3; %index of refraction of the water

%Need the distance of the image to the interface (depth) in mm
% z_w=889; %overhead with 35 in water
% z_w=[914.4,914.4,914.4,914.4,914.4,914.4,914.4,914.4];
z_w=[0,0,0,0,0,0,0,0]; %side

%line segments to use
lines=8;

%First we will select two points of known separation distance from a camera image.
%These points will be know as pos(n,:)=[(xi,n1),(yi,n1),(xi,n2),(yi,n2)],
%where n1 and 2 are the end points of a line n with known true length
%s_known(n).


% % select first an image file to get the path
[imname, impath] = uigetfile('.tif', 'Pick an image file');

T=imread([impath,imname]);
T=undistortImage(T,cameraParams_side);
imagesc(T);
I=getframe(gcf);

for nn=1:lines
% [imname, impath] = uigetfile('.tif', 'Pick an image file');
% 
% T=imread([impath,imname]);
% imagesc(T);
% I=getframe(gcf);
    for m=1:2:3
    H=impoint;
    pos(nn,m:m+1)=getPosition(H);
    end
 
    prompt = sprintf('What is the true length of the point pair number %d in mm: ',nn);
    s_known(nn) = input(prompt);
%     clf
end
%%
% load pos_refrac 
% load s_known
% 
% load pos_refrac_s 
% load s_known_s


%For this implementation we will do a brute force optimization over the
%free paramters:
% c=[cx,cy] is the camera's center of projection in air [pixels]
% f is the camera's focal length in air [mm]
% d is the distance from the interface to the center of projection of the
% lens [mm]
% cx=linspace(-10,10,20);
% cy=linspace(-10,10,20);
% f=linspace(100,10000,1000);

% cx=974.54; %center of projection at origin overhead from camera cal toolbox
% cy=538.54; %center of projection at origin overhead from camera cal toolbox
% cx=951.04; %side from camera cal toolbox
% cy=526.55; %side from camera cal toolbox
% f=955.30; %overhead from camera cal toolbox
% f=681.67; %side from camera cal toolbox
% 
% cx=981.44; %center of projection at origin overhead from matlab toolbox
% cy=553.04; %center of projection at origin overhead from matlab toolbox
% f=894.49; %overhead from camera matlab toolbox
% % 
cx=958.7337; %side from camera matlab toolbox
cy=555.2902; %side from camera matlab toolbox
f=698.6565; %side from camera matlab toolbox


d=linspace(100,300,1000);
% d=1435.5; %d for overhead camera 
% d=210.99; %side

for dd=1:length(d) %interface to center of projection
    for nn=1:lines %line segments defined by image    
        %radial position in image frame with center of projection
        %varied. n1 and n2 are the two points defining line n. 
        ri_n1(nn)=sqrt((pos(nn,1)-cx)^2+(pos(nn,2)-cy)^2);
        ri_n2(nn)=sqrt((pos(nn,3)-cx)^2+(pos(nn,4)-cy)^2);
                    
        %Angular position of n1 and n2 as center of projection
        %is varied. n1 and n2 are the two points defining line n
        theta1(nn)=atan2(pos(nn,2)-cy,pos(nn,1)-cx);
        theta2(nn)=atan2(pos(nn,4)-cy,pos(nn,3)-cx);
                    
        %positions in world frame. n1 and n2 are the two points defining line n 
        rw_n1(dd,nn)=(d(dd)/f)*ri_n1(nn)+((z_w(nn))/sqrt((f*n/ri_n1(nn))^2 + n^2 -1));
        rw_n2(dd,nn)=(d(dd)/f)*ri_n2(nn)+((z_w(nn))/sqrt((f*n/ri_n2(nn))^2 + n^2 -1));
        
        %length of line n where n1 and n2 are the two points defining line n 
        s(dd,nn)=sqrt(rw_n1(dd,nn)^2+rw_n2(dd,nn)^2-...
            2*rw_n1(dd,nn)*rw_n2(dd,nn)*cos(abs(theta1(nn)-theta2(nn))));
        
        error(dd,nn)=s_known(nn)-s(dd,nn);
    end
    rmse(dd)=sqrt(sum(error(dd,:).^2)/lines);
end
   
figure;
plot(d,rmse)
xlabel('d [mm]')
ylabel('RMSE [mm]')
title('Optimize d Parameter')

[Y,I]=min(rmse)

opt_d=d(I)

