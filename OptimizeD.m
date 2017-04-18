function [d_opt,error]=OptimizeD(lines)
%OptimizeD find the value of d to use in the formulation for refraction at a flat interface
%for a given experimental setup.
%Inputs: lines=number of line segments to use from the image
%Outputs: d_opt= Optimized d value in mm, error= rmse in mm

%Jeff Dusek
%11/21/16

%This code will be used to carry out camera calibration for robot tracking
%in the water lab taking into account the refraction due to the fluid. The
%method will be based on Treibitz et al. (2008). This method assumes that
%the refraction interface is perpendicular to the camera axis, and the
%refraction from the glass window will be neglected as in the paper.
%Variable names will follow the paper as well.

prompt = sprintf('Which camera is being used? For overhead input 1 and for side input 0: ');
cam=input(prompt);

%% Load camera calibration parameters found using Matlab image processing toolbox
load cameraParams_overhead
load cameraParams_side
%%

%Basic parameters
n=4/3; %index of refraction of the water

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
        for m=1:2:3
            H=impoint;
            pos(nn,m:m+1)=getPosition(H);
        end
        prompt2 = sprintf('What is the true length of the point pair number %d in mm: ',nn);
        s_known(nn) = input(prompt2);
        
        prompt3 = sprintf('What is the depth of the point pair number %d in mm: ',nn);
        z_w(nn) = input(prompt3);
end

%%


%For this implementation we will do a brute force optimization over the
%free paramters:
% c=[cx,cy] is the camera's center of projection in air [pixels]
% f is the camera's focal length in air [mm]
% d is the distance from the interface to the center of projection of the
% lens [mm]

if cam==1
cx=cameraParams_overhead.PrincipalPoint(1); %center of projection at origin overhead from matlab toolbox
cy=cameraParams_overhead.PrincipalPoint(2); %center of projection at origin overhead from matlab toolbox
f=mean(cameraParams_overhead.FocalLength); %overhead from camera matlab toolbox
d=linspace(1200,1500,1000);

elseif cam==0
cx=cameraParams_side.PrincipalPoint(1); %center of projection at origin side from matlab toolbox
cy=cameraParams_side.PrincipalPoint(2); %center of projection at origin sidefrom matlab toolbox
f=mean(cameraParams_side.FocalLength); %side from camera matlab toolbox
d=linspace(100,300,1000);
end


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

[Y,I]=min(rmse);
error=Y;

d_opt=d(I);

end

