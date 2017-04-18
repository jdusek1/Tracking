%Jeff Dusek
%11/21/16

%This code will be used to carry out camera calibration for robot tracking
%in the water lab taking into account the refraction due to the fluid. The
%method will be based on Treibitz et al. (2008). This method assumes that
%the refraction interface is perpendicular to the camera axis, and the
%refraction from the glass window will be neglected as in the paper.
%Variable names will follow the paper as well.

clc, clear all, close all

%Basic parameters
n=4/3; %index of refraction of the water

%Need the distance of the image to the interface (depth) in mm
z_w=889;

%line segments to use
lines=8;

%First we will select two points of known separation distance from a camera image.
%These points will be know as pos(n,:)=[(xi,n1),(yi,n1),(xi,n2),(yi,n2)],
%where n1 and 2 are the end points of a line n with known true length
%s_known(n).

%select first an image file to get the path
% [imname, impath] = uigetfile('.tif', 'Pick an image file');
% 
% T=imread([impath,imname]);
% imagesc(T);
% I=getframe(gcf);
% 
% for n=1:lines
%     for m=1:2:3
%     H=impoint;
%     pos(n,m:m+1)=getPosition(H);
%     end
%  
%     prompt = sprintf('What is the true length of the point pair number %d in mm: ',n);
%     s_known(n) = input(prompt);
% end
%%
load pos_refrac 
load s_known


%For this implementation we will do a brute force optimization over the
%free paramters:
% c=[cx,cy] is the camera's center of projection in air [pixels]
% f is the camera's focal length in air [mm]
% d is the distance from the interface to the center of projection of the
% lens [mm]
cx=973.54; %center of projection at origin
cy=548.46; %center of projection at origin
f=955.30; 
% d=linspace(1700,2000,1000);
% d=11829878.9;   
% d=1500

for nn=1:lines %line segments defined by image    
                    %radial position in image frame with center of projection
                    %varied. n1 and n2 are the two points defining line n.
                    %checked
                    ri_n1(nn)=sqrt((pos(nn,1)-cx)^2+(pos(nn,2)-cy)^2);
                    ri_n2(nn)=sqrt((pos(nn,3)-cx)^2+(pos(nn,4)-cy)^2);
                    
                    %Angular position of n1 and n2 as center of projection
                    %is varied. n1 and n2 are the two points defining line n
                    %checked
                    theta1(nn)=atan2(pos(nn,2)-cy,(pos(nn,1)-cx));
                    theta2(nn)=atan2(pos(nn,4)-cy,(pos(nn,3)-cx));
                    
                    %positions in world frame. n1 and n2 are the two points defining line n 
                    rw_n1(nn)=d/f*ri_n1(nn)+((z_w)/sqrt(((f*n)/ri_n1(nn))^2+n^2-1));
                    rw_n2(nn)=d/f*ri_n2(nn)+((z_w)/sqrt(((f*n)/ri_n2(nn))^2+n^2-1));
                    
                    %length of line n where n1 and n2 are the two points defining line n 
                    s(nn)=sqrt(rw_n1(nn)^2+rw_n2(nn)^2-...
                        2*rw_n1(nn)*rw_n2(nn)*cos(abs(theta1(nn)-theta2(nn))));
                    
                    error(nn)=s_known(nn)-s(nn);
end
                    
rmse=sqrt(sum(error.^2)/lines);
      
% rmse_2=squeeze(rmse(1,1,:,:));
% [F,D]=meshgrid(f,d);
% 
% figure;
% contour(F,D,rmse_2',100)
% xlabel('f')
% ylabel('d')
% colorbar

figure;
plot(d,squeeze(rmse(1,1,1,:)))
xlabel('d [mm]')
ylabel('RMSE [mm]')

