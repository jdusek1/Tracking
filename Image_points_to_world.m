%Test theory about camera rotation

clear variables, close all

%% Load camera calibration parameters found using Matlab image processing toolbox
load cameraParams_overhead
load cameraParams_side

%% Select the overhead video file and select points

%select overhead video file to get the path
[Vname_o, Vpath_o] = uigetfile('.avi', 'Pick overhead movie file');
file_number=str2double(Vname_o(end-9));

%open up a frame from the video and select the region to analyze
vidObj_o=VideoReader(Vname_o);

% %grab the position in pixels of 4 points from the overhead view

vidObj_o.CurrentTime=35; 
vid_im=readFrame(vidObj_o);
vid_im=undistortImage(vid_im,cameraParams_overhead);
figure(1)
imagesc(vid_im);
for n=1:4
H=impoint;
pos_o(n,:)=wait(H);
end
clf 
% clear vid_im
% clear H
% 
% %% Select the side video file and select points
% 
% %select side video file to get the path
% [Vname_s, Vpath_s] = uigetfile('.avi', 'Pick side movie file');
% % load Vname_s;
% % load Vpath_s;
% 
% %open up a frame from the video and select the region to analyze
% vidObj_s=VideoReader(Vname_s);
% % %grab the position in pixels of 4 points from the overhead view
% 
% vidObj_s.CurrentTime=35; 
% vid_im=readFrame(vidObj_s);
% vid_im=undistortImage(vid_im,cameraParams_side);
% figure(1)
% imagesc(vid_im);
% for n=1:4
% H=impoint;
% pos_s(n,:)=wait(H);
% end
% clf 
% clear H

A=300:800;
B=1300*ones(size(A));

pos_o=[B',A'];

C=linspace(200,1600,length(A));
D=300*ones(size(C));

pos_s=[C',D'];

%% Convert coordinates
for dd=1:length(pos_o)
    [world_outputs(dd,1),world_outputs(dd,2),world_outputs(dd,3),world_outputs(dd,4),world_outputs(dd,5),world_outputs(dd,6)] = Camera_Refraction(pos_o(dd,1),size(vid_im,1)-pos_o(dd,2),pos_s(dd,1),size(vid_im,1)-pos_s(dd,2));
end
x_tank=world_outputs(:,4);
y_tank=world_outputs(:,5);
z_tank=world_outputs(:,6);

    positionVector3=[0.1,0.1,0.8,0.8];
    ax(3)=subplot('Position',positionVector3);
    axis(ax(3),[-100 1900 -100 1900 -210 915])
    XL = get(gca, 'XLim');
    YL = get(gca, 'YLim');
    patch([XL(1), XL(2), XL(2), XL(1)], [YL(1), YL(1), YL(2), YL(2)], [0 0 0 0 ], 'FaceColor', [0.87058824300766 0.921568632125854 0.980392158031464],'FaceAlpha',0.4);
    hold on
    plot3(x_tank,y_tank,z_tank,'-*r')
    grid on
    title('Robot Location in Tank Frame')
    xlabel('x-location [mm]')
    ylabel('y-location [mm]')
    zlabel('Depth[mm]')
    axis(ax(3),[-100 1900 0 1900 -210 915])
    set(ax(3),'zdir','reverse')
