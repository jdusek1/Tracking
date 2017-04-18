% function [world_outputs]=Videos_to_xyz(start_time_o,end_time_o,start_time_s,end_time_s,frame_rate_divider)
%Analysis of overhead and side videos and conversion to xyz coordinates
%using refraction model

%Jeff Dusek
%11/29/16
clc, close all, clear variables
 
% thresh_low=2453;
% thresh_high=6769;

% %underwater square 1
% start_time_o=35;
% end_time_o=210;
% start_time_s=37.75;
% end_time_s=212.75;
% frame_rate_divider=3;

% %underwater square 2
% start_time_o=35;
% end_time_o=278;
% start_time_s=38;
% end_time_s=281;
% frame_rate_divider=3;

% %Underwater multiple depths 0003
% start_time_o=58;
% end_time_o=260;
% start_time_s=60.8;
% end_time_s=262.8;
% frame_rate_divider=3;

% %Diving 20 to 70
% start_time_o=20.35;
% end_time_o=355.35;
% start_time_s=23.53;
% end_time_s=358.53;
% frame_rate_divider=3;

% %Hovering
% start_time_o=17;
% end_time_o=359.99;
% start_time_s=20.17;
% end_time_s=362.5;
% frame_rate_divider=3;

% LED_time_o=14.33;
% LED_time_s=17.5;

%Caudal Turn 1
start_time_o=15.5;
end_time_o=40.5;
start_time_s=20;
end_time_s=65;
frame_rate_divider=1;


%% Load camera calibration parameters found using Matlab image processing toolbox
load cameraParams_overhead
load cameraParams_side

%% Experimental setup paramaters 

% % Values for d_o and d_s can be found using the function "OptimizeD" if
% not known
d_o=1349.2; %Distance from overhead camera to interface in millimeters
d_s=205.91; %Distance from side camera to interface in millimeters
z_free=438.15; %Distance from free surface to side camera axis in millimeters

%% Select fish data file
%choose data file 
% [pname, ppath] = uigetfile('.txt', 'Pick data file');
%% Select the overhead video file and designate the region of interest

%select overhead video file to get the path
[Vname_o, Vpath_o] = uigetfile('.avi', 'Pick overhead movie file');
file_number=str2double(Vname_o(end-9));

%open up a frame from the video and select the region to analyze
vidObj_o=VideoReader(Vname_o);

% %grab the position in pixels of four points to define the tank
% %area for the overhead camera to make the detection of the fish more robust

vidObj_o.CurrentTime=0; %beginning of the video
vid_im=readFrame(vidObj_o);
vid_im=undistortImage(vid_im,cameraParams_overhead);
figure(1)
imagesc(vid_im);
for n=1:4
H=impoint;
pos_o(n,:)=wait(H);
end
% clf 
% clear vid_im
% clear H

%% Select the side video file 

%select side video file to get the path
[Vname_s, Vpath_s] = uigetfile('.avi', 'Pick side movie file');

%open up a frame from the video and select the region to analyze
vidObj_s=VideoReader(Vname_s);

close all
%% Remove background from video

%Start frame for overhead camera
start_frame_o=start_time_o*vidObj_o.framerate;
end_frame_o=end_time_o*vidObj_o.framerate;

%Start frame for side camera
start_frame_s=start_time_s*vidObj_s.framerate;
end_frame_s=end_time_s*vidObj_s.framerate;

%Number of frames to use
frames=(end_time_o-start_time_o)*vidObj_o.framerate/frame_rate_divider;

cc_o=linspace(start_frame_o,end_frame_o,frames); 
cc_s=linspace(start_frame_s,end_frame_s,frames);

%Find the average frame across the full video 
AvgCframe_o=zeros(abs(round(min(pos_o(:,2)))-round(max(pos_o(:,2))))+1,abs(round(min(pos_o(:,1)))-round(max(pos_o(:,1))))+1,3);
h = waitbar(0,'Overhead Background Averaging Progress');
for n=1:size(cc_o,2)
    vidObj_o.CurrentTime=cc_o(n)/vidObj_o.framerate;
    cdata_o = readFrame(vidObj_o);
    cdata_o = undistortImage(cdata_o,cameraParams_overhead);
    cdata_crop_o=cdata_o(round(min(pos_o(:,2))):round(max(pos_o(:,2))),round(min(pos_o(:,1))):round(max(pos_o(:,1))),:);
    
    AvgCframe_o=AvgCframe_o+double(cdata_crop_o);
    waitbar(n / size(cc_o,2))
end
close(h)
AvgCframe_o=double(AvgCframe_o)/size(cc_o,2);
clear n

AvgCframe_s=zeros(vidObj_s.Height,vidObj_s.Width,3);
h = waitbar(0,'Side Background Averaging Progress');
for n=1:size(cc_s,2)
    vidObj_s.CurrentTime=cc_s(n)/vidObj_s.framerate;
    cdata_s = readFrame(vidObj_s);
    cdata_s = undistortImage(cdata_s,cameraParams_side);
    
    AvgCframe_s=AvgCframe_s+double(cdata_s);
    waitbar(n / size(cc_s,2))
end
close(h)
AvgCframe_s=double(AvgCframe_s)/size(cc_s,2);
clear n
clear c

%% Remove the background from frames to isolate the robot and track centroid
h = waitbar(0,'Background removal and centroid tracking progress');
c=1;
for n=1:frames
    % Read in the overhead frame, crop to desired size, remove background
    vidObj_o.CurrentTime=cc_o(n)/vidObj_o.framerate;
    frame_o=readFrame(vidObj_o);
    frame_o = undistortImage(frame_o,cameraParams_overhead);
    frame_o=frame_o(round(min(pos_o(:,2))):round(max(pos_o(:,2))),round(min(pos_o(:,1))):round(max(pos_o(:,1))),:);
    frame_o=uint8(-double(frame_o)+(AvgCframe_o));
    
    % Read in the side frame,remove background
    vidObj_s.CurrentTime=cc_s(n)/vidObj_s.framerate;
    frame_s=readFrame(vidObj_s);
    frame_s = undistortImage(frame_s,cameraParams_side);
    frame_s=uint8(-double(frame_s)+(AvgCframe_s));
    
    % Blur image
    Tfilt_o=imgaussfilt(frame_o,4);
    Tfilt_s=imgaussfilt(frame_s,4);
    
    % convert to HSV
    I_o=rgb2hsv(Tfilt_o);
    I_s=rgb2hsv(Tfilt_s);

%%%Thresholds in HSV with lights on
    
    % Define thresholds for channel 1 based on histogram settings
    channel1Min_o = 0.113;
    channel1Max_o = 0.415;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min_o = 0.000;
    channel2Max_o = 0.947;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min_o = 0.049;
    channel3Max_o = 0.422;

   % Define thresholds for channel 1 based on histogram settings
    channel1Min_s = 0.191;
    channel1Max_s = 0.539;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min_s = 0.194;
    channel2Max_s = 0.859;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min_s = 0.134;
    channel3Max_s = 1.000;

    % Create mask based on chosen histogram thresholds
    BW_o = (I_o(:,:,1) >= channel1Min_o ) & (I_o(:,:,1) <= channel1Max_o) & ...
    (I_o(:,:,2) >= channel2Min_o ) & (I_o(:,:,2) <= channel2Max_o) & ...
    (I_o(:,:,3) >= channel3Min_o ) & (I_o(:,:,3) <= channel3Max_o);

    % Create mask based on chosen histogram thresholds
    BW_s = (I_s(:,:,1) >= channel1Min_s ) & (I_s(:,:,1) <= channel1Max_s) & ...
    (I_s(:,:,2) >= channel2Min_s ) & (I_s(:,:,2) <= channel2Max_s) & ...
    (I_s(:,:,3) >= channel3Min_s ) & (I_s(:,:,3) <= channel3Max_s);

    % Initialize output masked image based on input image.
    maskedRGBImage_o  = Tfilt_o;
    maskedRGBImage_s  = Tfilt_s;

    % Set background pixels where BW is false to zero.
    maskedRGBImage_o(repmat(~BW_o,[1 1 3])) = 0;
    maskedRGBImage_s(repmat(~BW_s,[1 1 3])) = 0;
    
    % Find the blob properties of the fish using binary
    s_fish_o = regionprops(BW_o,'centroid','Extrema','MajorAxisLength',...
    'MinorAxisLength','Orientation');

    s_fish_s = regionprops(BW_s,'centroid','Extrema','MajorAxisLength',...
    'MinorAxisLength','Orientation');

    % find maximum of major axis length
    maj_axis_o=cell2mat({s_fish_o.MajorAxisLength});
    [x_majaxis_o,I_majaxis_o]=max(maj_axis_o);
    
    % For side camera choose lower centroid to account for possible
    % reflection
    for nn=1:size(cell2mat({s_fish_s.Centroid}),2)/2;
        maj_axis_s(nn)=s_fish_s(nn).MajorAxisLength;
        centroids_s(nn,:)=s_fish_s(nn).Centroid;
    end
    
    [big_s,I_big_s]=find(maj_axis_s > 20);
    big_centroids_s=centroids_s(I_big_s,:);
    [centroidy_s,I_centroidy_s]=max(big_centroids_s(:,2));
    bodylengths=maj_axis_s(I_big_s);
    bodylength(c)=maj_axis_s(I_centroidy_s);

    % Find Centroid
    centroids_fish_o(c,:)=s_fish_o(I_majaxis_o).Centroid;
    centroids_fish_s(c,:)=centroids_s(I_centroidy_s,:);
    
    %add a check for invalid centroid positions based on velocity
    if c==1
        %for timestep one we assign velocity as a NaN
        v_pix_s(c)=NaN;  
        v_compare=NaN;
        centroid_compare_s(c,:)=centroids_fish_s(1,:);
        centroid_compare_o(c,:)=centroids_fish_o(1,:);
    else
        %find the timestep between frames being processed
        dt=((cc_o(n)/vidObj_o.framerate)-(cc_o(n-1)/vidObj_o.framerate));

        %calculate velocity in pixels/s
        v_pix_s(c)=sqrt((centroids_fish_s(c,1)-centroid_compare_s(c-1,1))^2+(centroids_fish_s(c,2)-centroid_compare_s(c-1,2))^2)/dt;
        v_pix_o(c)=sqrt((centroids_fish_o(c,1)-centroid_compare_o(c-1,1))^2+(centroids_fish_o(c,2)-centroid_compare_o(c-1,2))^2)/dt;
        
%         if v_pix_s(c) > 700 %%%might need to work on what this threshold should actually be
%                 centroids_fish_o(c,:)=[NaN,NaN];
%                 centroids_fish_s(c,:)=[NaN,NaN];
%                 disp(['Violates velocity in side view at c=',num2str(c)])
%         end
%         
%         if v_pix_o(c) > 700 %%%might need to work on what this threshold should actually be
%                 centroids_fish_o(c,:)=[NaN,NaN];
%                 centroids_fish_s(c,:)=[NaN,NaN];
%                 disp(['Violates velocity in overhead view at c=',num2str(c)])
%         end
        
        if isnan(centroids_fish_s(c,1)) == 0
            centroid_compare_s(c,:)=centroids_fish_s(c,:);
            centroid_compare_o(c,:)=centroids_fish_o(c,:);
        else
            centroid_compare_s(c,:)=centroid_compare_s(c-1,:);
            centroid_compare_o(c,:)=centroid_compare_o(c-1,:);
        end    
    end   
    
c=c+1;
waitbar(n / size(cc_o,2))
end
close(h)
%% Adjust centroid locations for cropped image and filter

% % Adjust the pixel locations of the centroids to the full image size 
centroids_fish_o_full=[centroids_fish_o(:,1)+round(min(pos_o(:,1))),centroids_fish_o(:,2)+round(min(pos_o(:,2)))];


%Filtering plus interpolation of NaNs
[centroids_fish_o_filt]=nanmoving_average(centroids_fish_o_full,10,1,1);
[centroids_fish_s_filt]=nanmoving_average(centroids_fish_s,10,1,1);

%% Pass the centroid locations to the refraction function and get xyz coordinates

h = waitbar(0,'Conversion to World Coordinates');
c=1;
for dd=1:length(centroids_fish_o_filt)
    if isnan(centroids_fish_s(c,1)) == 1
        world_outputs(dd,:)= NaN(1,6);
    else
    [world_outputs(dd,1),world_outputs(dd,2),world_outputs(dd,3),world_outputs(dd,4),world_outputs(dd,5),world_outputs(dd,6)] = Camera_Refraction(centroids_fish_o_filt(dd,1),size(cdata_o,1)-centroids_fish_o_filt(dd,2),centroids_fish_s_filt(dd,1),size(cdata_s,1)-centroids_fish_s_filt(dd,2),z_free,d_o,d_s);
    end
    c=c+1;
    waitbar(dd / length(centroids_fish_o_filt))
end
close (h)

%% make sweet movie
% Tracking_vids_3D(world_outputs(:,4),world_outputs(:,5),world_outputs(:,6),centroids_fish_o_filt,centroids_fish_s_filt,Vname_o,Vname_s,file_number,start_time_o,end_time_o,start_time_s,end_time_s,frame_rate_divider)

%% Generate plot of trajectory

figure;
ax3=gca;
axis(ax3,[-100 1900 0 1900 -10 915])
XL = get(gca, 'XLim');
YL = get(gca, 'YLim');
patch([XL(1), XL(2), XL(2), XL(1)], [YL(1), YL(1), YL(2), YL(2)], [0 0 0 0 ], 'FaceColor', [0.87058824300766 0.921568632125854 0.980392158031464],'FaceAlpha',0.4);
hold on
plot3(world_outputs(:,4),world_outputs(:,5),world_outputs(:,6),'-*r')
grid on
title('Robot Location in Tank Frame')
xlabel('x-location [mm]')
ylabel('y-location [mm]')
zlabel('Depth[mm]')
set(ax3,'zdir','reverse')

%% Pressure comparison plot
% [depth_p,t_p]=pressure_depth(pname,world_outputs(:,6),dt,frames,thresh_low,thresh_high);
%