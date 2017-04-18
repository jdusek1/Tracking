% function [world_outputs]=Videos_to_xyz(start_time_o,end_time_o,start_time_s,end_time_s,frame_rate_divider)
%Analysis of overhead and side videos and conversion to xyz coordinates
%using refraction model

%Jeff Dusek
%11/29/16
clc, close all, clear variables

% %Underwater multiple depths 0003
% start_time_o=58;
% end_time_o=260;
% start_time_s=60.8;
% end_time_s=262.8;
% frame_rate_divider=3;

%Underwater multiple depths 0004
% start_time_o=66;
% end_time_o=290;
% start_time_s=68.7;
% end_time_s=292.7;
% frame_rate_divider=3;

% %Homing nn1
% start_time_o=35.5;
% end_time_o=77;
% frame_rate_divider=1;

% %Homing nn2
% start_time_o=24.13;
% end_time_o=44;
% frame_rate_divider=1;

% %Homing ne1
% start_time_o=34.75;
% end_time_o=58;
% frame_rate_divider=1;

% %Homing ne2
% start_time_o=23.75;
% end_time_o=53;
% frame_rate_divider=1;

% %Homing ns1
% start_time_o=25.5;
% end_time_o=60;
% frame_rate_divider=1;

% %Homing ns2
% start_time_o=24.25;
% end_time_o=65;
% frame_rate_divider=1;

% %Homing nw1
% start_time_o=36;
% end_time_o=85;
% frame_rate_divider=1;

% %Homing nw1
% start_time_o=31.5;
% end_time_o=80;
% frame_rate_divider=1;

%Homing ee1
% start_time_o=18.36;
% end_time_o=44;
% frame_rate_divider=1;
% %Homing ee2
% start_time_o=22;
% end_time_o=46;
% frame_rate_divider=1;
% %Homing en1
% start_time_o=22.7;
% end_time_o=44.7;
% frame_rate_divider=1;
% %Homing en2
% start_time_o=24.7;
% end_time_o=64;
% frame_rate_divider=1;
% %Homing es1
% start_time_o=21;
% end_time_o=57;
% frame_rate_divider=1;
% %Homing es2
% start_time_o=18.2;
% end_time_o=66;
% frame_rate_divider=1;
% %Homing ew1
% start_time_o=29.65;
% end_time_o=67;
% frame_rate_divider=1;
% %Homing ew2
% start_time_o=17.2;
% end_time_o=80;
% frame_rate_divider=1;

%Homing wn1
% start_time_o=42.3;
% end_time_o=82;
% frame_rate_divider=1;
% %Homing wn2
% start_time_o=20.3;
% end_time_o=56;
% frame_rate_divider=1;
% %Homing we1
% start_time_o=19;
% end_time_o=36;
% frame_rate_divider=1;
% %Homing we2
% start_time_o=17.4;
% end_time_o=35;
% frame_rate_divider=1;
% %Homing ws1
% start_time_o=16.8;
% end_time_o=41;
% frame_rate_divider=1;
% %Homing ws2
% start_time_o=15.7;
% end_time_o=83;
% frame_rate_divider=1;
% %Homing ww1
% start_time_o=19.2;
% end_time_o=62;
% frame_rate_divider=1;
% %Homing ww2
% start_time_o=17.5;
% end_time_o=63;
% frame_rate_divider=1;

%Submerged square 6
start_time_o=37;
end_time_o=260;
frame_rate_divider=3;

%% Load camera calibration parameters found using Matlab image processing toolbox
load cameraParams_overhead
load cameraParams_side

%% Experimental setup paramaters 

% % Values for d_o and d_s can be found using the function "OptimizeD" if
% not known
d_o=1349.2; %Distance from overhead camera to interface in millimeters
d_s=205.91; %Distance from side camera to interface in millimeters
z_free=438.15; %Distance from free surface to side camera axis in millimeters

% %% Select fish data file to get pressure
% % % choose data file 
[pname, ppath] = uigetfile('.txt', 'Pick data file');

fish_data_full=csvread(pname,1);

% % Submerged Square 3- incremental
% LED_time_o=14;

% % % Submerged Square 4- incremental
% LED_time_o=20;
% 
% % %Homing nn1
% LED_time_o=35.5;
 % %Homing nn2
% LED_time_o=24.13;
% % %Homing ne1
% LED_time_o=34.75;
% % %Homing ne2
% LED_time_o=23.75;
% % %Homing ns1
% LED_time_o=25.5;
% % %Homing ns2
% LED_time_o=24.25;
% % %Homing nw1
% LED_time_o=36;
% % %Homing nw2
% LED_time_o=31.5;

% %Homing ee1
% LED_time_o=18.36;
% % %Homing ee2
% LED_time_o=19.7;
% % %Homing en1
% LED_time_o=22.7;
% % %Homing en2
% LED_time_o=24.7;
% % %Homing es1
% LED_time_o=21;
% % %Homing es2
% LED_time_o=18.2;
% % %Homing ew1
% LED_time_o=29.65;
% % %Homing ew2
% LED_time_o=17.2;

% %Homing wn1
% LED_time_o=42.3;
% % %Homing wn2
% LED_time_o=20.3;
% % %Homing we1
% LED_time_o=19;
% % %Homing we2
% LED_time_o=17.4;
% % %Homing ws1
% LED_time_o=16.8;
% % %Homing ws2
% LED_time_o=15.7;
% % %Homing ww1
% LED_time_o=17.4;
% % % %Homing ww2
% LED_time_o=17.5;


% % Submerged square 6
LED_time_o=15.25;

d_track_time_o=(start_time_o-LED_time_o)*10;
track_end_o=((end_time_o-start_time_o)*10)+d_track_time_o;

%fish data during track
fish_data_track_I=find(fish_data_full(:,1)>=d_track_time_o & fish_data_full(:,1)<=track_end_o);
fish_data=fish_data_full(fish_data_track_I,:);

%time vector from fish
t_p=(fish_data(:,1)-min(fish_data(:,1)))/10;

fish_data(:,1)=t_p;

%unfiltered pressure from fish pressure sensor
p=fish_data(:,4);
P_off=csvread(pname,0,0,[0 0 0 0]);

%Pressure offset is calculated when fish on surface and sensor 3.5cm below
%waterline
% offset=mean(fish_data_full(end-20:end,4));
offset=P_off;

%Remove offset from pressure measurements
p_zeroed=p-offset;

%convert to depth in mm for 3
depth_p=(p_zeroed/9.81)+35;

% %convert to depth in mm for 4
% depth_p=(p_zeroed/9.81);

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
clear n
%% Select the side video file 
% 
% %select side video file to get the path
% [Vname_s, Vpath_s] = uigetfile('.avi', 'Pick side movie file');
% 
% %open up a frame from the video and select the region to analyze
% vidObj_s=VideoReader(Vname_s);
% 
% vidObj_s.CurrentTime=0; %beginning of the video
% vid_im_s=readFrame(vidObj_s);
% vid_im_s=undistortImage(vid_im_s,cameraParams_side);
% figure(2)
% imagesc(vid_im_s);
% for n=1:4
% H_s=impoint;
% pos_s(n,:)=wait(H_s);
% end
% 
% close all
%% Remove background from video

%Start frame for overhead camera
start_frame_o=start_time_o*vidObj_o.framerate;
end_frame_o=end_time_o*vidObj_o.framerate;

% %Start frame for side camera
% start_frame_s=start_time_s*vidObj_s.framerate;
% end_frame_s=end_time_s*vidObj_s.framerate;

%Number of frames to use
frames=(end_time_o-start_time_o)*vidObj_o.framerate/frame_rate_divider;

cc_o=linspace(start_frame_o,end_frame_o,frames); 
% cc_s=linspace(start_frame_s,end_frame_s,frames);

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

% % AvgCframe_s=zeros(vidObj_s.Height,vidObj_s.Width,3);
% AvgCframe_s=zeros(abs(round(min(pos_s(:,2)))-round(max(pos_s(:,2))))+1,abs(round(min(pos_s(:,1)))-round(max(pos_s(:,1))))+1,3);
% h = waitbar(0,'Side Background Averaging Progress');
% for n=1:size(cc_s,2)
%     vidObj_s.CurrentTime=cc_s(n)/vidObj_s.framerate;
%     cdata_s = readFrame(vidObj_s);
%     cdata_s = undistortImage(cdata_s,cameraParams_side);
%     cdata_crop_s=cdata_s(round(min(pos_s(:,2))):round(max(pos_s(:,2))),round(min(pos_s(:,1))):round(max(pos_s(:,1))),:);
%     
%     AvgCframe_s=AvgCframe_s+double(cdata_crop_s);
%     waitbar(n / size(cc_s,2))
% end
% close(h)
% AvgCframe_s=double(AvgCframe_s)/size(cc_s,2);
% clear n
% clear c

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
    
%     % Read in the side frame,remove background
%     vidObj_s.CurrentTime=cc_s(n)/vidObj_s.framerate;
%     frame_s=readFrame(vidObj_s);
%     frame_s = undistortImage(frame_s,cameraParams_side);
%     frame_s=frame_s(round(min(pos_s(:,2))):round(max(pos_s(:,2))),round(min(pos_s(:,1))):round(max(pos_s(:,1))),:);
%     frame_s=uint8(-double(frame_s)+(AvgCframe_s));
%     
    % Blur image
    Tfilt_o=imgaussfilt(frame_o,4);
%     Tfilt_s=imgaussfilt(frame_s,4);
    
    % convert to HSV
%     I_o=rgb2hsv(Tfilt_o);
%     I_s=rgb2hsv(Tfilt_s);

% %%%Thresholds in HSV with lights on
%     
%     % Define thresholds for channel 1 based on histogram settings
%     channel1Min_o = 0.113;
%     channel1Max_o = 0.415;
% 
%     % Define thresholds for channel 2 based on histogram settings
%     channel2Min_o = 0.000;
%     channel2Max_o = 0.947;
% 
%     % Define thresholds for channel 3 based on histogram settings
%     channel3Min_o = 0.049;
%     channel3Max_o = 0.422;
% 
%    % Define thresholds for channel 1 based on histogram settings
%     channel1Min_s = 0.191;
%     channel1Max_s = 0.539;
% 
%     % Define thresholds for channel 2 based on histogram settings
%     channel2Min_s = 0.194;
%     channel2Max_s = 0.859;
% 
%     % Define thresholds for channel 3 based on histogram settings
%     channel3Min_s = 0.134;
%     channel3Max_s = 1.000;
%     

 I_o=Tfilt_o;
 
    % Define thresholds for channel 1 based on histogram settings
channel1Min_o = 35.000;
channel1Max_o = 109.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min_o = 22.000;
channel2Max_o = 136.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min_o = 12.000;
channel3Max_o = 129.000;

    % Create mask based on chosen histogram thresholds
    BW_o = (I_o(:,:,1) >= channel1Min_o ) & (I_o(:,:,1) <= channel1Max_o) & ...
    (I_o(:,:,2) >= channel2Min_o ) & (I_o(:,:,2) <= channel2Max_o) & ...
    (I_o(:,:,3) >= channel3Min_o ) & (I_o(:,:,3) <= channel3Max_o);
% 
%     % Create mask based on chosen histogram thresholds
%     BW_s = (I_s(:,:,1) >= channel1Min_s ) & (I_s(:,:,1) <= channel1Max_s) & ...
%     (I_s(:,:,2) >= channel2Min_s ) & (I_s(:,:,2) <= channel2Max_s) & ...
%     (I_s(:,:,3) >= channel3Min_s ) & (I_s(:,:,3) <= channel3Max_s);

    % Initialize output masked image based on input image.
    maskedRGBImage_o  = Tfilt_o;
%     maskedRGBImage_s  = Tfilt_s;

    % Set background pixels where BW is false to zero.
    maskedRGBImage_o(repmat(~BW_o,[1 1 3])) = 0;
%     maskedRGBImage_s(repmat(~BW_s,[1 1 3])) = 0;
    
    % Find the blob properties of the fish using binary
    s_fish_o = regionprops(BW_o,'centroid','Extrema','MajorAxisLength',...
    'MinorAxisLength','Orientation');
% 
%     s_fish_s = regionprops(BW_s,'centroid','Extrema','MajorAxisLength',...
%     'MinorAxisLength','Orientation');

% % ORIGINAL CODE
    % find maximum of major axis length
    maj_axis_o=cell2mat({s_fish_o.MajorAxisLength});
    [x_majaxis_o,I_majaxis_o]=max(maj_axis_o);
%%%%%%
   
    
%     % For side camera choose lower centroid to account for possible
%     % reflection
%     for nn=1:size(cell2mat({s_fish_s.Centroid}),2)/2;
%         maj_axis_s(nn)=s_fish_s(nn).MajorAxisLength;
%         centroids_s(nn,:)=s_fish_s(nn).Centroid;
%     end
%     
%     [big_s,I_big_s]=find(maj_axis_s > 20);
%     big_centroids_s=centroids_s(I_big_s,:);
%     [centroidy_s,I_centroidy_s]=max(big_centroids_s(:,2));
%     bodylengths=maj_axis_s(I_big_s);
%     bodylength(c)=maj_axis_s(I_centroidy_s);

    % Find Centroid
    %%%%%%%
    centroids_fish_o(c,:)=s_fish_o(I_majaxis_o).Centroid;

%     centroids_fish_s(c,:)=centroids_s(I_centroidy_s,:);
    
    %add a check for invalid centroid positions based on velocity
    if c==1
        %for timestep one we assign velocity as a NaN
%         v_pix_s(c)=NaN;  
        v_compare=NaN;
%         centroid_compare_s(c,:)=centroids_fish_s(1,:);
        centroid_compare_o(c,:)=centroids_fish_o(1,:);
    else
        %find the timestep between frames being processed
        dt=((cc_o(n)/vidObj_o.framerate)-(cc_o(n-1)/vidObj_o.framerate));

        %calculate velocity in pixels/s
%         v_pix_s(c)=sqrt((centroids_fish_s(c,1)-centroid_compare_s(c-1,1))^2+(centroids_fish_s(c,2)-centroid_compare_s(c-1,2))^2)/dt;
        v_pix_o(c)=sqrt((centroids_fish_o(c,1)-centroid_compare_o(c-1,1))^2+(centroids_fish_o(c,2)-centroid_compare_o(c-1,2))^2)/dt;
%         
%         if v_pix_s(c) > 700 %%%might need to work on what this threshold should actually be
%                 centroids_fish_o(c,:)=[NaN,NaN];
%                 centroids_fish_s(c,:)=[NaN,NaN];
%                 disp(['Violates velocity in side view at c=',num2str(c)])
%         end
%         
        if v_pix_o(c) > 700 %%%might need to work on what this threshold should actually be
                centroids_fish_o(c,:)=[NaN,NaN];
%                 centroids_fish_s(c,:)=[NaN,NaN];
                disp(['Violates velocity in overhead view at c=',num2str(c)])
        end
        
        if isnan(centroids_fish_o(c,1)) == 0
%             centroid_compare_s(c,:)=centroids_fish_s(c,:);
            centroid_compare_o(c,:)=centroids_fish_o(c,:);
        else
%             centroid_compare_s(c,:)=centroid_compare_s(c-1,:);
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
% centroids_fish_s_full=[centroids_fish_s(:,1)+round(min(pos_s(:,1))),centroids_fish_s(:,2)+round(min(pos_s(:,2)))];


%Filtering plus interpolation of NaNs
[centroids_fish_o_filt]=nanmoving_average(centroids_fish_o_full,10,1,1);
% [centroids_fish_s_filt]=nanmoving_average(centroids_fish_s_full,10,1,1);

% %% Pass the centroid locations to the refraction function and get xyz coordinates
% 
% h = waitbar(0,'Conversion to World Coordinates');
% c=1;
% for dd=1:length(centroids_fish_o_filt)
%     if isnan(centroids_fish_s(c,1)) == 1
%         world_outputs(dd,:)= NaN(1,6);
%     else
%     [world_outputs(dd,1),world_outputs(dd,2),world_outputs(dd,3),world_outputs(dd,4),world_outputs(dd,5),world_outputs(dd,6)] = Camera_Refraction(centroids_fish_o_filt(dd,1),size(cdata_o,1)-centroids_fish_o_filt(dd,2),centroids_fish_s_filt(dd,1),size(cdata_s,1)-centroids_fish_s_filt(dd,2),z_free,d_o,d_s);
%     end
%     c=c+1;
%     waitbar(dd / length(centroids_fish_o_filt))
% end
% close (h)
% clear dd
% clear c
%% Pass the centroid locations to the refraction function and get xyz coordinates using pressure for depth

%time for camera tracking
t_track=0:dt:dt*(frames-1);

%Intepolate pressure data to the image times
depth_p_interp=interp1(t_p,depth_p,t_track);

h = waitbar(0,'Conversion to World Coordinates');
c=1;
for dd=1:length(centroids_fish_o_filt)
    if isnan(centroids_fish_o(c,1)) == 1
        outputs_press(dd,:)= NaN(1,3);
    else
    [outputs_press(dd,1),outputs_press(dd,2),outputs_press(dd,3)] = Camera_Refraction_pressure_depth(centroids_fish_o_filt(dd,1),size(cdata_o,1)-centroids_fish_o_filt(dd,2),depth_p_interp(dd),d_o);
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
% h1=plot3(world_outputs(:,4),world_outputs(:,5),world_outputs(:,6),'-*r')
h2=plot3(outputs_press(:,1),outputs_press(:,2),outputs_press(:,3),'-*b')
grid on
title('Robot Location in Tank Frame')
xlabel('x-location [mm]')
ylabel('y-location [mm]')
zlabel('Depth[mm]')
set(ax3,'zdir','reverse')
% legend([h1 h2],'Depth from tracking','Depth from Pressure')
% 
% figure;
% % ax3=gca;
% % axis(ax3,[-100 1900 0 1900 -10 915])
% plot(world_outputs(:,4),world_outputs(:,5),'r')
% hold on
% plot(outputs_press(:,1),outputs_press(:,2),'b')
% grid on
% title('Robot Location in Tank Frame')
% xlabel('x-location [mm]')
% ylabel('y-location [mm]')
% legend('Depth from tracking','Depth from Pressure')

%% Pressure comparison plot
% [depth_p,t_p]=pressure_depth(pname,world_outputs(:,6),dt,frames,thresh_low,thresh_high);
