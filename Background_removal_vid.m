%Background removal of videos from new testing tank

%Jeff Dusek
%11/7/16
clc, clear all, close all

%load camera parameters
load cameraParams_overhead

%select first an image file to get the path
[Vname, Vpath] = uigetfile('.avi', 'Pick movie file');

%open up a frame from the video and select the region to analyze
vidObj=VideoReader(Vname);

%grab the position in pixels of four points to define the tank
%area to make the detection of the fish more robust
vidObj.CurrentTime=0; %beginning of the video
vid_im=readFrame(vidObj);
vid_im=undistortImage(vid_im,cameraParams_overhead);
figure(1)
imagesc(vid_im);
for n=1:4
H=impoint;
pos(n,:)=wait(H);
end
clf

%%

start_time=0;
start_frame=start_time*vidObj.framerate;

end_time=270;
end_frame=end_time*vidObj.framerate;

frames=(end_time-start_time)*vidObj.framerate;

cc=linspace(start_frame,end_frame,frames);

AvgCframe=zeros(abs(round(min(pos(:,2)))-round(max(pos(:,2))))+1,abs(round(min(pos(:,1)))-round(max(pos(:,1))))+1,3);
h = waitbar(0,'Background Averaging Progress');
for n=1:size(cc,2)
    vidObj.CurrentTime=cc(n)/vidObj.framerate;
    cdata = readFrame(vidObj);
    cdata = undistortImage(cdata,cameraParams_overhead);
    cdata_crop=cdata(round(min(pos(:,2))):round(max(pos(:,2))),round(min(pos(:,1))):round(max(pos(:,1))),:);
    
    AvgCframe=AvgCframe+double(cdata_crop);
    waitbar(n / size(cc,2))
end
close(h)
AvgCframe=double(AvgCframe)/size(cc,2);
clear n

%% Remove the background from frames to isolate the robot
h = waitbar(0,'Background removal Centroid Extraction');
c=1;
for n=1:frames
    % Read in the overhead frame, crop to desired size, remove background
    vidObj.CurrentTime=cc(n)/vidObj.framerate;
    frame=readFrame(vidObj);
    frame = undistortImage(frame,cameraParams_overhead);
    frame=frame(round(min(pos(:,2))):round(max(pos(:,2))),round(min(pos(:,1))):round(max(pos(:,1))),:);
    frame=uint8(-double(frame)+(AvgCframe));

    % Blur image
    Tfilt=imgaussfilt(frame,4);
    
    % convert to HSV
    I=rgb2hsv(Tfilt);
    
    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.113;
    channel1Max = 0.415;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.000;
    channel2Max = 0.947;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min = 0.049;
    channel3Max = 0.422;

    % Create mask based on chosen histogram thresholds
    BW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min) & (I(:,:,3) <= channel3Max);

    % Initialize output masked image based on input image.
    maskedRGBImage  = Tfilt;

    % Set background pixels where BW is false to zero.
    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
    
    % Find the blob properties of the fish using binary
    s_fish = regionprops(BW,'centroid','Extrema','MajorAxisLength',...
    'MinorAxisLength','Orientation');

    % find maximum of major axis length
    maj_axis=cell2mat({s_fish.MajorAxisLength});
    [x_majaxis,I_majaxis]=max(maj_axis);
    

    % Find Centroid
    centroids_fish(c,:)=s_fish(I_majaxis).Centroid;
    
    %add a correction for centroid position based on velocity
    
    if c==1
        %for timestep one we assign velocity as a NaN
        centroid_compare(c,:)=centroids_fish(1,:);
    else
        %find the timestep between frames being processed
        dt=((cc(n)/vidObj.framerate)-(cc(n-1)/vidObj.framerate));

        %calculate velocity in pixels/s
        v_pix(c)=sqrt((centroids_fish(c,1)-centroid_compare(c-1,1))^2+(centroids_fish(c,2)-centroid_compare(c-1,2))^2)/dt;
        
        
        if v_pix(c) > 500 %%%might need to work on what this threshold should actually be
                centroids_fish(c,:)=[NaN,NaN];
                disp(['Violates velocity in overhead view at c=',num2str(c)])
        end
        
        if isnan(centroids_fish(c,1)) == 0
            centroid_compare(c,:)=centroids_fish(c,:);
        else
            centroid_compare(c,:)=centroid_compare(c-1,:);
        end    
    end 
c=c+1;
waitbar(n / size(cc,2))
end
close(h)
%% Adjust centroid locations for cropped image and filter

% % Adjust the pixel locations of the centroids to the full image size 
centroids_fish_full=[centroids_fish(:,1)+round(min(pos(:,1))),centroids_fish(:,2)+round(min(pos(:,2)))];


%Filtering plus interpolation of NaNs
[centroids_fish_filt]=nanmoving_average(centroids_fish_full,15,1,1);

%% Convert to real world with the assumption of fish at surface
f_o=894.4931;
d_o=1349.2;

centroids_fish_world=(d_o/f_o).*centroids_fish_filt;

%% Generate figures

h = waitbar(0,'Plot progress');
c=1;
for n=1:60:frames   
%     fig1=figure('visible','off');
%     ax(1)=subplot(2,2,1,'Parent',fig1);
%     imshow(read(vidObj,cc(n)))
%     title('Original Image')
%     ax(2)=subplot(2,2,2,'parent',fig1);
%     imshow(Tfilt)
%     title('Blurred Image')
%     ax(3)=subplot(2,2,3,'parent',fig1);
%     imshow(maskedRGBImage)
%     title('Masked')
%     ax(4)=subplot(2,2,4,'parent',fig1);
%     imshow(BW)
%     hold on 
%     title('Tracking Centroid')
%     plot(centroids_fish(c,1),centroids_fish(c,2), 'r*')
%     axis tight
%     set(gcf,'Position',[500 500 1000 500])
% 
%     F(c)=getframe(fig1);
%     hold off
    %%
    
%     fig2=figure('visible','off');
%     imshow(BW)
%     hold on
%     phi = linspace(0,2*pi,50);
%     cosphi = cos(phi);
%     sinphi = sin(phi);
% 
%     xbar = s_fish(I_majaxis).Centroid(1);
%     ybar = s_fish(I_majaxis).Centroid(2);
% 
%     a = s_fish(I_majaxis).MajorAxisLength/2;
%     b = s_fish(I_majaxis).MinorAxisLength/2;
% 
%     thetad=s_fish(I_majaxis).Orientation;
%     theta(c) = pi*thetad/180;
%     R = [ cos(theta(c))   sin(theta(c))
%          -sin(theta(c))   cos(theta(c))];
% 
%     xy = [a*cosphi; b*sinphi];
%     xy = R*xy;
% 
%     x = xy(1,:) + xbar;
%     y = xy(2,:) + ybar;
% 
%     plot(x,y,'r','LineWidth',2);
%     hold off
%     
%     F2(c)=getframe(fig2);
%%
%     %plot position
%     fig3=figure('visible','off');
%     plot(centroids_fish(1:c,1),centroids_fish(1:c,2),'r-*','linewidth',2)
%     legend('Centroid Positions')
%     title('Fish Position')
%     xlabel('Pixels')
%     ylabel('Pixels')
%     axis ij
%     axis([-50 650 200 900])
% 
%     F3(c)=getframe(fig3);
% %%
% Read in the overhead frame
    vidObj.CurrentTime=cc(n)/vidObj.framerate;
    frame = readFrame(vidObj);
    frame = undistortImage(frame,cameraParams_overhead);

    fig1=figure('visible','off');
    positionVector1=[0.1,0.1,0.45,0.85];
    ax(1)=subplot('Position',positionVector1);
    imshow(frame)
    hold on 
    plot(centroids_fish_filt(1:n,1),centroids_fish_filt(1:n,2), 'r','linewidth',2)
    title('Overhead View')
    axis tight
    
    positionVector2=[0.6,0.2,0.3,0.6];
    ax(2)=subplot('Position',positionVector2);
    plot(centroids_fish_world(1:n,1),centroids_fish_world(1:n,2),'-*r')
    grid on
    title('Robot Location')
    xlabel('x-location [mm]')
    ylabel('y-location [mm]')
    axis ij
    axis([900 2100 300 1500])
    
    set(gcf,'Position',[500 500 1000 500],'OuterPosition',[500 500 1100 600])
    
    F(c)=getframe(fig1);


c=c+1;
waitbar(n / size(cc,2))
end
close(h) 

movie2avi(F,'Location_Squares','FPS',10)
% movie2avi(F2,'BW_mask_long','FPS',30)
% movie2avi(F3,'Centroid_Locations_long','FPS',30)
%%

%plot position
figure;
plot(centroids_fish_world(:,1),centroids_fish_world(:,2),'r-*')
legend('Centroid Positions')
title('Fish Position')
xlabel('[mm]')
ylabel('[mm]')
axis ij
axis equal

