function Tracking_vids_3D(x_tank,y_tank,z_tank,centroids_fish_o_filt,centroids_fish_s_filt,Vname_o,Vname_s,file_number,start_time_o,end_time_o,start_time_s,end_time_s,frame_rate_divider)
% Make sweet video of fish tracking

% Jeff Dusek
% 12/1/16


%% Load camera calibration parameters found using Matlab image processing toolbox
load cameraParams_overhead
load cameraParams_side
%%

vidObj_o=VideoReader(Vname_o);
vidObj_s=VideoReader(Vname_s);

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

h = waitbar(0,'Creating Sweet Figure');
c=1;
for n=1:10:frames
    % Read in the overhead frame
    vidObj_o.CurrentTime=cc_o(n)/vidObj_o.framerate;
    frame_o = readFrame(vidObj_o);
    frame_o = undistortImage(frame_o,cameraParams_overhead);
   
    % Read in the side framend
    vidObj_s.CurrentTime=cc_s(n)/vidObj_s.framerate;
    frame_s=readFrame(vidObj_s);
    frame_s = undistortImage(frame_s,cameraParams_side);

    fig1=figure('visible','off');
    positionVector1=[0.1,0.4,0.3,0.7];
    ax(1)=subplot('Position',positionVector1);
    imshow(frame_o)
    hold on 
    plot(centroids_fish_o_filt(1:n,1),centroids_fish_o_filt(1:n,2), 'r','linewidth',2)
    title('Overhead View')
    axis tight
    
    positionVector2=[0.1,0.05,0.3,0.6];
    ax(2)=axes('Position',positionVector2);
    imshow(frame_s)
    hold on 
    plot(centroids_fish_s_filt(1:n,1),centroids_fish_s_filt(1:n,2), 'r','linewidth',2) 
    title('Side View')
    axis tight
    
    positionVector3=[0.5,0.1,0.4,0.8];
    ax(3)=subplot('Position',positionVector3);
    axis(ax(3),[-100 1900 -100 1900 -10 915])
    XL = get(gca, 'XLim');
    YL = get(gca, 'YLim');
    patch([XL(1), XL(2), XL(2), XL(1)], [YL(1), YL(1), YL(2), YL(2)], [0 0 0 0 ], 'FaceColor', [0.87058824300766 0.921568632125854 0.980392158031464],'FaceAlpha',0.4);
    hold on
    plot3(x_tank(1:n),y_tank(1:n),z_tank(1:n),'-*r')
    grid on
    title('Robot Location in Tank Frame')
    xlabel('x-location [mm]')
    ylabel('y-location [mm]')
    zlabel('Depth[mm]')
    axis(ax(3),[-100 1900 0 1900 -10 915])
    set(ax(3),'zdir','reverse')
    
    set(gcf,'Position',[500 500 1000 500],'OuterPosition',[500 500 1100 600])
    
    F(c)=getframe(fig1);

    c=c+1;
    waitbar(n / length(centroids_fish_o_filt))
    clf
end
close(h)

movie2avi(F,sprintf('Homing_000%d',file_number),'FPS',10)
end

