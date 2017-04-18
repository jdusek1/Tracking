% Look at velocity

% Jeff Dusek
% 12/1/16

clc, clear variables, close all

start_time=0;
end_time=45;


% load z_tank_0002;
% load z_cam_0002;
% load y_tank_0002;
% load y_cam_0002;
% load x_tank_0002;
% load x_cam_0002;
load centroids_fish_o_filt_0002;
load centroids_fish_s_filt_0002;
load centroids_fish_o_full_0002;
load centroids_fish_s_0002;
% load pos_o_vid;

load Vname_o;
load Vpath_o;
load Vname_s;
load Vpath_s;

vidObj_o=VideoReader(Vname_o);
vidObj_s=VideoReader(Vname_s);

%Start frame for overhead camera
start_frame_o=start_time*vidObj_o.framerate;
end_frame_o=end_time*vidObj_o.framerate;

%Start frame for side camera
start_frame_s=start_time*vidObj_s.framerate;
end_frame_s=end_time*vidObj_s.framerate;

%Number of frames to use
frames=(end_time-start_time)*vidObj_o.framerate;

cc_o=linspace(start_frame_o,end_frame_o,frames); 
cc_s=linspace(start_frame_s,end_frame_s,frames);


c=1;
for n=1:frames
    
    if n>1
        v_pix(n)=sqrt((centroids_fish_s(n,1)-centroids_fish_s(n-1,1))^2+(centroids_fish_s(n,2)-centroids_fish_s(n-1,2))^2)/((cc_o(n)/vidObj_o.framerate)-(cc_o(n-1)/vidObj_o.framerate));
        v_pixx(n)=(centroids_fish_s(n,1)-centroids_fish_s(n-1,1))/((cc_o(n)/vidObj_o.framerate)-(cc_o(n-1)/vidObj_o.framerate));
        v_pixy(n)=(centroids_fish_s(n,2)-centroids_fish_s(n-1,2))/((cc_o(n)/vidObj_o.framerate)-(cc_o(n-1)/vidObj_o.framerate));
    end
end



