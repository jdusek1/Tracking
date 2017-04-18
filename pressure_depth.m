% function [depth_p,t_p]=pressure_depth(pname,depth_cam,dt,frames,thresh_low,thresh_high)
%pressure_depth reads the pressure from the fish pressure sensor and
%converts to depth. Plots comparison to depth from tracking.

close all

% 20 to 70
thresh_low=1962;
thresh_high=6867;

% %Hovering
% start_time_o=17;
% end_time_o=359.99;
% start_time_s=20.17;
% end_time_s=362.5;
% frame_rate_divider=3;

% LED_time_o=14.33;
% LED_time_s=17.5;
% 
% thresh_low=3942;

depth_cam=world_outputs(:,6);

% %choose data file 
[pname, ppath] = uigetfile('.txt', 'Pick data file');

% %read in pressure measurments and pressure offset
% fish_data_full=csvread(pname,1);


%read in pressure measurments and pressure offset
fish_data=csvread(pname,1);
% 
% d_track_time_o=(start_time_o-LED_time_o)*10;
% track_end_o=((end_time_o-start_time_o)*10)+d_track_time_o;
% 
% %fish data during track
% fish_data_track_I=find(fish_data_full(:,1)>=d_track_time_o & fish_data_full(:,1)<=track_end_o);
% fish_data=fish_data_full(fish_data_track_I,:);
% 
% %time vector from fish
% t_p=(fish_data(:,1)-min(fish_data(:,1)))/10;

%unfiltered pressure from fish pressure sensor
p=fish_data(:,3);
P_off=csvread(pname,0,0,[0 0 0 0]);

%Pressure offset is calculated when fish on surface and sensor 3.5cm below
%waterline
offset=mean(p(end-3:end));

%Remove offset from pressure measurements
p_zeroed=p-offset;
% p_zeroed=p-P_off;

%convert to depth in mm
% depth_p=(p_zeroed/9.81)+35;
depth_p=(p_zeroed/9.81);

%time vector from fish
t_p=(fish_data(:,1)-min(fish_data(:,1)))/10;

%time for camera tracking
t_track=0:dt:dt*(frames-1);

%target depths
thresh_low_vec=(((P_off-offset)/9.81)+(thresh_low/9.81))*ones(length(t_p),1);
thresh_high_vec=(((P_off-offset)/9.81)+(thresh_high/9.81))*ones(length(t_p),1);

% thresh_low_vec=((thresh_low/9.81)+35)*ones(length(t_p),1);

figure(1)
hax(1)=axes();
h4=plot(t_p,thresh_low_vec,'k--','linewidth',1);
hold on
h5=plot(t_p,thresh_high_vec,'k--','linewidth',1);
h1=plot(t_track,depth_cam,'r','linewidth',1,'parent',hax(1));
h2=plot(t_track,-35+depth_cam,'color',[0.47 0.67 0.19],'linewidth',1);
h3=plot(t_p(1:20:end),depth_p(1:20:end),'bo','markersize',2,'markerfacecolor','b');
% h3=plot(t_p(1:1:end),depth_p(1:1:end),'b','linewidth',1)
legend_str={'Tracking','Track No Offset','Pressure','Target Depth'};
legend1=gridLegend([h1,h2,h3,h4],2,legend_str,'location','north','fontsize',6);
% legend_str={'Depth from Pressure','Target Depth'};
% legend1=gridLegend([h3,h4],2,legend_str,'location','north','fontsize',6);
set(legend1,'position',[0.4 0.82 0.3 0.05])
% set(legend1,'position',[0.35 0.82 0.35 0.05])
legend boxoff
set(hax(1),'YDir','Reverse')
set(hax(1),'Xlim',[0 330],'Ylim',[0 900])
title('Depth from Tracking and Pressure','fontweight','normal')
% title('Depth Maintenance Using Dorsal Fin','fontweight','normal')
xlabel('Time [s]')
ylabel('Depth [mm]')
set(hax(1),'fontsize',8);
set(hax(1),'FontName','Times')
set(hax(1),'FontWeight','normal')
set(hax(1),'box','off')
% 
% hax(2) = copyobj(hax(1),gcf);
% delete( get(hax(2),'Children') ) 
% h6=area(t_p,1*fish_data(:,2),0,'parent',hax(2));
% set(h6,'facecolor',[239, 149, 59]./255);
% set(h6,'facecolor',[224, 214, 204]./255);
% set(hax(2), 'Color','none', 'XTick',[],'Xlabel',[], ...
%     'YAxisLocation','right', 'Box','off','Ylim',[0 15],'YDir','normal','Ytickmode','manual',...
%     'ytick',[0 1],'Yticklabel',{'off','on'},'ticklength',[0 0])
% ax2 = hax(2);
% ax2.YLabel.String = 'Dorsal Fin State';
% ax2.YLabel.FontSize = 8;
% set(hax(2),'box','on')

set(gcf,'PaperUnits','inches','PaperPosition',[0 0.15 3.4 2],'PaperPositionMode','manual','papersize',[3.5 2.2])
print(sprintf('Depth_precision20to70'),'-dpdf')
% print(sprintf('Hovering'),'-dpdf')

% end