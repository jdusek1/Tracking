%Jeff Dusek
%1/24/2017

close all, clear variables, clc

%% 
load Square_sub_0002_10fps

% load Squares_sub_0006_10fps 

x_tank=world_outputs(:,4);
y_tank=world_outputs(:,5);
z_tank=world_outputs(:,6);

% x_tank=outputs_press(:,1);
% y_tank=outputs_press(:,2);
% z_tank=outputs_press(:,3);


%choose data file 
[pname, ppath] = uigetfile('.txt', 'Pick data file');

%read in pressure measurments and pressure offset
fish_data_full=csvread(pname,1);

%When finding time vector for the fish data, the dorsal fin activates ten
%seconds after the double LED blink, this is denoted by the "3" in the
%column 2. When column 2 shows "1" the fish is swimming forward. When
%column 2 shows "2" the fish is turning.

%To get the time to match the video, we need to look at the time of the LED
%blink, and the start time of the tracking.

% % Submerged Square 1
% LED_time_o=14.75;

% % Submerged Square 2
LED_time_o=15.25;
% 
% % % % Submerged Square 006
% LED_time_o=15.15;

d_track_time_o=(start_time_o-LED_time_o)*10;
track_end_o=((end_time_o-start_time_o)*10)+d_track_time_o;

%fish data during track
fish_data_track_I=find(fish_data_full(:,1)>=d_track_time_o & fish_data_full(:,1)<=track_end_o);
fish_data=fish_data_full(fish_data_track_I,:);

%time for camera tracking
t_track=0:dt:dt*(frames-1);

%time vector from fish
t_p=(fish_data(:,1)-min(fish_data(:,1)))/10;

fish_data(:,1)=t_p;

%% Identify points from the tracking where fish is turning based on robot state information

aa=1;
bb=1;
for nn=2:length(fish_data(:,2))
    if fish_data(nn-1,2)==1 && fish_data(nn,2)==2
        start_turn(aa)=t_p(nn);
        start_IMU_I(aa)=nn;
        end_straight(aa)=t_p(nn-1);
        aa=aa+1;
    elseif fish_data(nn-1,2)==2 && fish_data(nn,2)==1
        end_turn(bb)=t_p(nn-1);
        end_IMU_I(bb)=nn-1;
        start_straight(bb)=t_p(nn);
        bb=bb+1;
    end
end
end_straight(bb)=t_p(end);
clear aa
clear bb

cc=1;
dd=1;
straight_I={};
straight_segs={};
turn_segs={};
kk=1;
for mm=1:length(end_straight)-1
    if cc==1
        straight_I{cc}=find(t_track>=0 & t_track<=end_straight(mm));
        straight_segs{cc}=[x_tank(straight_I{cc}),y_tank(straight_I{cc}),z_tank(straight_I{cc})];
        straight_ends_x(:,cc)=[x_tank(min(straight_I{cc}));x_tank(max(straight_I{cc}))];
        straight_ends_y(:,cc)=[y_tank(min(straight_I{cc}));y_tank(max(straight_I{cc}))];
        straight_ends_z(:,cc)=[z_tank(min(straight_I{cc}));z_tank(max(straight_I{cc}))];
        cc=cc+1;
    end
    
    if cc~=1
        straight_I{cc}=find(t_track>=start_straight(mm) & t_track<=end_straight(mm+1));
        turn_I{dd}=find(t_track>=start_turn(mm) & t_track<=end_turn(mm));
        straight_segs{cc}=[x_tank(straight_I{cc}),y_tank(straight_I{cc}),z_tank(straight_I{cc})];
        turn_segs{dd}=[x_tank(turn_I{dd}),y_tank(turn_I{dd}),z_tank(turn_I{dd})];
        straight_ends_x(:,cc)=[x_tank(min(straight_I{cc}));x_tank(max(straight_I{cc}))];
        straight_ends_y(:,cc)=[y_tank(min(straight_I{cc}));y_tank(max(straight_I{cc}))];
        straight_ends_z(:,cc)=[z_tank(min(straight_I{cc}));z_tank(max(straight_I{cc}))];
        cc=cc+1;
        dd=dd+1;
    end
   
end
clear mm
clear nn
clear cc
clear dd

%% Find Segment Lengths

dt_frame=1/10;
bodylength=100;

for nn=1:size(straight_ends_x,2)
    seg_length_ideal(nn)=sqrt((straight_ends_x(1,nn)-straight_ends_x(2,nn))^2+(straight_ends_y(1,nn)-straight_ends_y(2,nn))^2+(straight_ends_z(1,nn)-straight_ends_z(2,nn))^2);
    
    x=straight_segs{nn}(:,1);
    y=straight_segs{nn}(:,2);
    z=straight_segs{nn}(:,3);
    
    [I]=find(isnan(x)==0);
    
    for aa=1:length(I)-1
        ddis1(aa)= sqrt((x(I(aa+1))-x(I(aa)))^2 + (y(I(aa+1))-y(I(aa)))^2 + (z(I(aa+1))-z(I(aa)))^2);
        ddis{:,nn}=ddis1;
        seg_length_track_cum{:,nn}=cumsum(ddis1);
        seg_length_track(nn)=sum(ddis1);
        dis{:,nn}=cumsum(ddis1);
        dt1(aa)=(I(aa+1)-I(aa))*dt_frame;
        dt_vel{:,nn}=dt1;
        vel1(aa)=ddis1/dt1;
        vel1_BL=vel1/bodylength;
        vel{:,nn}=vel1;
        Vel_seg(nn)=mean(vel1);
        vel_BL{:,nn}=vel1_BL;
        Vel_seg_BL(nn)=mean(vel1_BL);
        t{:,nn}=dt_frame*I(1:end-1);
    end
end
mean_vel_seg=mean(Vel_seg);
mean_vel_seg_BL=mean(Vel_seg_BL);
std_vel_seg=std(Vel_seg);

avg_seg_length_ideal=mean(seg_length_ideal(2:end-1));
std_seg_length_ideal=std(seg_length_ideal(2:end-1));

avg_seg_length_track=mean(seg_length_track(2:end-1));
std_seg_length_track=std(seg_length_track(2:end-1));

dev_length=seg_length_track(2:end-1)-seg_length_ideal(2:end-1);

clear nn
clear aa

figure(7)
boxplot(seg_length_track(2:end-1))
hold on
plot(1,seg_length_track(2:end-1),'o')
title('Segment Length from Tracking')
ylabel('millimeters')

figure(8)
boxplot(dev_length)
hold on
plot(1,dev_length,'o')
title('Deviation from Ideal Segment Length')
ylabel('millimeters')

%% Find angles between segments

for nn=1:size(turn_segs,2)
    ax=(straight_ends_x(1,nn)-straight_ends_x(2,nn))
    ay=(straight_ends_y(1,nn)-straight_ends_y(2,nn))
    IMU_values(nn)=sum(fish_data(start_IMU_I(nn):end_IMU_I(nn),3))*(t_p(2)-t_p(1));
    
    bx=(straight_ends_x(2,nn+1)-straight_ends_x(1,nn+1))
    by=(straight_ends_y(2,nn+1)-straight_ends_y(1,nn+1))
    
    turn_angle(nn)=acosd((ax*bx+ay*by)/(sqrt(ax^2+ay^2)*sqrt(bx^2+by^2)))
end

dev=turn_angle-90;



figure(6)
hax(20)=axes();
boxplot(dev)
hold on
plot(1,dev,'o')
title('Turning Angle Deviation from 90^o','fontsize',10','fontweight','normal')
ylabel('Deviation Angle [deg]')
set(hax(20),'fontsize',10);
set(hax(20),'FontName','Times')
set(hax(20),'FontWeight','normal')
set(hax(20),'Ytickmode','manual','Ytick',[-8:2:8])
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 2.9 2],'PaperPositionMode','manual','papersize',[3 2])
print(sprintf('deviation_boxplot_squares_nodorsal'),'-dpdf')

figure(9)
boxplot(IMU_values)
hold on
plot(1,IMU_values,'o')
title('Final Turning IMU Value')

figure(10)
plot(IMU_values,turn_angle,'*')
title('Final Turning IMU Value')

%%

figure(4)
kk=1;
hold on
for mm=1:size(straight_segs,2)
    plot(straight_segs{mm}(:,1),straight_segs{mm}(:,2),'linewidth',2)
    plot(straight_ends_x(:,mm),straight_ends_y(:,mm),'r-*')
    legendInfo{kk} = ['Line = ' num2str(mm)]; 
    kk=kk+1;
    legendInfo{kk} = ['Ideal Line = ' num2str(mm)];
    kk=kk+1;
end

for nn=1:size(turn_segs,2)
    plot(turn_segs{nn}(:,1),turn_segs{nn}(:,2),'--','linewidth',2)
    legendInfo{kk} = ['Turn = ' num2str(nn)]; 
    kk=kk+1;
end
clear mm
clear kk
figure(4)
legend(legendInfo,'location','eastoutside')
axis square
hold off

figure(5)
hax(5)=axes();
axis(hax(5),[0 1800 0 1800 0 915])
set(hax(5),'Xtickmode','manual','Xtick',[0:450:1800],...
'Ytickmode','manual','Ytick',[0:450:1800]);
XL = get(gca, 'XLim');
YL = get(gca, 'YLim');
patch([XL(1), XL(2), XL(2), XL(1)], [YL(1), YL(1), YL(2), YL(2)], [0 0 0 0 ], 'FaceColor', [0.87058824300766 0.921568632125854 0.980392158031464],'FaceAlpha',0.4);
hold on
for mm=1:size(straight_segs,2)
    plot3(straight_segs{mm}(:,1),straight_segs{mm}(:,2),straight_segs{mm}(:,3),'linewidth',2)
    plot3(straight_ends_x(:,mm),straight_ends_y(:,mm),straight_ends_z(:,mm),'r-*')
end

for nn=1:size(turn_segs,2)
    plot3(turn_segs{nn}(:,1),turn_segs{nn}(:,2),turn_segs{nn}(:,3),'--','linewidth',2)
end
figure(5)
grid on
    title('Robot Location in Tank Frame','fontsize',10','fontweight','normal')
    xlabel('x [mm]')
    ylabel('y [mm]')
    zlabel('Depth [mm]')
    set(hax(5),'zdir','reverse')
    set(hax(5),'fontsize',10);
    set(hax(5),'FontName','Times')
    set(hax(5),'FontWeight','normal')
    set(hax(5),'CameraPosition',[-7.0344e+03 -1.1846e+04 -1.6734e+03])
    box on
    clear mm
clear kk
hold off


%% Compute errors

% compute perpendicular distance to the ideal segment line

perp_dist={};
for mm=1:size(straight_segs,2)
    for kk=1:length(straight_segs{mm}(:,1))
        pt=straight_segs{mm}(kk,:);
        v1=[straight_ends_x(1,mm),straight_ends_y(1,mm),straight_ends_z(1,mm)];
        v2=[straight_ends_x(2,mm),straight_ends_y(2,mm),straight_ends_z(2,mm)];
        perp_dist_seg(kk) = point_to_line(pt,v1,v2);
    end
    perp_dist{mm}=perp_dist_seg;
    clear perp_dist_seg
end

clear mm

A=NaN*ones(length(t_track),1);
perp_error=A;
perp_turns=A;
for mm=1:size(straight_I,2)
    perp_error(straight_I{1,mm})=perp_dist{1,mm};
end

I_NaN=isnan(perp_error);
perp_turns(I_NaN)=0;



%% Find depth from pressure

thresh_low=981;
thresh_high=1079;

%unfiltered pressure from fish pressure sensor
p=fish_data(:,4);
P_off=csvread(pname,0,0,[0 0 0 0]);

%Pressure offset is calculated when fish on surface and sensor 3.5cm below
%waterline
% offset=mean(fish_data_full(end-20:end,4));
offset=P_off;

%Remove offset from pressure measurements
p_zeroed=p-offset;

%convert to depth in mm
depth_p=p_zeroed/9.81;

%target depths
thresh_low_vec=(((P_off-offset)/9.81)+(thresh_low/9.81))*ones(length(t_p),1);
thresh_high_vec=(((P_off-offset)/9.81)+(thresh_high/9.81))*ones(length(t_p),1);

%% Depth plot

figure(1)
hax(1)=axes();
h4=plot(t_p,thresh_low_vec,'k--','linewidth',1);
hold on
h5=plot(t_p,thresh_high_vec,'k--','linewidth',1);
h1=plot(t_track,z_tank,'r','linewidth',1,'parent',hax(1));
h2=plot(t_track,-90+z_tank,'color',[0.47 0.67 0.19],'linewidth',2);
h3=plot(t_p,depth_p,'b','linewidth',1);
legend_str={'Tracking','Track No Offset','Pressure','Thresholds'};
legend1=gridLegend([h1,h2,h3,h4],2,legend_str,'location','north','fontsize',6);
set(legend1,'position',[0.4 0.82 0.3 0.05])
legend boxoff
set(hax(1),'YDir','Reverse')
% set(hax(1),'Xlim',[0 180],'Ylim',[ 50 250])
title('Depth from Tracking and Pressure','fontweight','normal')
xlabel('Time [s]')
ylabel('Depth [mm]')
set(hax(1),'fontsize',8);
set(hax(1),'FontName','Times')
set(hax(1),'FontWeight','normal')
set(hax(1),'box','off')

hax(2) = copyobj(hax(1),gcf);
delete( get(hax(2),'Children') ) 
h6=area(t_p,1*fish_data(:,2),0,'parent',hax(2));
set(hax(2), 'Color','none', 'XTick',[],'Xlabel',[], ...
    'YAxisLocation','right', 'Box','off','Ylim',[1 8],'YDir','normal','Ytickmode','manual',...
    'ytick',[1 2],'Yticklabel',{'Straight','Turn'},'ticklength',[0 0])
ax2 = hax(2);
ax2.YLabel.String = 'Fish Behavior';
ax2.YLabel.FontSize = 8;
set(hax(2),'box','on')

set(gcf,'PaperUnits','inches','PaperPosition',[0 0 3.4 2],'PaperPositionMode','manual','papersize',[3.5 2])
print(sprintf('Squares_Depth'),'-dpdf')

%% Position/turning plot

figure(2)
hax(1)=axes();
h1=plot(t_track,x_tank,'r','linewidth',1,'parent',hax(1));
hold on
h2=plot(t_track,y_tank,'color',[0.47 0.67 0.19],'linewidth',2);
legend_str={'x position','y position'};
legend1=gridLegend([h1,h2],2,legend_str,'location','north','fontsize',6);
set(legend1,'position',[0.4 0.82 0.3 0.05])
legend boxoff
set(hax(1),'YDir','Reverse')
set(hax(1),'Xlim',[0 180],'Ylim',[ 300 1600])
title('Depth from Tracking and Pressure','fontweight','normal')
xlabel('Time [s]')
ylabel('Depth [mm]')
set(hax(1),'fontsize',8);
set(hax(1),'FontName','Times')
set(hax(1),'FontWeight','normal')
set(hax(1),'box','off')

hax(2) = copyobj(hax(1),gcf);
delete( get(hax(2),'Children') ) 
h6=area(t_p,1*fish_data(:,2),0,'parent',hax(2));
set(hax(2), 'Color','none', 'XTick',[],'Xlabel',[], ...
    'YAxisLocation','right', 'Box','off','Ylim',[1 8],'YDir','normal','Ytickmode','manual',...
    'ytick',[1 2],'Yticklabel',{'Straight','Turn'},'ticklength',[0 0])
ax2 = hax(2);
ax2.YLabel.String = 'Fish Behavior';
ax2.YLabel.FontSize = 8;
set(hax(2),'box','on')

set(gcf,'PaperUnits','inches','PaperPosition',[0 0 3.4 2],'PaperPositionMode','manual','papersize',[3.5 2])
print(sprintf('Squares_Turns'),'-dpdf')


%% Make plot with 2D and 3D trajectory plus error

map= [7, 198, 134
    51, 125, 184
    209, 26, 41
    206, 13, 157
    240, 101, 30
    142, 187, 225] ./ 255;

fig3=figure(3);
set(gcf,'units','inches','OuterPosition',[20 10 7.00 4.5])
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 7.00 4.5],'PaperPositionMode','manual','papersize',[7.00 4.5])

    positionVector1=[0,0.2,3.5,3.5];
    hax(1)=axes('Units','inches','OuterPosition',positionVector1);
    kk=1;
    hold on
    B=4;
    for mm=1:size(straight_segs,2)
        h3(mm)=plot(straight_segs{mm}(:,1),straight_segs{mm}(:,2),'linewidth',1,'color',map(kk,:));
        plot(straight_ends_x(:,mm),straight_ends_y(:,mm),'r*','MarkerSize',1)
        
        legend_str{kk} = ['Square ' num2str(kk)];
       
        if mm==B;
            kk=kk+1;
            B=B+4;
        end
    end
    clear mm
    clear B
    clear kk
   
    B=4;
    kk=1;
    for nn=1:size(turn_segs,2)
        plot(turn_segs{nn}(:,1),turn_segs{nn}(:,2),'linewidth',1,'color',map(kk,:))
        
        if nn==B;
            kk=kk+1;
            B=B+4;
        end
    end
    set(hax(1),'Xlim',[400 1800],'Ylim',[400 1800],'PlotBoxAspectRatio',[1 1 1],'Xgrid','on','Ygrid','on',...
        'Xtickmode','manual','Xtick',[400:400:1800],'Xminortick','on','xminorgrid','on',...
    'Ytickmode','manual','Ytick',[400:400:1800],'Yminortick','on','Yminorgrid','on');
    hax(1).Title.String = 'Overhead Trajectory';
    hax(1).Title.FontWeight = 'normal';
    hax(1).Title.FontSize = 10;
    ylabel('Y-Position [mm]')
    xlabel('X-Position [mm]')
    set(hax(1),'fontsize',10);
    set(hax(1),'FontName','Times')
    set(hax(1),'FontWeight','normal')
    box on
%     legend31=gridLegend([h3(1),h3(5),h3(9),h3(13),h3(17),h3(21)],2,legend_str,'location','north','fontsize',6);
%     legend31=legend([h3(1),h3(5),h3(9),h3(13),h3(17),h3(21)],legend_str,'position',[0.199404761904761 0.574701195219123 0.135912698412699 0.193227091633466],'fontsize',6);
legend31=legend([h3(1),h3(5),h3(9),h3(13),h3(17)],legend_str,'position',[0.2 0.1 0.587301587301585 0.0458167330677291],'fontsize',10,'orientation','horizontal');
    legend boxoff
    clear mm
    clear B
    clear kk
    clear nn
    
    positionVector2=[2.75,.5,4.5,2.5];
    hax(2)=axes('Units','inches','OuterPosition',positionVector2);
    axis(hax(2),[0 1800 0 1800 0 915])
    set(hax(2),'Xtickmode','manual','Xtick',[0:450:1800],...
    'Ytickmode','manual','Ytick',[0:450:1800]);
    XL = get(gca, 'XLim');
    YL = get(gca, 'YLim');
    patch([XL(1), XL(2), XL(2), XL(1)], [YL(1), YL(1), YL(2), YL(2)], [0 0 0 0 ], 'FaceColor', [0.87058824300766 0.921568632125854 0.980392158031464],'FaceAlpha',0.4);
    kk=1;
    hold on
    B=4;
    for mm=1:size(straight_segs,2)
        plot3(straight_segs{mm}(:,1),straight_segs{mm}(:,2),straight_segs{mm}(:,3),'linewidth',1,'color',map(kk,:));
        plot3(straight_ends_x(:,mm),straight_ends_y(:,mm),straight_ends_z(:,mm),'r*','MarkerSize',1)
       
        if mm==B;
            kk=kk+1;
            B=B+4;
        end
    end
    clear mm
    clear B
    clear kk
   
    B=4;
    kk=1;
    for nn=1:size(turn_segs,2)
        plot3(turn_segs{nn}(:,1),turn_segs{nn}(:,2),turn_segs{nn}(:,3),'linewidth',1,'color',map(kk,:))
        
        if nn==B;
            kk=kk+1;
            B=B+4;
        end
    end
    grid on
    title('Robot Location in Tank Frame','fontsize',10','fontweight','normal')
    xlabel('x [mm]')
    ylabel('y [mm]')
    zlabel('Depth [mm]')
    set(hax(2),'zdir','reverse')
    set(hax(2),'fontsize',10);
    set(hax(2),'FontName','Times')
    set(hax(2),'FontWeight','normal')
    set(hax(2),'CameraPosition',[-7.0344e+03 -1.1846e+04 -1.6734e+03])
    box on
% 
%     positionVector3=[0.1,0.05,7,1.15];
%     hax(3)=axes('Units','inches','OuterPosition',positionVector3);
%     plot(t_track,perp_error,'r','linewidth',1);
%     hold on
%     plot(t_track,perp_turns,'b','linewidth',1);
%     title('Perpendicular Distance from Ideal Track','fontsize',10','fontweight','normal')
%     xlabel('Time [s]')
%     ylabel('Error [mm]')
%     set(hax(3),'fontsize',10);
%     set(hax(3),'FontName','Times')
%     set(hax(3),'FontWeight','normal','Ytickmode','manual','Ytick',[0,10])
%     axis(hax(3),[0 250 -5 15])
%     legend3=legend('Straight Segment','Turning');
%     set(legend3,...
%     'Position',[0.584824349647265 0.0215838500303422 0.340277777777777 0.0597609561752987],...
%     'Orientation','horizontal','fontsize',6);
%     grid on
%     legend boxoff

    
    print(sprintf('Squares_Trajectories'),'-dpdf')
    
    
    
    
    %% plot with 2D and 3D trajectory no NaN

map= [7, 198, 134
    51, 125, 184
    209, 26, 41
    206, 13, 157
    240, 101, 30
    142, 187, 225] ./ 255;

fig1=figure(111);
set(gcf,'units','inches','OuterPosition',[20 10 7.00 3.7])
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 7.00 3.7],'PaperPositionMode','manual','papersize',[7.00 3.7])

    positionVector1=[-0.2,-0.4,3.9,3.9];
    hax(1)=axes('Units','inches','OuterPosition',positionVector1);
    kk=1;
    hold on
    B=4;
    for mm=1:size(straight_segs,2)
        Ix=find(isnan(straight_segs{mm}(:,1))==1);
        straight_segs_noNaN{mm}(:,:)=straight_segs{mm}(:,:);
        straight_segs_noNaN{mm}(Ix,:)=[];
        h3(mm)=plot(straight_segs_noNaN{mm}(:,1),straight_segs_noNaN{mm}(:,2),'linewidth',1,'color',map(kk,:));
        plot(straight_ends_x(:,mm),straight_ends_y(:,mm),'r*','MarkerSize',1)
        
        legend_str{kk} = ['Square ' num2str(kk)];
       
        if mm==B;
            kk=kk+1;
            B=B+4;
        end
    end
    clear mm
    clear B
    clear kk
   
    B=4;
    kk=1;
    for nn=1:size(turn_segs,2)
        Ix2=find(isnan(turn_segs{nn}(:,1))==1);
        turn_segs_noNaN{nn}(:,:)=turn_segs{nn}(:,:);
        turn_segs_noNaN{nn}(Ix2,:)=[];
        plot(turn_segs_noNaN{nn}(:,1),turn_segs_noNaN{nn}(:,2),'linewidth',1,'color',map(kk,:))
        
        if nn==B;
            kk=kk+1;
            B=B+4;
        end
    end
    set(hax(1),'Xlim',[200 1100],'Ylim',[450 1350],'PlotBoxAspectRatio',[1 1 1],'Xgrid','on','Ygrid','on',...
        'Xtickmode','manual','Xtick',[200:300:1100],'Xminortick','on','xminorgrid','on',...
    'Ytickmode','manual','Ytick',[450:300:1350],'Yminortick','on','Yminorgrid','on');
    hax(1).Title.String = 'Overhead Trajectory';
    hax(1).Title.FontWeight = 'normal';
    hax(1).Title.FontSize = 10;
    ylabel('Y-Position [mm]')
    xlabel('X-Position [mm]')
    set(hax(1),'fontsize',10);
    set(hax(1),'FontName','Times')
    set(hax(1),'FontWeight','normal')
    box on
%     legend31=gridLegend([h3(1),h3(5),h3(9),h3(13),h3(17),h3(21)],2,legend_str,'location','north','fontsize',6);
%     legend31=legend([h3(1),h3(5),h3(9),h3(13),h3(17),h3(21)],legend_str,'position',[0.199404761904761 0.574701195219123 0.135912698412699 0.193227091633466],'fontsize',6);
legend31=legend([h3(1),h3(5),h3(9),h3(13),h3(17),h3(21)],legend_str,'position',[0.2 0.05 0.587301587301585 0.0458167330677291],'fontsize',10,'orientation','horizontal');
% legend31=legend([h3(1),h3(5),h3(9),h3(13),h3(17)],legend_str,'position',[0.2 0.05 0.587301587301585 0.0458167330677291],'fontsize',10,'orientation','horizontal');
    legend boxoff
    clear mm
    clear B
    clear kk
    clear nn
    
    positionVector2=[2.75,.2,4.5,2.5];
    hax(2)=axes('Units','inches','OuterPosition',positionVector2);
    axis(hax(2),[0 1800 0 1800 0 915])
    set(hax(2),'Xtickmode','manual','Xtick',[0:450:1800],...
    'Ytickmode','manual','Ytick',[0:450:1800]);
    XL = get(gca, 'XLim');
    YL = get(gca, 'YLim');
    patch([XL(1), XL(2), XL(2), XL(1)], [YL(1), YL(1), YL(2), YL(2)], [0 0 0 0 ], 'FaceColor', [0.87058824300766 0.921568632125854 0.980392158031464],'FaceAlpha',0.4);
    kk=1;
    hold on
    B=4;
    for mm=1:size(straight_segs,2)
        plot3(straight_segs_noNaN{mm}(:,1),straight_segs_noNaN{mm}(:,2),straight_segs_noNaN{mm}(:,3),'linewidth',1,'color',map(kk,:));
        plot3(straight_ends_x(:,mm),straight_ends_y(:,mm),straight_ends_z(:,mm),'r*','MarkerSize',1)
       
        if mm==B;
            kk=kk+1;
            B=B+4;
        end
    end
    clear mm
    clear B
    clear kk
   
    B=4;
    kk=1;
    for nn=1:size(turn_segs,2)
        plot3(turn_segs_noNaN{nn}(:,1),turn_segs_noNaN{nn}(:,2),turn_segs_noNaN{nn}(:,3),'linewidth',1,'color',map(kk,:))
        
        if nn==B;
            kk=kk+1;
            B=B+4;
        end
    end
    grid on
    title('Robot Location in Tank Frame','fontsize',10','fontweight','normal')
    xlabel('x [mm]')
    ylabel('y [mm]')
    zlabel('Depth [mm]')
    set(hax(2),'zdir','reverse')
    set(hax(2),'fontsize',10);
    set(hax(2),'FontName','Times')
    set(hax(2),'FontWeight','normal')
    set(hax(2),'CameraPosition',[-1.1354   -0.7776   -0.1673].*1.0e+04)
    box on

    
    print(sprintf('Squares_Trajectories_noNaN'),'-dpdf')
    
    %% Find Turning Radius
    
    clear nn
    for nn=1:size(turn_segs_noNaN,2)
        circ_out(nn,:)=CircleFitByPratt(turn_segs_noNaN{nn}(:,1:2));
    end
    
    circ_radius=circ_out(:,3);
    
   mean_radius=mean(circ_radius);
   std_radius=std(circ_radius);
   median_radius=median(circ_radius);
   
    figure(222)
    boxplot(circ_radius)
    hold on
    plot(1,circ_radius,'o')
    title('Turning Radius')
    ylabel('millimeters')
    
    figure(333)
    for nn=1:size(turn_segs_noNaN,2)
        hold on
        plot(turn_segs_noNaN{nn}(:,1),turn_segs_noNaN{nn}(:,2),'*')
        h=circle(circ_out(nn,1),circ_out(nn,2),circ_out(nn,3));
    end

    
        
    