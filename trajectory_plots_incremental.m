%Jeff Dusek
%1/24/2017

close all, clear variables, clc


%% 
load Square_sub_0003_10fps_crop_noNaN

x_tank=world_outputs(:,4);
y_tank=world_outputs(:,5);
z_tank=world_outputs(:,6);

% % load Square_sub_0004_10fps_noNaN_pressure

% load Squares_sub_0006_10fps
% 
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

% % Submerged Square 3- incremental
LED_time_o=14;

% % % Submerged Square 4- incremental
% LED_time_o=20;

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
cc=1;
dd=1;
ee=1;
ff=1;
for nn=2:length(fish_data(:,2))
    if fish_data(nn-1,2)==1 && fish_data(nn,2)~=1
        end_straight(aa)=t_p(nn-1);
        aa=aa+1;
    elseif fish_data(nn-1,2)==2 && fish_data(nn,2)~=2
        end_turn(bb)=t_p(nn-1);
        bb=bb+1;
    elseif fish_data(nn-1,2)==3 && fish_data(nn,2)~=3
        end_rise(cc)=t_p(nn-1);
        cc=cc+1;
    end
    
    if fish_data(nn,2)==1 && fish_data(nn-1,2)~=1
        start_straight(dd)=t_p(nn);
        dd=dd+1;
    elseif fish_data(nn,2)==2 && fish_data(nn-1,2)~=2
        start_turn(ee)=t_p(nn);
        ee=ee+1;
    elseif fish_data(nn,2)==3 && fish_data(nn-1,2)~=3
        start_rise(ff)=t_p(nn);
        ff=ff+1;
    end
end
end_straight(bb)=t_p(end);
clear aa bb cc dd ee ff

cc=1;
dd=1;
straight_I={};
turn_I={};
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


ee=1;
rise_I={};
rise_segs={};
for mm=1:length(end_rise)
        rise_I{ee}=find(t_track>=start_rise(mm) & t_track<=end_rise(mm));
        rise_segs{ee}=[x_tank(rise_I{ee}),y_tank(rise_I{ee}),z_tank(rise_I{ee})];
        ee=ee+1;
end
clear mm
clear ee

%% Find Segment Lengths

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
    end
end
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
    ax=(straight_ends_x(1,nn)-straight_ends_x(2,nn));
    ay=(straight_ends_y(1,nn)-straight_ends_y(2,nn));
    
    bx=(straight_ends_x(2,nn+1)-straight_ends_x(1,nn+1));
    by=(straight_ends_y(2,nn+1)-straight_ends_y(1,nn+1));
    
    turn_angle(nn)=acosd((ax*bx+ay*by)/(sqrt(ax^2+ay^2)*sqrt(bx^2+by^2)));
end

dev=turn_angle-90;

figure(6)
boxplot(dev)
hold on
plot(1,dev,'o')
title('Turning Angle Deviation from 90^o')

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

for ll=1:size(rise_segs,2)
    plot3(rise_segs{ll}(:,1),rise_segs{ll}(:,2),rise_segs{ll}(:,3),'r','linewidth',2)
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
    clear mm nn ll 
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

% % Thresholds for 0003
% thresh_low=[660,460,260,60];
% thresh_high=[700,500,300,100];

% Thresholds for 0004
thresh_low=[680,510,340,170];
thresh_high=[720,550,380,210];


%unfiltered pressure from fish pressure sensor
p=fish_data(:,4);
P_off=csvread(pname,0,0,[0 0 0 0]);

%Pressure offset is calculated when fish on surface and sensor 3.5cm below
%waterline
% offset=mean(fish_data_full(end-20:end,4));
offset=P_off;

%Remove offset from pressure measurements
p_zeroed=p-offset;

% %convert to depth in mm for 0003
% depth_p=(p_zeroed/9.81)+35;

%convert to depth in mm for 0004
depth_p=(p_zeroed/9.81);

%target depths
thresh_low_vec=repmat(thresh_low,length(t_p),1);
thresh_high_vec=repmat(thresh_high,length(t_p),1);

%% Depth plot

figure(1)
hax(1)=axes();
hold on
for vv=1:size(thresh_low_vec,2)
    h4=fill([t_p',fliplr(t_p')],[thresh_high_vec(:,vv)',thresh_low_vec(:,vv)'],[216, 215, 208]./256);
end
h3=plot(t_p,depth_p,'b','linewidth',2);
legend_str={'Depth from Pressure','Square Depth Thresholds'};
legend1=legend([h3,h4],legend_str,'location','north','fontsize',6);
set(legend1,'position',[0.5 0.25 0.3 0.05])
legend boxoff
set(hax(1),'YDir','Reverse')
set(hax(1),'Xlim',[0 202],'Ylim',[0 900])
title('Submerged Squares at Incremental Depth','fontweight','normal')
xlabel('Time [s]')
ylabel('Depth [mm]')
set(hax(1),'fontsize',8);
set(hax(1),'FontName','Times')
set(hax(1),'FontWeight','normal')
set(hax(1),'box','on')

set(gcf,'PaperUnits','inches','PaperPosition',[0 0.025 3.4 2],'PaperPositionMode','manual','papersize',[3.5 2])
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
set(gcf,'units','inches','OuterPosition',[20 10 7.00 3.7])
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 7.00 3.7],'PaperPositionMode','manual','papersize',[7.00 3.7])

    positionVector1=[-0.2,-0.4,3.9,3.9];
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
    
for ll=1:size(rise_segs,2)
    h4=plot(rise_segs{ll}(:,1),rise_segs{ll}(:,2),'r','linewidth',1);
end
%     set(hax(1),'Xlim',[200 1300],'Ylim',[650 1750],'PlotBoxAspectRatio',[1 1 1],'Xgrid','off','Ygrid','off',...
%         'Xtickmode','manual','Xtick',[200:200:1300],'Xminortick','on','xminorgrid','off',...
%     'Ytickmode','manual','Ytick',[650:200:1750],'Yminortick','on','Yminorgrid','off');
    set(hax(1),'Xlim',[200 1300],'Ylim',[450 1550],'PlotBoxAspectRatio',[1 1 1],'Xgrid','on','Ygrid','on',...
        'Xtickmode','manual','Xtick',[200:200:1300],'Xminortick','on','xminorgrid','on',...
    'Ytickmode','manual','Ytick',[450:200:1550],'Yminortick','on','Yminorgrid','on');
    hax(1).Title.String = 'Overhead Trajectory';
    hax(1).Title.FontWeight = 'normal';
    hax(1).Title.FontSize = 10;
    ylabel('Y-Position [mm]')
    xlabel('X-Position [mm]')
    set(hax(1),'fontsize',10);
    set(hax(1),'FontName','Times')
    set(hax(1),'FontWeight','normal')
    legend31=legend([h3(1),h3(5),h3(9),h3(13),h4],[legend_str,'Rising'],'position',[0.2 0.05 0.587301587301585 0.0458167330677291],'fontsize',10,'orientation','horizontal');
%     legend31=legend([h3(1),h3(5),h3(9),h3(13),h3(17),h4],[legend_str,'Rising'],'position',[0.2 0.1 0.587301587301585 0.0458167330677291],'fontsize',10,'orientation','horizontal');
    legend boxoff
    box on
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
    
for ll=1:size(rise_segs,2)
    plot3(rise_segs{ll}(:,1),rise_segs{ll}(:,2),rise_segs{ll}(:,3),'r','linewidth',1);
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

    
    print(sprintf('Squares_Trajectories_Incremental'),'-dpdf')