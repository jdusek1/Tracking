%Background removal of videos from new testing tank

%Jeff Dusek
%11/7/16
clc, clear all, close all

%select first an image file to get the path
[imname, impath] = uigetfile('.tif', 'Pick an image file');

%grab the position in pixels of four points to define the tank
%area to make the detection of the fish more robust
T=imread([impath,imname]);
T=T(:,:,1:3);
figure(1)
imagesc(T);
I=getframe(gcf);
for n=1:4
H=impoint;
pos(n,:)=wait(H);
end
clf

%%

%Read images within the designated area and take an average of the RGB
%values that represent the image background

startn_avg=600;
stp_avg=5;
endn_avg=1000;

fidi=fopen([impath,num2str(1,'%03g'),'.tif']);

c=1;
for n=startn_avg:stp_avg:endn_avg
    if fidi==-1;
        disp('file not found')
        break
     end
clf
fclose(fidi);
    
%read image file
T=imread([impath,num2str(n,'%03g'),'.tif'],'PixelRegion',{[round(min(pos(:,2))),round(max(pos(:,2)))],...
    [round(min(pos(:,1))),round(max(pos(:,1)))]});
Tr(:,:,c)=double(T(:,:,1));
Tg(:,:,c)=double(T(:,:,2));
Tb(:,:,c)=double(T(:,:,3));

c=c+1;
fidi=fopen([impath,num2str(n+stp_avg,'%03g'),'.tif']);
end

R_avg=mean(Tr,3);
G_avg=mean(Tg,3);
B_avg=mean(Tb,3);

RGB_avg(:,:,1)=R_avg;
RGB_avg(:,:,2)=G_avg;
RGB_avg(:,:,3)=B_avg;

%%

startn_track=600;
stp_track=1;
endn_track=605;

fidi=fopen([impath,num2str(1,'%03g'),'.tif']);

cc=1;
for nn=startn_track:stp_track:endn_track
    if fidi==-1;
        disp('file not found')
        break
     end
clf
fclose(fidi);
    
%read image file
I=imread([impath,num2str(nn,'%03g'),'.tif'],'PixelRegion',{[round(min(pos(:,2))),round(max(pos(:,2)))],...
    [round(min(pos(:,1))),round(max(pos(:,1)))]});

I_removed=uint8(-double(I)+(RGB_avg));

I_noback{cc}=I_removed;

cc=cc+1;
fidi=fopen([impath,num2str(nn+stp_track,'%03g'),'.tif']);
end
