close all
clc
clear
WorkPath='G:\ÍÐÅÌ¼ì²â\ÍÐÅÌ¼ì²âÕûÀí\tray_decete_one\result\';
%% Í¼£¬ÍÐÅÌÒÉËÆµã
voxelpoint=load([WorkPath,'bag_voxel.txt']);
voxel=voxelpoint;
TrayEdgeResponseGrid = load([WorkPath,'stray_voxel.txt']);
fig1=figure; hold on,axis equal
    plot3(voxel(:,1),voxel(:,2),voxel(:,3),'c.'),
    plot3(TrayEdgeResponseGrid(:,1),TrayEdgeResponseGrid(:,2),TrayEdgeResponseGrid(:,3),'k*')
%     for i=1:length(TrayEdgeResponseGrid)
%         quiver3(TrayEdgeResponseGrid(i,1),TrayEdgeResponseGrid(i,2),TrayEdgeResponseGrid(i,3)...
%             ,DragImg(i,1),DragImg(i,2),DragImg(i,3),'r','filled','LineWidth',1);
%     end
    h = [ 550 300 400 300];
    set(gcf,'Position',h)
xlabel('X/cm');
ylabel('Y/cm');
annotation1 = annotation(fig1,'arrow',[0.139 0.139],[0.90 0.98]);
annotation2 = annotation(fig1,'arrow',[0.88 0.96],[0.125 0.125]);
hold off;
%% part
 a =1;
for i=1:length(TrayEdgeResponseGrid)
if(TrayEdgeResponseGrid(i,1)>7&&TrayEdgeResponseGrid(i,1)<13)
leftpoint(a,:) = TrayEdgeResponseGrid(i,:);
a=a+1;
end
end
 a =1;
for i=1:length(TrayEdgeResponseGrid)
if(TrayEdgeResponseGrid(i,1)>50&&TrayEdgeResponseGrid(i,1)<56)
rightpoint(a,:) = TrayEdgeResponseGrid(i,:);
a=a+1;
end
end
 a =1;
for i=1:length(TrayEdgeResponseGrid)
if(TrayEdgeResponseGrid(i,2)>7&&TrayEdgeResponseGrid(i,2)<14)
downpoint(a,:) = TrayEdgeResponseGrid(i,:);
a=a+1;
end
end
 a =1;
for i=1:length(TrayEdgeResponseGrid)
if(TrayEdgeResponseGrid(i,2)>83&&TrayEdgeResponseGrid(i,2)<89)
    uppoint(a,:) = TrayEdgeResponseGrid(i,:);
a=a+1;
end
end
%% left
 iter =10000;
d1 = zeros(iter,length(leftpoint));
num = zeros(iter,1);
idx = randperm(length(leftpoint),2); 
sample = leftpoint(idx,:);
 point1 = sample;
 for i=1:iter
for j = 1:length(leftpoint)
    p = leftpoint(j,:);
     ll = [sample(1,1)-sample(2,1),sample(1,2)-sample(2,2),sample(1,3)-sample(2,3)];
     pl = [p(1)-sample(2,1),p(2)-sample(2,2),p(3)-sample(2,3)];
    tem = cross(pl,ll);
    d1(i,j)= norm(tem)/norm(ll);
 if(d1(i,j)) <7
     num(i) = num(i)+1;
 end
end
if(i >1&&num(i)>num(i-1))
    point1 = sample;
end
idx = randperm(length(leftpoint),2); 
sample = leftpoint(idx,:); 
 end
figure; hold on,axis equal
    plot3(voxel(:,1),voxel(:,2),voxel(:,3),'c.');
    plot3(TrayEdgeResponseGrid(:,1),TrayEdgeResponseGrid(:,2),TrayEdgeResponseGrid(:,3),'k*');
 line(point1(:,1),point1(:,2),point1(:,3),'color','b');
%% right 
 iter =10000;
d2 = zeros(iter,length(rightpoint));
num = zeros(iter,1);
idx = randperm(length(rightpoint),2); 
sample = rightpoint(idx,:);
 point2 = sample;
 for i=1:iter
for j = 1:length(rightpoint)
    p = rightpoint(j,:);
     ll = [sample(1,1)-sample(2,1),sample(1,2)-sample(2,2),sample(1,3)-sample(2,3)];
     pl = [p(1)-sample(2,1),p(2)-sample(2,2),p(3)-sample(2,3)];
    tem = cross(pl,ll);
    d2(i,j)= norm(tem)/norm(ll);
 if(d2(i,j)) <7
     num(i) = num(i)+1;
 end
end
if(i >1&&num(i)>num(i-1))
    point2 = sample;
end
idx = randperm(length(rightpoint),2); 
sample = rightpoint(idx,:); 
 end
figure; hold on,axis equal
    plot3(voxel(:,1),voxel(:,2),voxel(:,3),'c.');
    plot3(TrayEdgeResponseGrid(:,1),TrayEdgeResponseGrid(:,2),TrayEdgeResponseGrid(:,3),'k*');
 line(point2(:,1),point2(:,2),point2(:,3),'color','b');
 %% up
 iter =10000;
d3 = zeros(iter,length(uppoint));
num = zeros(iter,1);
idx = randperm(length(uppoint),2); 
sample = uppoint(idx,:);
 point3 = sample;
 for i=1:iter
for j = 1:length(uppoint)
    p = uppoint(j,:);
     ll = [sample(1,1)-sample(2,1),sample(1,2)-sample(2,2),sample(1,3)-sample(2,3)];
     pl = [p(1)-sample(2,1),p(2)-sample(2,2),p(3)-sample(2,3)];
    tem = cross(pl,ll);
    d3(i,j)= norm(tem)/norm(ll);
 if(d3(i,j)) <6
     num(i) = num(i)+1;
 end
end
if(i >1&&num(i)>num(i-1))
    point3 = sample;
end
idx = randperm(length(uppoint),2); 
sample = uppoint(idx,:); 
 end
figure; hold on,axis equal
    plot3(voxel(:,1),voxel(:,2),voxel(:,3),'c.');
    plot3(TrayEdgeResponseGrid(:,1),TrayEdgeResponseGrid(:,2),TrayEdgeResponseGrid(:,3),'k*');
 line(point3(:,1),point3(:,2),point3(:,3),'color','b');
 %% down
 iter =10000;
d4 = zeros(iter,length(downpoint));
num = zeros(iter,1);
idx = randperm(length(downpoint),2); 
sample = downpoint(idx,:);
 point4 = sample;
 for i=1:iter
for j = 1:length(downpoint)
    p = downpoint(j,:);
     ll = [sample(1,1)-sample(2,1),sample(1,2)-sample(2,2),sample(1,3)-sample(2,3)];
     pl = [p(1)-sample(2,1),p(2)-sample(2,2),p(3)-sample(2,3)];
    tem = cross(pl,ll);
    d4(i,j)= norm(tem)/norm(ll);
 if(d4(i,j)) <8
     num(i) = num(i)+1;
 end
end
if(i >1&&num(i)>num(i-1))
    point4 = sample;
end
idx = randperm(length(downpoint),2); 
sample = downpoint(idx,:); 
 end
figure; hold on,axis equal
    plot3(voxel(:,1),voxel(:,2),voxel(:,3),'c.');
    plot3(TrayEdgeResponseGrid(:,1),TrayEdgeResponseGrid(:,2),TrayEdgeResponseGrid(:,3),'k*');
 line(point4(:,1),point4(:,2),point4(:,3),'color','b');
 %% figure
  k1 = polyfit(point1(:,2),point1(:,1),1);
 y1 = 12:83;
 x1 = polyval(k1,y1);
 
 k2 = polyfit(point2(:,2),point2(:,1),1);
 y2 = 12:83;
 x2 = polyval(k2,y2);
 
  k3 = polyfit(point3(:,1),point3(:,2),1);
 x3 = 12:55;
 y3 = polyval(k3,x3);
 
  k4 = polyfit(point4(:,1),point4(:,2),1);
 x4 = 12:55;
 y4 = polyval(k4,x4);
figure; hold on,axis equal
    plot3(voxel(:,1),voxel(:,2),voxel(:,3),'c.');
    plot3(TrayEdgeResponseGrid(:,1),TrayEdgeResponseGrid(:,2),TrayEdgeResponseGrid(:,3),'k*');
    plot(x1,y1,'linewidth',3,'color','r');
    plot(x2,y2,'linewidth',3,'color','r');
    plot(x3,y3,'linewidth',3,'color','r');
    plot(x4,y4,'linewidth',3,'color','r');