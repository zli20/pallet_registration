 k1 = polyfit(leftpoint1(:,2),leftpoint1(:,1),1);
 y1 = 12:83;
 x1 = polyval(k1,y1);
 
 k2 = polyfit(rightpoint1(:,2),rightpoint1(:,1),1);
 y2 = 12:83;
 x2 = polyval(k2,y2);
 
  k3 = polyfit(uppoint1(:,1),uppoint1(:,2),1);
 x3 = 12:55;
 y3 = polyval(k3,x3);
 
  k4 = polyfit(downpoint1(:,1),downpoint1(:,2),1);
 x4 = 12:55;
 y4 = polyval(k4,x4);
figure; hold on,axis equal
   plot(skeletonv(:,1),skeletonv(:,2),'g.');
    plot(x1,y1,'linewidth',3,'color','r');
    plot(x2,y2,'linewidth',3,'color','r');
    plot(x3,y3,'linewidth',3,'color','r');
    plot(x4,y4,'linewidth',3,'color','r');