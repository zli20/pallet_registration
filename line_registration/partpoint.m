 a =1;
for i=1:length(skeletonv)
if(skeletonv(i,1)>10&&skeletonv(i,1)<13)
leftpoint1(a,:) = skeletonv(i,:);
a=a+1;
end
end
 a =1;
for i=1:length(skeletonv)
if(skeletonv(i,1)>50&&skeletonv(i,1)<56)
rightpoint1(a,:) = skeletonv(i,:);
a=a+1;
end
end
 a =1;
for i=1:length(skeletonv)
if(skeletonv(i,2)>7&&skeletonv(i,2)<14)
downpoint1(a,:) = skeletonv(i,:);
a=a+1;
end
end
 a =1;
for i=1:length(skeletonv)
if(skeletonv(i,2)>83&&skeletonv(i,2)<89)
    uppoint1(a,:) = skeletonv(i,:);
a=a+1;
end
end