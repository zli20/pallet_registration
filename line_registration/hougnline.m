
[m,n]=size(lefttt);%ͳ�Ƶ��� 
n_max=300;%����ռ���������ֵ  
h=zeros(315,2*n_max);  
theta_i=1;  
sigma=70;%���������ֵ   
%ֱ�߹�ʽ�Ƶ�  
for theta=0:0.01:3.14  
    p=[-sin(theta),cos(theta)];  
    d=p*lefttt;  
    for i=1:n
   %���ڻ���ռ���d�Ƚϴ󣬶�dֵ����������  
    h(theta_i,round(d(i)/10+n_max))=h(theta_i,round(d(i)/10+n_max))+1;  
    end  
    theta_i=theta_i+1;  
end  
[theta_x,cc]=find(h>sigma);%����ͶƱ������sigma��λ��  
l_number=size(theta_x);%����ֱ������  
r=(cc-n_max)*10;%����ԭ�ؾ���R  
theta_x=0.01*theta_x;%��theta��ԭ  
figure('color','w');  axis equal;
plot(lefttt(1,:),lefttt(2,:),'*');  axis equal;
hold on  
x_line=0:20:100;  
  
for i=1:l_number  
%    if(abs(cos(theta_x(i)))<0.01)%б�ʲ����ڵ����  
%         x=r(i);y=1:100;  
%         plot(x,y,'r*');  
%     else  
        y(i,:)=tan(theta_x(i))*x_line+r(i)/cos(theta_x(i));%�����������         
end  
 plot(x_line,y(i,:),'r-'); 
hold off  
figure('color','w');  
imshow(uint8(10*h));%չʾ����ռ��� 


%%
 iter =20000;
d = zeros(iter,length(uppoint));
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
    d(i,j)= norm(tem)/norm(ll);
 if(d(i,j)) <10
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
    plot3(voxel(:,1),voxel(:,2),voxel(:,3),'c.'),
    plot3(TrayEdgeResponseGrid(:,1),TrayEdgeResponseGrid(:,2),TrayEdgeResponseGrid(:,3),'k*')
 line(point3(:,1),point3(:,2),point3(:,3),'color','b');


