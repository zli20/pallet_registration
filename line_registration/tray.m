close all
clc
WorkPath='G:\托盘检测\托盘检测整理\tray_decete_one\result\';
WorkPath2='G:\托盘检测\行李数据\带托盘分类\';
GridEdgeOffset =8;
%% 原始点云
point=load([WorkPath,'bag.txt']);
ske1 = load([WorkPath,'ske.txt']);
Ps=point;
fig1=figure('color','w'); hold on,
plot3(ske1(:,1),ske1(:,2),ske1(:,3),'b.'),
plot3(Ps(:,1),Ps(:,2),Ps(:,3),'g.'),
axis equal;%横纵坐标比设为1:1
axis off;
h = [ 50 50 400 300];
set(gcf,'Position',h)%设置图像距离figure边距
xlabel('X/mm');
ylabel('Y/mm');
%% voxel点云
voxelpoint=load([WorkPath,'bag_voxel.txt']);
voxel=voxelpoint;
fig2=figure('color','w'); hold on,

ssk(:,1)  = (ske1(:,1) - (min(point(:,1))))/10 + GridEdgeOffset;
ssk(:,2)  = (ske1(:,2) - (min(point(:,2))))/10 + GridEdgeOffset;
ssk(:,3) = ske1(:,3)/10;

plot3(ssk(:,1),ssk(:,2),ssk(:,3),'g.'),

plot3(voxel(:,1),voxel(:,2),voxel(:,3),'g.'),
axis equal;%横纵坐标比设为1:1
h = [ 50 50 400 300];
set(gcf,'Position',h)%设置图像距离figure边距
xlabel('X/mm');
ylabel('Y/mm');
axis off;
%annotation1 = annotation(fig2,'arrow',[0.130 0.130],[0.110 0.925]);%画箭头
%annotation2 = annotation(fig2,'arrow',[0.130 0.905],[0.110 0.110]);
%% 图，托盘疑似点
TrayEdgeResponseGrid = load([WorkPath,'stray_voxel.txt']);
if ~isempty(TrayEdgeResponseGrid)
    fig3 = figure('color','w');
    hold on, axis off;axis equal;
   % plot3(voxel(:,1),voxel(:,2),voxel(:,3),'c.'),
    plot3(TrayEdgeResponseGrid(:,1),TrayEdgeResponseGrid(:,2),TrayEdgeResponseGrid(:,3),'k*')
%     for i=1:length(TrayEdgeResponseGrid)
%         quiver3(TrayEdgeResponseGrid(i,1),TrayEdgeResponseGrid(i,2),TrayEdgeResponseGrid(i,3)...
%             ,DragImg(i,1),DragImg(i,2),DragImg(i,3),'r','filled','LineWidth',1);
%     end
    h = [ 550 300 400 300];
    set(gcf,'Position',h)
end
xlabel('X/cm');
ylabel('Y/cm');
hold off;
%% 托盘原始角点
TrayEdges=load([WorkPath,'RCorners_tray.txt']);
angle=-0.0174;%采用的旋转矩阵右乘，所以要加负号
leftEdge=TrayEdges(2,1);
rightEdge=TrayEdges(4,1);
frontEdge=TrayEdges(2,2);
rearEdge=TrayEdges(4,2);
%PlotBoundingBox('g','-',angle,0,leftEdge,rightEdge,frontEdge,rearEdge);
rotateMat=[cos(angle) -sin(angle);
    sin(angle) cos(angle)];
TrayCorners=zeros(4,3);
TrayCorners(1,1:2)=[leftEdge frontEdge]*rotateMat;%左前
TrayCorners(2,1:2)=[leftEdge rearEdge]*rotateMat;%左后
TrayCorners(3,1:2)=[rightEdge frontEdge]*rotateMat;%右前
TrayCorners(4,1:2)=[rightEdge rearEdge]*rotateMat;%右后
%% 图，托盘上边缘配准
fig4=figure('color','w'); hold on, axis off;axis equal;
h = [ 150 50 400 300];
set(gcf,'Position',h)
plot3(TrayEdgeResponseGrid(:,1),TrayEdgeResponseGrid(:,2),TrayEdgeResponseGrid(:,3),'k*')

xlabel('X/cm');
ylabel('Y/cm');

% 托盘上边缘配准
%大头坐标值低的情况
%先初始化托盘的4个角点
%tray    425  760 
%00  450 760
%20 430 720
%460 780
%470 780
% 40 
TrayWidth=450;
TrayLength=760;
GridWidth=10;

%初始化配准模板
%使用坐标轴对齐的固定模板位置
% leftNear=[0 0 0];%左侧坐标值低的角点
% leftFar=[0 TrayLength/GridWidth 0];
% rightNear=[TrayWidth/GridWidth 0 0];
% rightFar=[TrayWidth/GridWidth TrayLength/GridWidth 0];
%使用弹性包围盒的模板初始位置
LengthVector=TrayCorners(2,:)-TrayCorners(1,:);
WidthVector=TrayCorners(3,:)-TrayCorners(1,:);
%解算Voxel模型下的四个角点
%kong+1+2
leftNear=TrayCorners(1,:);
leftNear(1)=(leftNear(1) - (min(point(:,1))))/GridWidth;
leftNear(2)=(leftNear(2) - (min(point(:,2))))/GridWidth;



leftNear(1:2)=leftNear(1:2)+GridEdgeOffset;
leftFar=leftNear+TrayLength*LengthVector/norm(LengthVector)/GridWidth+1;
rightNear=leftNear+TrayWidth*WidthVector/norm(WidthVector)/GridWidth+2;
rightFar=rightNear+TrayLength*LengthVector/norm(LengthVector)/GridWidth;
rightFar(3)=rightFar(3)/GridWidth;

tic;
%根据初始化模板的四个角点，使用两点式定义托盘边缘线段；
%第一维为线段条数；第二维代表线段有两个端点；第三维代表三维点有三个坐标值
TrayLines=zeros(4,2,3);
TrayLines(1,:,:)=[leftNear;leftFar];
TrayLines(2,:,:)=[leftFar;rightFar];
TrayLines(3,:,:)=[rightFar;rightNear];
TrayLines(4,:,:)=[rightNear;leftNear];
% 
% %%绘制旋转中心
rotateCenter=(leftNear+rightFar)/2;
% plot3(rotateCenter(1),rotateCenter(2),rotateCenter(3),'bo','linewidth',2);
%计算出每条线的长度值，主要用作点到线段距离的计算（用于判断垂足是否落在线段上）
LineLength=zeros(4);
for i=1:4
    LineVector=TrayLines(i,1,:)-TrayLines(i,2,:);
    LineVector=squeeze(LineVector);
    LineLength(i)=norm(LineVector);
end

%绘制四条边
% tLine=zeros(2,3);
% for i=1:4
%     for j=1:2
%         plot3(TrayLines(i,j,1),TrayLines(i,j,2),TrayLines(i,j,3),'bs');
%     end
%     tLine(:,:)=TrayLines(i,:,:);
%     line(tLine(:,1),tLine(:,2),tLine(:,3),'color','b','linewidth',3);
% end

%%初始化配准的相关参数
directedDistance=zeros(1,3);%当前有向距离
gravity=zeros(1,3);%当前牵拽力
sumPtGravity=zeros(1,3);%当前点的牵拽力矢量总和
meanPtGravity=zeros(1,3);%当前点受到的平均矢量牵拽力
sumTotalGravity=zeros(1,3);
meanTotalGravity=[1000 1000 1000];%模型受到的平均牵拽力
sumPtEnergy=0;
meanPtEnergy=0;
sumTotalEnergy=0;
meanTotalEnergy=0;
torsionAngle=0;%当前扭角；带有方向；逆时针为正方向
sumPtTorsionAngle=0;
meanPtTorsionAngle=0;
sumTotalTorsionAngle=0;%扭角总和
meanTotalTorsionAngle=0;
Gravities=zeros(size(TrayEdgeResponseGrid,1),3);%拉力，只有大小，全部作用到模板的质心上
TorsionAngles=zeros(1,size(TrayEdgeResponseGrid,1));%扭矩,以旋转角代表扭矩的大小；带有方向；逆时针为正方向
Energies=zeros(1,size(TrayEdgeResponseGrid,1));
EnergiesAllTimes=zeros(zeros,35);

%%配准大循环
cntRecycleTimes=0;
DistanceMinLimit=0.5;
GravityMaxLimit=2;
maxGravity=[0 0 0];
maxTorsionAngle=0;
minPtDistance=100000;
xzjz = [1 0 0;0 1 0; 0 0 1];
py = [0 0 0];
while(cntRecycleTimes<70 )%&& norm(meanTotalGravity)>0.001
    for nPt=1:size(TrayEdgeResponseGrid,1)
        sumPtGravity=[0 0 0];
        sumPtEnergy=0;
        sumPtTorsionAngle=0;
        maxGravity=[0 0 0];
        maxTorsionAngle=0;
        minPtDistance=100000;
        
        %oneLine=TrayLines(1,:,:);
        %oneLine=squeeze(oneLine);
        for nLine=1:4
            directedDistance=...
                GetDirectedDistanceFromPt2Line(squeeze(TrayLines(nLine,:,:)),TrayEdgeResponseGrid(nPt,:));
            lDistance=norm(directedDistance);         
            %计算引力
            if lDistance==0
                lDistance=DistanceMinLimit;
                gravity(:)=0.0000001;
            else
                if lDistance<DistanceMinLimit
                    lDistance=DistanceMinLimit;
                    directedDistance=DistanceMinLimit*directedDistance/norm(directedDistance);
                end
                gravity=directedDistance/lDistance^2; %力与有向距离方向相同，其值与距离的平方成反比
            end
            sumPtGravity=sumPtGravity+gravity;
            if norm(gravity)>norm(maxGravity)
                maxGravity=gravity;
            end
            %计算势能
            sumPtEnergy=sumPtEnergy+lDistance;
            if lDistance<minPtDistance
                minPtDistance=lDistance;
            end
            %计算扭角
            torsionAngle=GetTorque(rotateCenter,TrayEdgeResponseGrid(nPt,:),gravity);%将力矩的大小作为扭角
            sumPtTorsionAngle=sumPtTorsionAngle+torsionAngle;
            if abs(torsionAngle)>abs(maxTorsionAngle)
                maxTorsionAngle=torsionAngle;
            end
        end
        meanPtGravity=maxGravity;
        Gravities(nPt,:)=meanPtGravity;
        
        Energies(nPt)=-1/minPtDistance;
        
        meanPtTorsionAngle=maxTorsionAngle;
        TorsionAngles(nPt)=meanPtTorsionAngle;
    end
    sumTotalEnergy=sum(Energies(:,:));
    EnergiesAllTimes(cntRecycleTimes+1)=sumTotalEnergy;
    meanTotalEnergy=sumTotalEnergy/nPt;
    
    meanTotalGravity=10* sum(Gravities(:,:))/nPt;
    
    sumTotalTorsionAngle=sum(TorsionAngles);
    meanTotalTorsionAngle=0.005*sumTotalTorsionAngle/nPt;
    R=[
        cos(meanTotalTorsionAngle) -sin(meanTotalTorsionAngle) 0;
        sin(meanTotalTorsionAngle) cos(meanTotalTorsionAngle) 0;
        0 0 1];
    xzjz = xzjz * R;
    %先旋转
    oneLine=zeros(2,3);
    for i=1:4
        oneLine=TrayLines(i,:,:);
        oneLine=squeeze(oneLine);
        oneLine(:,1)=oneLine(:,1)-rotateCenter(1);
        oneLine(:,2)=oneLine(:,2)-rotateCenter(2);
        oneLine=(R*oneLine')';
        oneLine(:,1)=oneLine(:,1)+rotateCenter(1);
        oneLine(:,2)=oneLine(:,2)+rotateCenter(2);
        TrayLines(i,:,:)=oneLine;
    end
    %再平移
    TrayLines(:,:,1)=TrayLines(:,:,1)+meanTotalGravity(1);
    TrayLines(:,:,2)=TrayLines(:,:,2)+meanTotalGravity(2);
    rotateCenter=rotateCenter+meanTotalGravity;
    py(1) =  py(1)+meanTotalGravity(1);
    py(2) =  py(2)+meanTotalGravity(2);
%     %重新绘制托盘模型
%     for i=1:4
%         for j=1:2
%             plot3(TrayLines(i,j,1),TrayLines(i,j,2),TrayLines(i,j,3),'ys');
%         end
%         tLine(:,:)=TrayLines(i,:,:);
%         line(tLine(:,1),tLine(:,2),tLine(:,3),'color','y');
%         plot3(rotateCenter(1),rotateCenter(2),rotateCenter(3),'yo','linewidth',1);
%     end
%     
    cntRecycleTimes=cntRecycleTimes+1;
    %Title=['第',num2str(cntRecycleTimes),'次迭代，当前能量：',num2str(sumTotalEnergy)];
    %title(Title);
end
%cntRecycleTimes
toc
t = toc;
%查看匹配效果
plot3(TrayEdgeResponseGrid(:,1),TrayEdgeResponseGrid(:,2),TrayEdgeResponseGrid(:,3),'k.','linewidth',1)
for i=1:4
    for j=1:2
        plot3(TrayLines(i,j,1),TrayLines(i,j,2),TrayLines(i,j,3),'bs','linewidth',3);
    end
    tLine(:,:)=TrayLines(i,:,:);
    line(tLine(:,1),tLine(:,2),tLine(:,3),'color','b','linewidth',3);
    plot3(rotateCenter(1),rotateCenter(2),rotateCenter(3),'bo','linewidth',3);
end
%% 能量变化图
fig5=figure('color','w');
h = [ 550 50 400 300];
set(gcf,'Position',h)
plot(EnergiesAllTimes,'*-');
xlabel('iteration times');
ylabel('potential energy');
 %Title=['当前势能：',num2str(sumTotalEnergy)];
  %title(Title,'FontSize',15);
annotation1 = annotation(fig5,'arrow',[0.1399 0.1399],[0.90 0.969]);
annotation2 = annotation(fig5,'arrow',[0.88 0.96],[0.15 0.15]);
%% 骨架
fig6 = figure('color','w'); hold on,
modelske=load([WorkPath,'llzskele.txt']);

ske = modelske/10;
h = [ 150 50 400 300];
set(gcf,'Position',h)

axis equal;%横纵坐标比设为1:1
axis off;
h = [ 50 50 400 300];
set(gcf,'Position',h)%设置图像距离figure边距
LengthVector=TrayCorners(2,:)-TrayCorners(1,:);
WidthVector=TrayCorners(3,:)-TrayCorners(1,:);
%解算Voxel模型下的四个角点
leftNear=TrayCorners(1,:);
leftNear(1)=(leftNear(1) - (min(point(:,1))))/GridWidth;
leftNear(2)=(leftNear(2) - (min(point(:,2))))/GridWidth;

% p = min(Ps(:,2))
% q = min(Ps(:,1))
TrayWidth=440;
TrayLength=790;
GridWidth=10;
ske(:,1) = ske(:,1) + 25.8796+8;
ske(:,2) = ske(:,2) + 56.2034+8;
%plot3(ske(:,1),ske(:,2),ske(:,3),'k.')
plot3(ssk(:,1),ssk(:,2),ssk(:,3),'k.');
leftNear(1:2)=leftNear(1:2)+GridEdgeOffset;
leftFar=leftNear+TrayLength*LengthVector/norm(LengthVector)/GridWidth;
rightNear=leftNear+TrayWidth*WidthVector/norm(WidthVector)/GridWidth;
rightFar=rightNear+TrayLength*LengthVector/norm(LengthVector)/GridWidth;
rightFar(3)=rightFar(3)/GridWidth;

%根据初始化模板的四个角点，使用两点式定义托盘边缘线段；
%第一维为线段条数；第二维代表线段有两个端点；第三维代表三维点有三个坐标值
TrayLines=zeros(4,2,3);
TrayLines(1,:,:)=[leftNear;leftFar];
TrayLines(2,:,:)=[leftFar;rightFar];
TrayLines(3,:,:)=[rightFar;rightNear];
TrayLines(4,:,:)=[rightNear;leftNear];

%%绘制旋转中心
rotateCenter=(leftNear+rightFar)/2;
plot3(rotateCenter(1),rotateCenter(2),rotateCenter(3),'bo','linewidth',2);
%计算出每条线的长度值，主要用作点到线段距离的计算（用于判断垂足是否落在线段上）
LineLength=zeros(4);
for i=1:4
    LineVector=TrayLines(i,1,:)-TrayLines(i,2,:);
    LineVector=squeeze(LineVector);
    LineLength(i)=norm(LineVector);
end
%%绘制四条边
tLine=zeros(2,3);
for i=1:4
    for j=1:2
        plot3(TrayLines(i,j,1),TrayLines(i,j,2),TrayLines(i,j,3),'bs');
    end
    tLine(:,:)=TrayLines(i,:,:);
    line(tLine(:,1),tLine(:,2),tLine(:,3),'color','b','linewidth',3);
end