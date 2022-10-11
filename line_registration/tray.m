close all
clc
WorkPath='G:\���̼��\���̼������\tray_decete_one\result\';
WorkPath2='G:\���̼��\��������\�����̷���\';
GridEdgeOffset =8;
%% ԭʼ����
point=load([WorkPath,'bag.txt']);
ske1 = load([WorkPath,'ske.txt']);
Ps=point;
fig1=figure('color','w'); hold on,
plot3(ske1(:,1),ske1(:,2),ske1(:,3),'b.'),
plot3(Ps(:,1),Ps(:,2),Ps(:,3),'g.'),
axis equal;%�����������Ϊ1:1
axis off;
h = [ 50 50 400 300];
set(gcf,'Position',h)%����ͼ�����figure�߾�
xlabel('X/mm');
ylabel('Y/mm');
%% voxel����
voxelpoint=load([WorkPath,'bag_voxel.txt']);
voxel=voxelpoint;
fig2=figure('color','w'); hold on,

ssk(:,1)  = (ske1(:,1) - (min(point(:,1))))/10 + GridEdgeOffset;
ssk(:,2)  = (ske1(:,2) - (min(point(:,2))))/10 + GridEdgeOffset;
ssk(:,3) = ske1(:,3)/10;

plot3(ssk(:,1),ssk(:,2),ssk(:,3),'g.'),

plot3(voxel(:,1),voxel(:,2),voxel(:,3),'g.'),
axis equal;%�����������Ϊ1:1
h = [ 50 50 400 300];
set(gcf,'Position',h)%����ͼ�����figure�߾�
xlabel('X/mm');
ylabel('Y/mm');
axis off;
%annotation1 = annotation(fig2,'arrow',[0.130 0.130],[0.110 0.925]);%����ͷ
%annotation2 = annotation(fig2,'arrow',[0.130 0.905],[0.110 0.110]);
%% ͼ���������Ƶ�
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
%% ����ԭʼ�ǵ�
TrayEdges=load([WorkPath,'RCorners_tray.txt']);
angle=-0.0174;%���õ���ת�����ҳˣ�����Ҫ�Ӹ���
leftEdge=TrayEdges(2,1);
rightEdge=TrayEdges(4,1);
frontEdge=TrayEdges(2,2);
rearEdge=TrayEdges(4,2);
%PlotBoundingBox('g','-',angle,0,leftEdge,rightEdge,frontEdge,rearEdge);
rotateMat=[cos(angle) -sin(angle);
    sin(angle) cos(angle)];
TrayCorners=zeros(4,3);
TrayCorners(1,1:2)=[leftEdge frontEdge]*rotateMat;%��ǰ
TrayCorners(2,1:2)=[leftEdge rearEdge]*rotateMat;%���
TrayCorners(3,1:2)=[rightEdge frontEdge]*rotateMat;%��ǰ
TrayCorners(4,1:2)=[rightEdge rearEdge]*rotateMat;%�Һ�
%% ͼ�������ϱ�Ե��׼
fig4=figure('color','w'); hold on, axis off;axis equal;
h = [ 150 50 400 300];
set(gcf,'Position',h)
plot3(TrayEdgeResponseGrid(:,1),TrayEdgeResponseGrid(:,2),TrayEdgeResponseGrid(:,3),'k*')

xlabel('X/cm');
ylabel('Y/cm');

% �����ϱ�Ե��׼
%��ͷ����ֵ�͵����
%�ȳ�ʼ�����̵�4���ǵ�
%tray    425  760 
%00  450 760
%20 430 720
%460 780
%470 780
% 40 
TrayWidth=450;
TrayLength=760;
GridWidth=10;

%��ʼ����׼ģ��
%ʹ�����������Ĺ̶�ģ��λ��
% leftNear=[0 0 0];%�������ֵ�͵Ľǵ�
% leftFar=[0 TrayLength/GridWidth 0];
% rightNear=[TrayWidth/GridWidth 0 0];
% rightFar=[TrayWidth/GridWidth TrayLength/GridWidth 0];
%ʹ�õ��԰�Χ�е�ģ���ʼλ��
LengthVector=TrayCorners(2,:)-TrayCorners(1,:);
WidthVector=TrayCorners(3,:)-TrayCorners(1,:);
%����Voxelģ���µ��ĸ��ǵ�
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
%���ݳ�ʼ��ģ����ĸ��ǵ㣬ʹ������ʽ�������̱�Ե�߶Σ�
%��һάΪ�߶��������ڶ�ά�����߶��������˵㣻����ά������ά������������ֵ
TrayLines=zeros(4,2,3);
TrayLines(1,:,:)=[leftNear;leftFar];
TrayLines(2,:,:)=[leftFar;rightFar];
TrayLines(3,:,:)=[rightFar;rightNear];
TrayLines(4,:,:)=[rightNear;leftNear];
% 
% %%������ת����
rotateCenter=(leftNear+rightFar)/2;
% plot3(rotateCenter(1),rotateCenter(2),rotateCenter(3),'bo','linewidth',2);
%�����ÿ���ߵĳ���ֵ����Ҫ�����㵽�߶ξ���ļ��㣨�����жϴ����Ƿ������߶��ϣ�
LineLength=zeros(4);
for i=1:4
    LineVector=TrayLines(i,1,:)-TrayLines(i,2,:);
    LineVector=squeeze(LineVector);
    LineLength(i)=norm(LineVector);
end

%����������
% tLine=zeros(2,3);
% for i=1:4
%     for j=1:2
%         plot3(TrayLines(i,j,1),TrayLines(i,j,2),TrayLines(i,j,3),'bs');
%     end
%     tLine(:,:)=TrayLines(i,:,:);
%     line(tLine(:,1),tLine(:,2),tLine(:,3),'color','b','linewidth',3);
% end

%%��ʼ����׼����ز���
directedDistance=zeros(1,3);%��ǰ�������
gravity=zeros(1,3);%��ǰǣק��
sumPtGravity=zeros(1,3);%��ǰ���ǣק��ʸ���ܺ�
meanPtGravity=zeros(1,3);%��ǰ���ܵ���ƽ��ʸ��ǣק��
sumTotalGravity=zeros(1,3);
meanTotalGravity=[1000 1000 1000];%ģ���ܵ���ƽ��ǣק��
sumPtEnergy=0;
meanPtEnergy=0;
sumTotalEnergy=0;
meanTotalEnergy=0;
torsionAngle=0;%��ǰŤ�ǣ����з�����ʱ��Ϊ������
sumPtTorsionAngle=0;
meanPtTorsionAngle=0;
sumTotalTorsionAngle=0;%Ť���ܺ�
meanTotalTorsionAngle=0;
Gravities=zeros(size(TrayEdgeResponseGrid,1),3);%������ֻ�д�С��ȫ�����õ�ģ���������
TorsionAngles=zeros(1,size(TrayEdgeResponseGrid,1));%Ť��,����ת�Ǵ���Ť�صĴ�С�����з�����ʱ��Ϊ������
Energies=zeros(1,size(TrayEdgeResponseGrid,1));
EnergiesAllTimes=zeros(zeros,35);

%%��׼��ѭ��
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
            %��������
            if lDistance==0
                lDistance=DistanceMinLimit;
                gravity(:)=0.0000001;
            else
                if lDistance<DistanceMinLimit
                    lDistance=DistanceMinLimit;
                    directedDistance=DistanceMinLimit*directedDistance/norm(directedDistance);
                end
                gravity=directedDistance/lDistance^2; %����������뷽����ͬ����ֵ������ƽ���ɷ���
            end
            sumPtGravity=sumPtGravity+gravity;
            if norm(gravity)>norm(maxGravity)
                maxGravity=gravity;
            end
            %��������
            sumPtEnergy=sumPtEnergy+lDistance;
            if lDistance<minPtDistance
                minPtDistance=lDistance;
            end
            %����Ť��
            torsionAngle=GetTorque(rotateCenter,TrayEdgeResponseGrid(nPt,:),gravity);%�����صĴ�С��ΪŤ��
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
    %����ת
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
    %��ƽ��
    TrayLines(:,:,1)=TrayLines(:,:,1)+meanTotalGravity(1);
    TrayLines(:,:,2)=TrayLines(:,:,2)+meanTotalGravity(2);
    rotateCenter=rotateCenter+meanTotalGravity;
    py(1) =  py(1)+meanTotalGravity(1);
    py(2) =  py(2)+meanTotalGravity(2);
%     %���»�������ģ��
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
    %Title=['��',num2str(cntRecycleTimes),'�ε�������ǰ������',num2str(sumTotalEnergy)];
    %title(Title);
end
%cntRecycleTimes
toc
t = toc;
%�鿴ƥ��Ч��
plot3(TrayEdgeResponseGrid(:,1),TrayEdgeResponseGrid(:,2),TrayEdgeResponseGrid(:,3),'k.','linewidth',1)
for i=1:4
    for j=1:2
        plot3(TrayLines(i,j,1),TrayLines(i,j,2),TrayLines(i,j,3),'bs','linewidth',3);
    end
    tLine(:,:)=TrayLines(i,:,:);
    line(tLine(:,1),tLine(:,2),tLine(:,3),'color','b','linewidth',3);
    plot3(rotateCenter(1),rotateCenter(2),rotateCenter(3),'bo','linewidth',3);
end
%% �����仯ͼ
fig5=figure('color','w');
h = [ 550 50 400 300];
set(gcf,'Position',h)
plot(EnergiesAllTimes,'*-');
xlabel('iteration times');
ylabel('potential energy');
 %Title=['��ǰ���ܣ�',num2str(sumTotalEnergy)];
  %title(Title,'FontSize',15);
annotation1 = annotation(fig5,'arrow',[0.1399 0.1399],[0.90 0.969]);
annotation2 = annotation(fig5,'arrow',[0.88 0.96],[0.15 0.15]);
%% �Ǽ�
fig6 = figure('color','w'); hold on,
modelske=load([WorkPath,'llzskele.txt']);

ske = modelske/10;
h = [ 150 50 400 300];
set(gcf,'Position',h)

axis equal;%�����������Ϊ1:1
axis off;
h = [ 50 50 400 300];
set(gcf,'Position',h)%����ͼ�����figure�߾�
LengthVector=TrayCorners(2,:)-TrayCorners(1,:);
WidthVector=TrayCorners(3,:)-TrayCorners(1,:);
%����Voxelģ���µ��ĸ��ǵ�
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

%���ݳ�ʼ��ģ����ĸ��ǵ㣬ʹ������ʽ�������̱�Ե�߶Σ�
%��һάΪ�߶��������ڶ�ά�����߶��������˵㣻����ά������ά������������ֵ
TrayLines=zeros(4,2,3);
TrayLines(1,:,:)=[leftNear;leftFar];
TrayLines(2,:,:)=[leftFar;rightFar];
TrayLines(3,:,:)=[rightFar;rightNear];
TrayLines(4,:,:)=[rightNear;leftNear];

%%������ת����
rotateCenter=(leftNear+rightFar)/2;
plot3(rotateCenter(1),rotateCenter(2),rotateCenter(3),'bo','linewidth',2);
%�����ÿ���ߵĳ���ֵ����Ҫ�����㵽�߶ξ���ļ��㣨�����жϴ����Ƿ������߶��ϣ�
LineLength=zeros(4);
for i=1:4
    LineVector=TrayLines(i,1,:)-TrayLines(i,2,:);
    LineVector=squeeze(LineVector);
    LineLength(i)=norm(LineVector);
end
%%����������
tLine=zeros(2,3);
for i=1:4
    for j=1:2
        plot3(TrayLines(i,j,1),TrayLines(i,j,2),TrayLines(i,j,3),'bs');
    end
    tLine(:,:)=TrayLines(i,:,:);
    line(tLine(:,1),tLine(:,2),tLine(:,3),'color','b','linewidth',3);
end