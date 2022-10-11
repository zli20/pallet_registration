%根据指定点和对应的牵拽向量计算出
%输入的
%输入的
%返回的
function torqueValue=GetTorque(rotateCenter,pt,gravity)
%因为只允许在水平面上旋转，因此将所有在Z轴方向上的值清除
rotateCenter(3)=0;
pt(3)=0;
gravity(3)=0;

%O为旋转中心，P为指定点，F为牵拽力
OP=pt-rotateCenter;
torque=cross(OP,gravity);%扭矩
torqueValue=torque(3);

end