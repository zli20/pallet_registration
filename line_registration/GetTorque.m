%����ָ����Ͷ�Ӧ��ǣק���������
%�����
%�����
%���ص�
function torqueValue=GetTorque(rotateCenter,pt,gravity)
%��Ϊֻ������ˮƽ������ת����˽�������Z�᷽���ϵ�ֵ���
rotateCenter(3)=0;
pt(3)=0;
gravity(3)=0;

%OΪ��ת���ģ�PΪָ���㣬FΪǣק��
OP=pt-rotateCenter;
torque=cross(OP,gravity);%Ť��
torqueValue=torque(3);

end