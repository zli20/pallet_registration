%�����ָ���㵽ָ���߶ε��������
%�ο�http://blog.sina.com.cn/s/blog_5d5c80840101bnhw.html
%�����line��ָ�����߶Σ�2*3���󣬵�һ��Ϊ�߶εĵ�һ���˵㣬�ڶ���Ϊ�߶εĵڶ����˵�
%�����pt��ָ���ĵ�
%���ص�force��������룬��ά����
function directedDistance=GetDirectedDistanceFromPt2Line(line,pt)
% AΪ��һ���˵㣬BΪ�ڶ����˵㣬PΪָ���ĵ㣬CΪ����
A=line(1,:);
B=line(2,:);
P=pt;
A(3)=0;
B(3)=0;
P(3)=0;

AB=B-A;
AP=P-A;
BP=P-B;
lAB2=dot(AB,AB);
dot_AP_AB=dot(AP,AB);
CA=(-dot_AP_AB)/lAB2 * AB;
CP=CA+AP;
r=dot_AP_AB/lAB2;

if r<0
    directedDistance=AP;
elseif r>1
    directedDistance=BP;
else
    directedDistance=CP;
end

end