%计算出指定点到指定线段的有向距离
%参考http://blog.sina.com.cn/s/blog_5d5c80840101bnhw.html
%输入的line：指定的线段；2*3矩阵，第一行为线段的第一个端点，第二行为线段的第二个端点
%输入的pt：指定的点
%返回的force：有向距离，三维向量
function directedDistance=GetDirectedDistanceFromPt2Line(line,pt)
% A为第一个端点，B为第二个端点，P为指定的点，C为垂足
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