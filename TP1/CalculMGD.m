
function [g_06,g_elem] = CalculMGD(alpha,d,theta,r)

    g_06 = eye(4);
    N = length(alpha);
    g_elem = cell(1,length(alpha));

    for i=1:N
        g_elem{i} = CalculTransformationElem(alpha(i),d(i),theta(i),r(i));
        g_06 = g_06*g_elem{i};
    end 
end