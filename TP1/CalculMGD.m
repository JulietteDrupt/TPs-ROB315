function [g_06,g_elem] = CalculMGD(alpha,d,theta,r)
%CALCULMGD : calcule le MGD d'un robot quelconque en chaîne ouverte simple

    g_06 = eye(4);
    N = length(alpha);
    g_elem = cell(1,N);

    for i=1:N
        g_elem{i} = CalculTransformationElem(alpha(i),d(i),theta(i),r(i));
        g_06 = g_06*g_elem{i};
    end 
end

