function [J] = CalculJacobienne(alpha,d,theta,r)

    global rE;

    [g_06,g_elem] = CalculMGD(alpha,d,theta,r);
    N = length(alpha);
    
    g_0E = g_06*CalculTransformationElem(0,0,0,rE);
    
    J = [];
    
    for i=1:N
        g_0i = eye(4);
        for j=1:i
            g_0i = g_0i*g_elem{j};
        end
       
        R_0i = g_0i(1:3,1:3);
        
        p_0E = g_0E(1:3,4);
        p_0i = g_0i(1:3,4);
        p_iE = p_0E - p_0i;
        
        J = cat(2,J,[cross(R_0i*[0 0 1].',p_iE);R_0i*[0 0 1].']);    
    end
end