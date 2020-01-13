function [J] = CalculJacobienne(alpha,d,theta,r)

    %Global variables => Doesn't work well with simulink
    %global rE;
    
    rE =0.1;

    [g_06,g_elem] = CalculMGD(alpha,d,theta,r);
    N = length(alpha);
    
    g_0E = g_06*CalculTransformationElem(0,0,0,rE);
    
    J = zeros(size(alpha,2),size(alpha,2));
    
    for i=1:N
        g_0i = eye(4);
        for j=1:i
            g_0i = g_0i*g_elem{j};
        end
       
        R_0i = g_0i(1:3,1:3);
        
        p_0E = g_0E(1:3,4);
        p_0i = g_0i(1:3,4);
        p_iE = p_0E - p_0i;
        
        J(:,i) = [cross(R_0i*[0 0 1].',p_iE);R_0i*[0 0 1].'];   
    end
end