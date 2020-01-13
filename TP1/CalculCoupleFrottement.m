function[gammaF] = CalculCoupleFrottement(q_point)
    
    %Global variables => Doesn't work well with simulink
    %global Fv;
    
    Fv = ones(size(q_point))*10;
    
    gammaF = diag(q_point)*Fv;
    
end