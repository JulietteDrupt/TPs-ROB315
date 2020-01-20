%>% ======================================================================
%>%> @brief Calcule le vecteur des couples dus aux frottements en utilisant
%un modèle de frottements visqueux (slide 159 du cours)
%>%>
%>%> @param q_point Vecteur des vitesses articulaires nombre_articulationsx1
%>%>
%>%> @retval gammaF Vecteur des couples induits par les frottements
%>%> nombre_articulationsx1
%>% ======================================================================
function[gammaF] = CalculCoupleFrottement(q_point)
    
    %Variables globales => Ne fonctionnent pas bien avec Simulink
    %global Fv;
    
    Fv = ones(size(q_point))*10;
    
    gammaF = diag(q_point)*Fv;
    
end