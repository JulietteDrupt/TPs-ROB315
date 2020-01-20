%>% ======================================================================
%>%> @brief Retourne, à un instant donné, la consigne de trajectoire 
%>%> correspondant à une interpolation polynomiale du 5eme degré 
%>%> (slide 198 du cours)
%>%> 
%>%> @param qdi Configuration articulaire initiale de la trajectoire
%>%> nombre_articulationsx1
%>%> @param qdf Configuration articulaire finale de la trajectoire
%>%> nombre_articulationsx1
%>%> @param t Instant auquel la consigne de trajectoire est calculée 1x1
%>%>
%>%> @retval qc Consigne de trajectoire à l'instant t sous la forme d'une
%>%> configuration articulaire nombre_articulationsx1
%>% ======================================================================
function[qc] = GeneTraj(qdi,qdf,t)
    tf = 500;   %Temps en ms !
    D = qdf - qdi;
    
    %Calcul de l'interpolation polynomiale de degré 5
    r = 10*(t/tf)^3 - 15*(t/tf)^4 + 6*(t/tf)^5;
    
    qc = qdi + D*r;
end