%>% ======================================================================
%>%> @brief Retourne, � un instant donn�, la consigne de trajectoire 
%>%> correspondant � une interpolation polynomiale du 5eme degr� 
%>%> (slide 198 du cours)
%>%> 
%>%> @param qdi Configuration articulaire initiale de la trajectoire
%>%> nombre_articulationsx1
%>%> @param qdf Configuration articulaire finale de la trajectoire
%>%> nombre_articulationsx1
%>%> @param t Instant auquel la consigne de trajectoire est calcul�e 1x1
%>%>
%>%> @retval qc Consigne de trajectoire � l'instant t sous la forme d'une
%>%> configuration articulaire nombre_articulationsx1
%>% ======================================================================
function[qc] = GeneTraj(qdi,qdf,t)
    tf = 500;   %Temps en ms !
    D = qdf - qdi;
    
    %Calcul de l'interpolation polynomiale de degr� 5
    r = 10*(t/tf)^3 - 15*(t/tf)^4 + 6*(t/tf)^5;
    
    qc = qdi + D*r;
end