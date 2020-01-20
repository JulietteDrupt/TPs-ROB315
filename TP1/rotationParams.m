%>% ======================================================================
%>%> @brief Retourne l'angle et l'axe de rotation associés à une matrice de
%>%> rotation, calculés grace à la formule de Rodrigues (slide 55 du cours)
%>%> 
%>%> @param R Matrice de rotation 3x3
%>%>
%>%> @retval angle Angle de rotation 1x1
%>%> @retval axe Axe de rotation 3x1
%>% ======================================================================
function [angle,axe] = parametresRotation(R)
    angle = atan2(0.5*sqrt((R(3,2)-R(2,3))^2 + (R(1,3) - R(3,1))^2 + (R(2,1) - R(1,2))^2),0.5*(R(1,1) + R(2,2) + R(3,3) - 1));
    axe = [(R(3,2) - R(2,3))/(2*sin(angle)) (R(1,3) - R(3,1))/(2*sin(angle)) (R(2,1) - R(1,2))/(2*sin(angle))].';
end