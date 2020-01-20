%>% ======================================================================
%>%> @brief R�sout le Mod�le Cin�matique Inverse en utilisant une 
%>%> discr�tisation de la trajectoire d�sir�e et un appel it�ratif du MGI
%>%> - Restriction aux seules vitesses de translation
%>%> 
%>%> @param Xdi Position cart�sienne initiale l'organe terminal 3x1
%>%> @param Xdf Position cart�sienne finale l'organe terminal 3x1
%>%> @param V Vitesse de translation de l'organe terminal 1x1
%>%> @param Te P�riode d'�chantillonnage de la trajectoire 1x1
%>%> @param qi Configuration articulaire initiale nombre_articulationsx1
%>%>
%>%> @retval qd Liste des configurations articulaires de la trajectoire
%>%> nombre_articulationsxlongueur_trajectoire
%>%> @retval Xd Liste des positions de l'organe terminale de la trajectoire
%>%> 3xlongueur_trajectoire
%>% ======================================================================
function [qd,Xd] = MCI(Xdi,Xdf,V,Te,qi)

    global kmax epsilon alphaStep; 
    
    %Calcul de la distance � parcourir et du pas de discr�tisation
    distance = norm(Xdf - Xdi);
    ds = V*Te;
    
    %Intialisation des listes
    Xd = [Xdi];
    qd = [qi];
    qdk = qi;
    Xdk = Xdi;
    
    %Discr�tisation de la trajectoire
    for i = 1:distance/ds
        %Calcul de la position de l'organe terminal
        Xdk = Xdk+ds*(Xdf-Xdi)/distance;
        Xd = [Xd Xdk];
        
        %Calcul du MGI
        qdk = MGI(Xdk,qdk,kmax,epsilon,alphaStep);
        qd = [qd, qdk];
    end
end