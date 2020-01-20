%>% ======================================================================
%>%> @brief Résout le Modèle Cinématique Inverse en utilisant une 
%>%> discrétisation de la trajectoire désirée et un appel itératif du MGI
%>%> - Restriction aux seules vitesses de translation
%>%> 
%>%> @param Xdi Position cartésienne initiale l'organe terminal 3x1
%>%> @param Xdf Position cartésienne finale l'organe terminal 3x1
%>%> @param V Vitesse de translation de l'organe terminal 1x1
%>%> @param Te Période d'échantillonnage de la trajectoire 1x1
%>%> @param qi Configuration articulaire initiale nombre_articulationsx1
%>%>
%>%> @retval qd Liste des configurations articulaires de la trajectoire
%>%> nombre_articulationsxlongueur_trajectoire
%>%> @retval Xd Liste des positions de l'organe terminale de la trajectoire
%>%> 3xlongueur_trajectoire
%>% ======================================================================
function [qd,Xd] = MCI(Xdi,Xdf,V,Te,qi)

    global kmax epsilon alphaStep; 
    
    %Calcul de la distance à parcourir et du pas de discrétisation
    distance = norm(Xdf - Xdi);
    ds = V*Te;
    
    %Intialisation des listes
    Xd = [Xdi];
    qd = [qi];
    qdk = qi;
    Xdk = Xdi;
    
    %Discrétisation de la trajectoire
    for i = 1:distance/ds
        %Calcul de la position de l'organe terminal
        Xdk = Xdk+ds*(Xdf-Xdi)/distance;
        Xd = [Xd Xdk];
        
        %Calcul du MGI
        qdk = MGI(Xdk,qdk,kmax,epsilon,alphaStep);
        qd = [qd, qdk];
    end
end