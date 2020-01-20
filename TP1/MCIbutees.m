%>% ======================================================================
%>%> @brief Résout le Modèle Cinématique Inverse en utilisant la méthode 
%>%> du gradient projeté pour l'évitement des butées articulaires 
%>%> (slide 116 du cours) - Restriction aux seules vitesses de translation
%>%> 
%>%> @param Xdi Position cartésienne initiale l'organe terminal 3x1
%>%> @param Xdf Position cartésienne finale l'organe terminal 3x1
%>%> @param V Vitesse de translation de l'organe terminal 1x1
%>%> @param Te Période d'échantillonnage de la trajectoire 1x1
%>%> @param qi Configuration articulaire initiale nombre_articulationsx1
%>%> @param alphaH Facteur d'évitement des butées articulaires 1x1
%>%> @param qmin, qmax Valeurs des butées articulaires
%>%> nombre_articulationsx1
%>%>
%>%> @retval qd Liste des configurations articulaires de la trajectoire
%>%> nombre_articulationsxlongueur_trajectoire
%>%> @retval Xd Liste des positions de l'organe terminale de la trajectoire
%>%> 3xlongueur_trajectoire
%>% ======================================================================
function [qd,Xd] = MCIbutees(Xdi,Xdf,V,Te,qi,alphaH,qmin,qmax)

    global kmax epsilon;

    %Calcul de la distance à parcourir et du pas de discrétisation
    distance = norm(Xdf - Xdi);
    ds = V*Te;
    
    %Initialisation des listes
    Xd = [Xdi];
    qd = [qi];
    qdk = qi;
    Xdk = Xdi;
    
    %Discrétisation de la trajectoire
    for i = 1:distance/ds
        %Calcul de la position de l'organe terminal
        Xdk = Xdk+ds*(Xdf-Xdi)/distance;
        Xd = [Xd Xdk];
        
        theta = [qdk(1) qdk(2) qdk(3)+pi/2 qdk(4) qdk(5) qdk(6)];
        
        %Calcul du MGI avec évitement des butées articulaires
        qdk = MGIbutees(Xdk,qdk,kmax,epsilon,qmin,qmax,alphaH);
        qd = [qd, qdk];
    end
end

%>% ======================================================================
%>%> @brief Résout le Modèle Géométrique Inverse en utilisant une méthode
%>%> itérative basée sur la pseudo-inverse de la matrice jacobienne et la 
%>%> méthode du gradient projeté pour l'évitement des butées articulaires
%>%> - Restriction aux seules vitesses de translation
%>%>
%>%> @param Xd Position cartésienne désirée pour l'organe terminal 3x1
%>%> @param q0 Configuration articulaire initiale de la méthode itérative
%>%> nombre_articulationsx1
%>%> @param kmax Nombre maximal d'itérations 1x1
%>%> @param epsilon Critère d'arrêt sur l'erreur en position de l'organe
%>%> terminal 1x1
%>%> @param qmin, qmax Valeurs des butées articulaires
%>%> nombre_articulationsx1
%>%> @param alphaH Facteur d'évitement des butées articulaires 1x1
%>%>
%>%> @retval q Configuration articulaire solution nombre_articulationx1
%>% ======================================================================
function [q] = MGIbutees(Xd,q0,kmax,epsilon,qmin,qmax,alphaH)

    global alpha d r rE;

    %Calcul du MGD
    theta = [q0(1) q0(2) q0(3)+pi/2 q0(4) q0(5) q0(6)];
    [g_06,~] = CalculMGD(alpha,d,theta,r);
    g_0E = g_06*CalculTransformationElem(0,0,0,rE);
    
    k = 0;
    %Iterations
    while(norm(Xd - g_0E(1:3,4)) > epsilon && k < kmax)
        k = k+1;
        
        %Calcul de la matrice jacobienne
        J = CalculJacobienne(alpha,d,theta,r);
        J = J(1:3,:);

        %Calcul itératif de la configuration articulaire avec projection du
        %gradient de la fonction de coût dans le noyaux de la matrice
        %jacobienne
        q0 = q0 + pinv(J)*(Xd - g_0E(1:3,4)) - alphaH*(eye(size(J,2)) - pinv(J)*J)*H(q0,qmax,qmin);
        
        %Calcul du MGD
        theta = [q0(1) q0(2) q0(3)+pi/2 q0(4) q0(5) q0(6)];
        [g_06,~] = CalculMGD(alpha,d,theta,r);
        g_0E = g_06*CalculTransformationElem(0,0,0,rE);  
    end
    q = q0;
end

%>% ======================================================================
%>%> @brief Retourne la valeur du gradient de la fonction de coût utilisée 
%>%> par la méthode du grandient projeté pour éviter les butées articulaires
%>%>
%>%> @param q Configuration articulaire courante nombre_articulationsx1
%>%> @param qmin, qmax Valeurs des butées articulaires
%>%> nombre_articulationsx1
%>%>
%>%> @retval h valeur du gradient de la fonction de coût 1x1
%>% ======================================================================
function [h] = H(q,qmax,qmin)
    h = [];
    dq = qmax-qmin;
    qbarre = dq/2;
    for i = 1:size(q,1)
        h = [h ; 2*(q(i)-qbarre(i))/dq(i)^2];
    end
end
