%>% ======================================================================
%>%> @brief Calcule la matrice d'inertie A(q) en utilisant le calcul de 
%>%> l'énergie cinétique de la chaîne articulée (slide 145 du cours)
%>%>
%>%> @param q Configuration articulaire nombre_articulationsx1
%>%>
%>%> @retval A Matrice d'intertie nombre_articulationsxnombre_articulations
%>% ======================================================================
function A = CalculMatriceInertie(q)

    %Variables globales => Ne fonctionnent pas bien avec Simulink
    %global alpha d r m OG I Rred Jm;
    
    %Parametres geometriques du robot - Convention DHM
    d3 = 0.7;
    r1 = 0.5;
    r4 = 0.2;

    alpha = [0 pi/2 0 pi/2 -pi/2 pi/2];
    d = [0 0 d3 0 0 0];
    r = [r1 0 0 r4 0 0];

    %Parametres inertiels du robot

    m = [15 10 1 7 1 0.5];
    OG = [0 0 -0.25;0.35 0 0;0 -0.1 0; 0 0 0;0 0 0;0 0 0];

    I = [[0.8 0 0.05;0 0.8 0;0.05 0 0.1];
        [0.1 0 0.1;0 1.5 0;0.1 0 1.5];
        [0.05 0 0;0 0.01 0;0 0 0.05];
        [0.5 0 0;0 0.5 0;0 0 0.5];
        [0.01 0 0;0 0.01 0;0 0 0.01];
        [0.01 0 0;0 0.01 0;0 0 0.01]];

    %Paramètres inertiels des rotors

    Rred = [100 100 100 70 70 70];
    Jm = ones(size(Rred))*10^(-5);

    theta = [q(1) q(2) q(3)+pi/2 q(4) q(5) q(6)].';
    
    %initialisation des tableaux
    g_0i = eye(4);
    Ig = zeros(3*size(q,1),3);
    A = zeros(size(q,1),size(q,1));
    
    for i = 1:length(m)
        %Transport du tenseur d'inertie en Gi grace au théorème d'Huygens
        %(slide 143 du cours)
        Huy = [OG(i,2)^2+OG(i,3)^2 -OG(i,1)*OG(i,2) -OG(i,1)*OG(i,3);
                -OG(i,1)*OG(i,2) OG(i,1)^2+OG(i,3)^2 -OG(i,2)*OG(i,3);
                -OG(i,1)*OG(i,3) -OG(i,2)*OG(i,3) OG(i,1)^2+OG(i,2)^2];
        
        %Transport du tenseur d'inertie dans le repère R0
        g_elem = CalculTransformationElem(alpha(i),d(i),theta(i),r(i));
        g_0i = g_0i*g_elem;
        ROi = g_0i(1:3,1:3);

        Ig(3*(i-1)+1:3*i,:) = ROi*(I(3*(i-1)+1:3*i,:)-m(i).*Huy)*ROi.';        
    end 
    
    %Calcul des matrices jacobiennes aux centres de masse
    [OJv_Gi, OJ_wi] = CalculMatriceJacobienneGi(alpha, d, theta, r, OG(:,1), OG(:,2), OG(:,3));

    for i=1:size(q,1) 
        A = A + OJv_Gi(:,:,i).'*OJv_Gi(:,:,i)*m(i)+ OJ_wi(:,:,i).'*Ig(3*(i-1)+1:3*i,:)*OJ_wi(:,:,i);
    end
   
    %Ajout de la contribution des actionneurs (slide 144 du cours)
    A = A + diag(Rred.^2.*Jm);

end


    
