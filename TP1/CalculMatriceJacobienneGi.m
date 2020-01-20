%>% ======================================================================
%>%> @brief Calcule les matrices jacobiennes exprimées aux centres de 
%>%> masse en utilsant la formule de Varignon (slide 145 du cours)
%>%>
%>%> @param alpha,d,theta,r Liste des paramètres de la convention DHM 
%>%> nombre_articulationsx1
%>%> @param x_G,y_G,z_G Coordonnées des centres de masse
%>%> nombre_articulationsx3
%>%>
%>%> @retval OJv_Gi Matrices jacobiennes relatives aux vitesses de translation
%>%> 3xnombre_articulationsxnombre_articulations
%>%> @retval OJ_wi Matrices jacobiennes relatives aux vitesses de rotation
%>%> 3xnombre_articulationsxnombre_articulations
%>% ======================================================================
function [OJv_Gi, OJ_wi] = CalculMatriceJacobienneGi(alpha, d, theta, r, x_G, y_G, z_G)

    %Variables globales => Ne fonctionne pas bien avec Simulink
    %global rE;
    
    rE = 0.1;

    %Calcul de la matrice jacobienne au niveau de l'organe terminal
    OJ_OE = CalculJacobienne(alpha, d, theta, r);

    %Initialisation des tableaux
    OJv_Gi = zeros(3,size(OJ_OE,2),size(OJ_OE,2));
    OJ_wi = zeros(3,size(OJ_OE,2),size(OJ_OE,2));
    OJ_Gi = zeros(size(OJ_OE,1),size(OJ_OE,2),size(OJ_OE,2));

    g_0i = eye(4);

    %Calcul du vecteur O0OE dans le repère R0
    [g_06,~] = CalculMGD(alpha,d,theta,r);
    g_6E = CalculTransformationElem(0,0,0,rE);
    g_0E = g_06*g_6E;
    OO0OE = g_0E(1:3,4); OOEO0 = -OO0OE;

    for i=1:size(OJ_OE,2)
        
        OJ_Oi = OJ_OE(:,1:i);
        iOiGi = [x_G(i), y_G(i), z_G(i)]';
        
        %Calcul de la matrice de rotation entre les repères R0 et R(i)
        g_elem = CalculTransformationElem(alpha(i),d(i),theta(i),r(i));
        g_0i = g_0i*g_elem;
        ROi = g_0i(1:3,1:3);
        
        %Calcul du vecteur OEGi dans le repère R0
        OOiGi = ROi*iOiGi;
        
        OO0Oi = g_0i(1:3,4);
        OOEOi = OOEO0 + OO0Oi;
        OOEGi = OOEOi + OOiGi;
        
        %Construction de la matrice de préproduit vectoriel
        OPreproduitVect_OEGi = [ 0 -OOEGi(3) OOEGi(2);
                                OOEGi(3) 0 -OOEGi(1) ;
                                -OOEGi(2) OOEGi(1) 0 ];

        %Application de la formule de Varignon
        OJ_Gi(:,1:i,i) = OJ_Gi(:,1:i,i) + [eye(3), -OPreproduitVect_OEGi; zeros(3,3), eye(3)]*OJ_Oi;
        
        %Extraction des matrices jacobiennes spécifiques
        OJv_Gi(:,:,i) = OJ_Gi(1:3,:,i);
        OJ_wi(:,:,i) = OJ_Gi(4:6,:,i);
    end
end

