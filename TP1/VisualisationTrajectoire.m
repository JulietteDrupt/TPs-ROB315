%>% ======================================================================
%>%> @brief Retourne un visuel de la trajectoire définie par la liste de
%>%> configurations articulaires Q
%>%> 
%>%> @param Q Liste des configurations articulaires de la trajectoire
%>%> nombre_articulations x longeur_trajectoire
%>% ======================================================================
function VisualisationTrajectoire(Q)
    
    global alpha d r rE;
    
    P=[];
    
    for i=1:size(Q,2)
        Qi = Q(:,i);
        
        %MGD
        theta_Q = [Qi(1) Qi(2) Qi(3)+pi/2 Qi(4) Qi(5) Qi(6)];
        [g_06_Q,~] = CalculMGD(alpha,d,theta_Q,r);
        g_0E_Q = g_06_Q*CalculTransformationElem(0,0,0,rE);
        
        %Coordonnées cartésiennes de l'organe terminal
        P = [g_0E_Q(1:3,4) P];
    end
    
    %Plot des repères intermédiaires
    VisualisationRepere(Q(:,1));
    VisualisationRepere(Q(:,end));
    
    VisualisationRepere(Q(:,floor(size(Q,2)/3)));
    VisualisationRepere(Q(:,floor(2*size(Q,2)/3)));
    
    %Plot de la trajectoire
    plot3(P(1,:),P(2,:),P(3,:),'c--');
    
    hold on;
    grid on;
    axis equal;
end