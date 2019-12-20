function VisualisationRepere(q)

    global alpha d r rE;

    theta = [q(1) q(2) q(3)+pi/2 q(4) q(5) q(6)];
    
    [g_06,g_elem] = CalculMGD(alpha,d,theta,r);
    
    g_6E = CalculTransformationElem(0,0,0,rE);
    g_0E = g_06*g_6E;
    
    VisualisationTriedre(eye(4),'b');
    VisualisationTriedre(g_0E,'r');
    VisualisationRobot(g_elem,g_6E);
    grid on;
    axis equal;
end

function VisualisationRobot(g_elem,g_6E)
    pointActuel = [0;0;0];
    rotation = eye(3);

    plot3([pointActuel(1)],[pointActuel(2)],[pointActuel(3)],'go','MarkerFaceColor','g');
    hold on;

    for i = 1:length(g_elem)
        pointSuivant = pointActuel + rotation*g_elem{i}(1:3,4);
        rotation = rotation*g_elem{i}(1:3,1:3);
        
        plot3([pointActuel(1) pointSuivant(1)],[pointActuel(2) pointSuivant(2)],[pointActuel(3) pointSuivant(3)],'g');
        hold on;
        plot3([pointSuivant(1)],[pointSuivant(2)],[pointSuivant(3)],'go','MarkerFaceColor','g');
        hold on;
        
        pointActuel = pointSuivant;
    end
    
    pointSuivant = pointActuel + rotation*g_6E(1:3,4);
    plot3([pointActuel(1) pointSuivant(1)],[pointActuel(2) pointSuivant(2)],[pointActuel(3) pointSuivant(3)],'g');
    hold on;
    plot3([pointSuivant(1)],[pointSuivant(2)],[pointSuivant(3)],'go','MarkerFaceColor','g');
    hold on;
end

function VisualisationTriedre(g, couleur)
    centre = g(1:3,4);
    rotation = g(1:3,1:3);
    
    X = rotation*[1;0;0];
    Y = rotation*[0;1;0];
    Z = rotation*[0;0;1];
    
    plot3([centre(1) (centre(1)+X(1))],[centre(2) (centre(2)+X(2))],[centre(3) (centre(3)+X(3))], couleur);
    hold on;
    plot3([centre(1) (centre(1)+Y(1))],[centre(2) (centre(2)+Y(2))],[centre(3) (centre(3)+Y(3))], couleur);
    hold on;
    plot3([centre(1) (centre(1)+Z(1))],[centre(2) (centre(2)+Z(2))],[centre(3) (centre(3)+Z(3))], couleur);
    hold on;
end