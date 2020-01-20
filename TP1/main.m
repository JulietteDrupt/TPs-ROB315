
%Parametres geometriques du robot - Convention DHM
d3 = 0.7;
r1 = 0.5;
r4 = 0.2;

global alpha d r;

alpha = [0 pi/2 0 pi/2 -pi/2 pi/2];
d = [0 0 d3 0 0 0];
r = [r1 0 0 r4 0 0];

global rE;

rE = 0.1;

%Configurations articulaires initiale et finale
qi = [-pi/2 0 -pi/2 -pi/2 -pi/2 -pi/2].';
qf = [0 pi/4 0 pi/2 pi/2 0].';

%Configuration articulaire choisie
q = qf;

theta = [q(1) q(2) q(3)+pi/2 q(4) q(5) q(6)].';

%Calcul du modèle géométrique direct
[g_06,g_elem] = CalculMGD(alpha,d,theta,r);
g_0E = g_06*CalculTransformationElem(0,0,0,rE);

%Determination de la configuration de l'organe terminal
P = g_0E(1:3,4);
R = g_0E(1:3,1:3);
[angle,axe] = rotationParams(R);

disp('Question 4');
disp('Position');
disp(P);
disp('Vecteur de rotation');
disp(axe);
disp('Angle de rotation');
disp(angle);

figure;
%Visualisation des reperes
VisualisationRepere(q);
title('Reperes');

%Calcul de la matrice jacobienne
J = CalculJacobienne(alpha,d,theta,r);

%Vitesse articulaire et torseur cinematique de l'organe terminal
q_point = [0.5 1.0 -0.5 0.5 1.0 -0.5].';
x_point = J*q_point;

disp('Question 6');
disp('Vitesse de translation');
disp(x_point(1:3));
disp('Vitesse de rotation');
disp(x_point(4:6));

%Restriction aux translations
J = J(1:3,:);

figure;
%Visualisation des ellipsoides
VisualisationEllipsoide(J,q,g_0E);
title('Ellipsoide');

%Calcul de la manipulabilite et de la direction privilégiée de transmission
%en vitesse

M = J*J.';

[vecteurs_propres,valeurs_propres]=eig(M);
valeurs_singulieres = sort(sqrt(diag(valeurs_propres)));

disp('Question 7');
disp('Direction privilegiee');
disp(vecteurs_propres(:,end));

Manip = prod(valeurs_singulieres);

disp('Manipulabilite');
disp(Manip)

%Positions cartésiennes initiale et finale de l'organe terminal
Xdi = [-0.1 -0.7 0.3].';
Xdf = [0.64 -0.1 1.14].';

%Conditions initiales sur la configuration articulaire
q0i = [-1.57 0.00 -1.47 -1.47 -1.47 -1.47].';
q0f = [0 0.8 0 1 2 0].';

%Choix de la position et de la condition initiales
Xd = Xdf;
q0 = q0f;

%Paramètres du calcul du MGI
global kmax alphaStep epsilon;

kmax = 100;
alphaStep = 0.005;
epsilon = 10^(-3);

q_etoile = MGI(Xd,q0,kmax,epsilon,alphaStep);

disp('Question 8');
disp('Configuration articulaire');
disp(q_etoile);

theta_etoile = [q_etoile(1) q_etoile(2) q_etoile(3)+pi/2 q_etoile(4) q_etoile(5) q_etoile(6)];
[g_06_etoile,~] = CalculMGD(alpha,d,theta_etoile,r);
g_0E_etoile = g_06_etoile*CalculTransformationElem(0,0,0,rE);

disp('Position objectif');
disp(Xd);
disp('Position calculee');
disp(g_0E_etoile(1:3,4));
disp('Erreur');
disp(norm(Xd-g_0E_etoile(1:3,4)));

%Parametres du calcul du MCI
V = 1;
Te = 10^(-3);

[Q,X] = MCI(Xdi,Xdf,V,Te,qi);

figure;
%Visualisation de la trajectoire réelle de l'organe terminal
VisualisationTrajectoire(Q);
%Visualisation de la consigne de trajectoire de l'organe terminal
plot3(X(1,:),X(2,:),X(3,:),'m--');
hold on;
title('Trajectoire brute');

%Valeurs des butees articulaires
qmin = [-pi -pi/2 -pi -pi -pi/2 -pi].';
qmax = [0 pi/2 0 pi/2 pi/2 pi/2].'; 

%Parametre d'evitement des butees
alphaH = 0.01;

[Q_butees,X_butees] = MCIbutees(Xdi,Xdf,V,Te,qi,alphaH,qmin,qmax);

figure;
%Visualisation de la trajectoire réelle de l'organe terminal
VisualisationTrajectoire(Q_butees);
%Visualisation de la consigne de trajectoire de l'organe terminal
plot3(X_butees(1,:),X_butees(2,:),X_butees(3,:),'m--');
hold on;
title('Trajectoire avec evitement');


%Plot des valeurs des variables articulaires

T=[];

Q_plot = Q;

for i = 0:size(Q_plot,2)-1
    T = [T i*Te]; 
end

figure;
for i = 1:6
    subplot(3,2,i)
    plot(T,Q_plot(i,:),'b');
    legend(['q' num2str(i)]);
    hold on;
    plot(T,ones(1,size(Q_plot,2))*qmin(i),'r');
    hold on;
    plot(T,ones(1,size(Q_plot,2))*qmax(i),'r');
    hold on;
    xlabel('Temps (en s)');
    ylabel('Angle (en rad)');
end
title('Trajectoire brute');

T = [];

Q_plot = Q_butees;

for i = 0:size(Q_plot,2)-1
    T = [T i*Te]; 
end

figure;
for i = 1:6
    subplot(3,2,i)
    plot(T,Q_plot(i,:),'b');
    legend(['q' num2str(i)]);
    hold on;
    plot(T,ones(1,size(Q_plot,2))*qmin(i),'r');
    hold on;
    plot(T,ones(1,size(Q_plot,2))*qmax(i),'r');
    hold on;
    xlabel('Temps (en s)');
    ylabel('Angle (en rad)');
end
title('Trajectoire avec evitement');



