
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
q = qi;

theta = [q(1) q(2) q(3)+pi/2 q(4) q(5) q(6)].';

%Calcul du modèle géométrique direct
[g_06,g_elem] = CalculMGD(alpha,d,theta,r);
g_0E = g_06*CalculTransformationElem(0,0,0,rE);

%Determination de la configuration de l'organe terminal
P = g_0E(1:3,4);
R = g_0E(1:3,1:3);
[angle,axe] = rotationParams(R);

disp('Position');
disp(P);
disp('Vecteur de rotation');
disp(axe);
disp('Angle de rotation');
disp(angle);

figure;
%Visualisation des reperes
VisualisationRepere(q);

%Calcul de la matrice jacobienne
J = CalculJacobienne(alpha,d,theta,r);

%Vitesse articulaire et torseur cinematique de l'organe terminal
q_point = [0.5 1.0 -0.5 0.5 1.0 -0.5].';
x_point = J*q_point;

disp('Vitesse de translation');
disp(x_point(1:3));
disp('Vitesse de rotation');
disp(x_point(4:6));

%Restriction aux translations
J = J(1:3,:);

figure;
%Visualisation des ellipsoides
VisualisationEllipsoide(J,q,g_0E);

%Calcul de la manipulabilite
Manip = sqrt(det(J*J.'));

disp('Manipulabilite');
disp(Manip)

%Positions cartésiennes initiale et finale de l'organe terminal
Xdi = [-0.1 -0.7 0.3].';
Xdf = [0.64 -0.1 1.14].';

%Conditions initiales sur la configuration articulaire
q0i = [-1.57 0.00 -1.47 -1.47 -1.47 -1.47].';
q0f = [0 0.8 0 1 2 0].';

%Choix de la position et de la condition initiales
Xd = Xdi;
q0 = q0i;

%Paramètres du calcul du MGI
global kmax alphaStep epsilon;

kmax = 100;
alphaStep = 0.005;
epsilon = 10^(-3);

q_etoile = MGI(Xd,q0,kmax,epsilon,alphaStep);

theta_etoile = [q_etoile(1) q_etoile(2) q_etoile(3)+pi/2 q_etoile(4) q_etoile(5) q_etoile(6)];
[g_06_etoile,~] = CalculMGD(alpha,d,theta_etoile,r);
g_0E_etoile = g_06_etoile*CalculTransformationElem(0,0,0,rE);

disp('Position objectif');
disp(Xd);
disp('Position calculee');
disp(g_0E_etoile(1:3,4));

%Parametres du calcul du MCI
V = 1;
Te = 10^(-3);

Q = MCI(Xdi,Xdf,V,Te,qi);

Qf = Q(:,end);

theta_Q = [Qf(1) Qf(2) Qf(3)+pi/2 Qf(4) Qf(5) Qf(6)];
[g_06_Q,~] = CalculMGD(alpha,d,theta_Q,r);
g_0E_Q = g_06_Q*CalculTransformationElem(0,0,0,rE);

disp('Position visee');
disp(Xdf);
disp('Position atteinte');
disp(g_0E_Q(1:3,4));

%Valeurs des butees articulaires
qmin = [-pi -pi/2 -pi -pi -pi/2 -pi].';
qmax = [0 pi/2 0 pi/2 pi/2 pi/2].'; 

%Parametre d'evitement des butees
alphaH = 0.01;

Q_butees = MCIbutees(Xdi,Xdf,V,Te,qi,alphaH,qmin,qmax);

Qf_butees = Q_butees(:,end);

theta_Q_butees = [Qf_butees(1) Qf_butees(2) Qf_butees(3)+pi/2 Qf_butees(4) Qf_butees(5) Qf_butees(6)];
[g_06_Q,~] = CalculMGD(alpha,d,theta_Q_butees,r);
g_0E_Q = g_06_Q*CalculTransformationElem(0,0,0,rE);

disp('Position visee');
disp(Xdf);
disp('Position atteinte');
disp(g_0E_Q(1:3,4));

%Plot

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
end

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
end

