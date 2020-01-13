
%Parametres geometriques du robot - Convention DHM
d3 = 0.7;
r1 = 0.5;
r4 = 0.2;

global alpha d r rE;

alpha = [0 pi/2 0 pi/2 -pi/2 pi/2];
d = [0 0 d3 0 0 0];
r = [r1 0 0 r4 0 0];

rE = 0.1;

%Parametres inertiels du robot

global m OG;

m = [15 10 1 7 1 0.5];
OG = [0 0 -0.25;0.35 0 0;0 -0.1 0; 0 0 0;0 0 0;0 0 0];

global I;

I = [[0.8 0 0.05;0 0.8 0;0.05 0 0.1];
    [0.1 0 0.1;0 1.5 0;0.1 0 1.5];
    [0.05 0 0;0 0.01 0;0 0 0.05];
    [0.5 0 0;0 0.5 0;0 0 0.5];
    [0.01 0 0;0 0.01 0;0 0 0.01];
    [0.01 0 0;0 0.01 0;0 0 0.01]];

%Paramètres inertiels des rotors

global Rred Jm;

Rred = [100 100 100 70 70 70];
Jm = ones(size(Rred))*10^(-5);

%Paramètres du modèle de frottement

global Fv;

Fv = ones(size(Rred))*10;

%Configurations articulaires initiale et finale
qi = [-1 0 -1 -1 -1 -1].';
qf = [0 1 0 0 0 0].';

%Configuration articulaire choisie
q = qi;

%Valeurs des butees articulaires
qmin = [-pi -pi/2 -pi -pi -pi/2 -pi].';
qmax = [0 pi/2 0 pi/2 pi/2 pi/2].'; 

theta = [q(1) q(2) q(3)+pi/2 q(4) q(5) q(6)].';

%Calcul des matrices jacobiennes en Gi
[OJv_Gi, OJ_wi] = CalculMatriceJacobienneGi(alpha, d, theta, r, OG(:,1),OG(:,2),OG(:,3));

%Calcul de la matrice d'inertie
A = CalculMatriceInertie(q);

disp('Matrice interie');
disp(A);

%Calcul des bornes sur la matrice d'inertie
Nconfigs = 1000;
mu2 = 0;
mu1 = 999999;

for i=1:Nconfigs
    qTemp = qmin + rand()*(qmax-qmin);
    A = CalculMatriceInertie(qTemp);
    [~,valeurs] = eig(A);
    tempMu2 = max(diag(valeurs));
    tempMu1 = min(diag(valeurs));
    
    if(tempMu2 > mu2)
        mu2 = tempMu2;
    end
    
    if(tempMu1 < mu1)
        mu1 = tempMu1;
    end
end

disp('Borne inferieure sur A');
disp(mu1);
disp('Borne superieure sur A');
disp(mu2);

%Calcul du vecteur des couples de gravite
G = CalculCoupleGravite(q);

disp('Vecteur des couples de gravite');
disp(G);

%Calcul de la borne superieure sur le vecteur des couples de gravite
gb = 0;

for i=1:Nconfigs
    qTemp = qmin + rand()*(qmax-qmin);
    G = CalculCoupleGravite(q);
    tempGb = norm(G,1);
    
    if(tempGb > gb)
        gb = tempGb;
    end
end

disp('Borne superieure sur G selon la norme 1');
disp(gb);

%Calcul des temps d'exectuion minimaux de la trajectoire qi->qf pour chaque
%articulation

tf = CalculTempsTrajectoire(qi,qf,mu2);

disp('Temps execution minimal de la trajectoire pour chaque articulation');
disp(tf);

