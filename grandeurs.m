% clc, clear, format short g;

%% Masses
mw = 0.1; % kg, masse roue + engrenage + rotor
mb = 1;   % kg, masse chassis
M = mb + 2*mw;

%% Longueur et paramètres caractéristiques
l = 20e-2;  % m, distance entre roues
d = 42e-2;  % m, distance AC
rho = 4e-2;  % m, rayon des roues
L = 2.3e-3;  % inuctance moteur
R = 2.35;  % résistance moteur
k = 0.55;    % constante de couple

AS = 36e-3;

%% Moments d'inertie
Ixw = 12753 * 1e-9;
Iyw = 24874 * 1e-9;

Ix = 5.756e6 * 1e-9;
Iz = 8.872e6 * 1e-9;

Iphi = Ix + Iz + 2 * Ixw + mw * l^2 / 2 + Iyw * l^2 / (2 * rho^2);

K1 = M + 2 * Iyw / rho^2;
K2 = Iphi + mb * d^2;

%% Point d'équilibre
uBar = 0.02; % m/s
rBar = 0.0; % rad/s
yBar = 0;
psiBar = 0;

%% Grandeurs à l'équilibre
ilBar = rho/k * mb * d * (0.5*rBar^2 - 1/l * uBar*rBar);
irBar = rho/k * mb * d * (0.5*rBar^2 + 1/l * uBar*rBar);

UlBar = R * ilBar + k / rho * (uBar - l/2 * rBar);
UrBar = R * irBar + k / rho * (uBar + l/2 * rBar);
UplusBar = UlBar + UrBar;
UmoinsBar = UlBar - UrBar;

K4 = l*k / (2 * rho * R * K2);
K5 = 1/K2 * (mb*d*uBar - l^2*k^2 / (2*rho^2*R));

%% Courbe du parcours
% nuage = readmatrix("courbe_parametree.csv");
% nuage = nuage(1:100:end, 1:2) * 1e-3;

nuage = zeros(1000, 2);
for i=1 : 1000
    nuage(i, 1) = i * 1e-2;
end