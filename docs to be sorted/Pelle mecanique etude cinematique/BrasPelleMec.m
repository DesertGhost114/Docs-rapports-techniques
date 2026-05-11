%% Les ddls du bras de la pelle mécanique
syms a b c dc hg kj alpha2 alpha3

%% Matrices de rotation

% Rotx
% Roty
% Rotz

%% Vecteur de la Pelle mécanique
OA=[0; 0; a];
AB=[-b; 0;0];
BC=[-c; 0;0];
BD=[db*cos(alpha2); 0; db*sin(alpha2)];
CD=[dc*cos(alpha3); 0; dc*sin(alpha3)];

DE=[-e;0;0];
EF=[-f;0;0];


FG=[fg*cos(alpha8);0; fg*sin(alpha8)];
FI=[-fi;0;0];
GH=[hg*cos(alpha9);0; hg*sin(alpha9)];
IH=[hi*cos(alpha10);0; hi*sin(alpha10)];

HL=[-lh;0;0];
HJ=[jh*cos(alpha13);0; jhg*sin(alpha13)];
JK=[kj*cos(alpha14);0; kj*sin(alpha14)];

LK=[lk*cos(alpha15);0; lk*sin(alpha15)];

LM=[-lm;0;0];
MN=[mn*cos(alpha18);0; mn*sin(alpha18)];
KN=[nk*cos(alpha17);0; nk*sin(alpha17)];