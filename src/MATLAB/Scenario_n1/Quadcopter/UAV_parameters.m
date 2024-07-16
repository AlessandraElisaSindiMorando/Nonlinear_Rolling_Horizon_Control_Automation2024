% Parameters has been chosen according to 
% https://www.parrot.com/assets/s3fs-public/2021-09/bebop-2_user-guide_uk.pdf

% drone mass
mass = 0.5;%[kg]

% gravitational acceleration
g = 9.81;%[ms^{-2}]

% sampling time 
Ts = 0.1;%[s]
% 
% % Referenze for the altitude z time-varing
% ZRef = 2*ones(1,60*2*10+1);
% % for t < 1 min, z1_Ref = 2 m
% 
% for k = 600:1200
% % for t >= 1 min, z1_Ref = 3 m
%     ZRef(1,k) = 3;
% end

% Referenze for the altitude z time-varing
ZRef = 2*ones(1,60*10+1);

t = 0:0.1:30;
ZRef(1,1:301) = 1 + t/30;
