function [Av,Bv] = ABqLd_WT_beta(DT,rhoP,Rotor_Lambda,Rotor_Pitch,Rotor_cQ)
% ABqLd calculates system (A) and input (B) matrix for quasiLPV
% representation from ABgL and augments the states with integrating
% disturbances.
%
% Inputs: * DT: Sampling time
%         * rho: Scheduling parameter vector: rho = V
%
% Outputs: * A: Augmented system matrix A(rho) at current rho
%          * B: Augmented Input matrix B(rho) at current rho

% Pablo S.G. Cisneros, Herbert Werner, ICS TUHH
% modified for WECS simulation: Antje Dittmer

% Calculate system and input matrices at current wind speed
[Arho,Brho] = ABqL_WT_beta(rhoP,Rotor_Lambda,Rotor_Pitch,Rotor_cQ);

%% Calculate discrete forms of A,B
A2 = Arho*Arho;
A3 = Arho*A2;
Apre = (1.0/24.0)*DT^4*A3+(1.0/6.0)*DT^3*A2+0.5*DT^2*Arho+DT*eye(size(Arho));

%% Finish calculation of A,B
Ak = Apre*Arho + eye(size(Arho));
Bk = Apre*Brho;

%% Calculate new matrices augmented system
ny = 2; %
nx = size(Arho,1);
nyvec = nx-1:nx; % 
nd = 1;% B: columns disturbance input
nuvec = 2:3; % B: columns control input
Cdiag = eye(ny); %diag([97,1]); %eye(ny); %diag([97,1]);


A = [eye(2) Cdiag * Ak(nyvec,:);
    zeros(size(Ak,1),ny), Ak];
B = [Cdiag * Bk(nyvec,nuvec); Bk(:,nuvec)];
Bv1 = [Cdiag * Bk(nyvec,nd); Bk(:,nd)];

Av = [A,Bv1; [zeros(nd,(nx+ny)) 0*eye(nd)]];
Bv = [B; zeros(nd, size(B,2))];


end