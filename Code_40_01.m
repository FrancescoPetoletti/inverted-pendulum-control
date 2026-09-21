%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%                                                         %%%%%%%%%
%%%%%%%%%       ANALISI E SIMULAZIONE DI SISTEMI AEROSPAZIALI     %%%%%%%%%
%%%%%%%%%                       A.A. 2025-2026                    %%%%%%%%%
%%%%%%%%%                                                         %%%%%%%%%
%%%%%%%%%                          GRUPPO 40                      %%%%%%%%%
%%%%%%%%%                        LABORATORIO 01                    %%%%%%%%
%%%%%%%%%                                                         %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;
close all;
tic
%% CODE

% importazione parametri
lab1P=lab1P_parameters();

% 2.1

% definisco i parametri inziali
z0=[0; 0; deg2rad(1); 0];

% introduco il vettore tempo
t=linspace(0, 100, 1000);
t_0=t(1);
t_f=t(end);

% risolvo il sistema con una ODE (in base all'analisi cambio il solver
% selezionato)
ODE_obj=ode;
ODE_obj.ODEFcn = @(t, z) lab1P_f1(z, lab1P);
ODE_obj.InitialValue=z0;
ODE_obj.Solver='ode45';

ODEResults_obj=solve(ODE_obj, t_0, t_f);
% definisco il vettore dei tempi
t_ci=ODEResults_obj.Time;
% definisco il vettore delle soluzioni
z=ODEResults_obj.Solution;


% figure per il documento inziali
figure('Name','Posizione');
plot(t_ci, z(1,:), 'LineWidth',1.5);
xlabel('Tempo t [s]');
ylabel('Posizione x [m]');
grid on;

figure('Name','Angolo')
plot(t_ci, rad2deg(z(3,:)), 'LineWidth', 1.5);
xlabel('Tempo t [s]');
ylabel('Angolo \theta [deg]');
grid on;

% ripeto la risoluzione del sistema usando un ode più precisa

ODE_obj=ode;
ODE_obj.ODEFcn = @(t, z) lab1P_f1(z, lab1P);
ODE_obj.InitialValue=z0;
ODE_obj.Solver='ode89';

% utilizzo tolleranze più stringenti

ODE_obj.AbsoluteTolerance=1e-12;
ODE_obj.RelativeTolerance=1e-6;

ODEResults_obj=solve(ODE_obj, t_0, t_f);
t2_ci=ODEResults_obj.Time;
z2=ODEResults_obj.Solution;

% utilizzo un interpolatore per riportare i vettori theta alla stessa lunghezza
% (per quanto si introducano errori dovuti all'utilizzo della spline in
% questa prima analisi essi risultano accettabili)
z2_allineato = interp1(t_ci, z(3,:), t2_ci, 'spline');

% rappresentazione differenza 2 ode
figure('Name','Differenza con ode 89')
subplot(2, 1, 1);
plot(t2_ci, abs(rad2deg(z2(3,:)-z2_allineato)));
xlabel('Tempo t [s]');
ylabel('Differenza tra gli angoli [deg]');
grid on;

% riporto i vettori x alla stessa lunghezza
z2_1allineato = interp1(t_ci, z(1,:), t2_ci, 'spline');

subplot(2, 1, 2);
plot(t2_ci, abs((z2(1,:)-z2_1allineato)));
xlabel('Tempo t [s]');
ylabel('Differenza tra le posizioni [m]');
grid on;

% studio errore singolo ode

% risolvo nuovamente il sistema variando la tolleranza
% ridefinisco il blocco ODE (in base all'analisi cambio il solver
% selezionato)
ODE_obj=ode;
ODE_obj.ODEFcn = @(t, z) lab1P_f1(z, lab1P);
ODE_obj.InitialValue=z0;
ODE_obj.Solver='ode23';

% definisco un vettore di tolleranze relative e assolute
rel_tolvect=[1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7];
abs_tolvect=rel_tolvect.^2;

% uso lo stesso vettore dei tempi per risolvere la Ode
t_grid = linspace(t_0, t_f, 2000);

ODEResults_obj=solve(ODE_obj, t_grid);
zz=ODEResults_obj.Solution;

% definisco dei parametri base
testtol_x=zz(1, :);
testtol_theta=zz(3, :);

% definisco un vettore tempo di esecuzione
t_soluzione=zeros(length(rel_tolvect), 1);
% definisco i vettori errore
max_errx=zeros(length(rel_tolvect), 1);
max_errtheta=zeros(length(rel_tolvect), 1);

% risolvo la ode per ogni tolleranza
for j=1:length(rel_tolvect)
    
    tic
    ODE_obj.AbsoluteTolerance=abs_tolvect(j);
    ODE_obj.RelativeTolerance=rel_tolvect(j);
    
    ODEResults_obj=solve(ODE_obj, t_grid);
    z_succ=ODEResults_obj.Solution;
    t_succ=ODEResults_obj.Time;

    % definisco l'errore a ogni passo
    max_errx(j)=max(abs(testtol_x-z_succ(1, :)));
    max_errtheta(j)=max(abs(testtol_theta-z_succ(3, :)));

    % sostituisco i parametri base
    testtol_x=z_succ(1, :);
    testtol_theta=z_succ(3, :);
    t_soluzione(j)=toc;

end

% rappresento le differenze tra le varie tolleranze
figure('Name','Errore tra ode');
subplot(2, 1, 1);
loglog(rel_tolvect, max_errx, '-o');
xlabel('Tolleranza');
ylabel('Errore massimo su x');
set(gca, 'XDir', 'reverse');
grid on;

subplot(2, 1, 2);
loglog(rel_tolvect, rad2deg(max_errtheta), '-o');
xlabel('Tolleranza');
ylabel('Errore massimo su theta');
set(gca, 'XDir', 'reverse');
grid on;


% 2.2

% riinizzializzo i parametri come in precedenza con le nuove condizioni
z0=[0; 0; 0; 0];
t=linspace(0, 50, 1000);
t_0=t(1);
t_f=t(end);

ODE_obj=ode;
ODE_obj.ODEFcn = @(t, z) lab1P_f2(t, z, @lab1P_input, lab1P); 
ODE_obj.InitialValue=z0;
ODE_obj.Solver='ode23';


ODEResults_obj=solve(ODE_obj, t_0, t_f);
t_d=ODEResults_obj.Time;
z_d=ODEResults_obj.Solution;

% Disturbo
% inizializzo il vettore
dis=zeros(size(t));

% riempio il vettore usando la stessa funzione usata nell'ode
for j=1:length(t)
   
    dis(j)=lab1P_input(t(j), lab1P);

end

% figure per il documento 
figure('Name','Risposta del sistema a disturbo')
subplot(1, 3, 1)
plot(t, dis, 'LineWidth',1.5);
xlabel('Tempo t [s]');
ylabel('Disturbo');
grid on;

subplot(1, 3, 2)
plot(t_d, z_d(1,:), 'LineWidth',1.5);
xlabel('Tempo t [s]');
ylabel('Posizione x [m]');
grid on;

subplot(1, 3, 3)
plot(t_d, rad2deg(z_d(3,:)), 'LineWidth', 1.5);
xlabel('Tempo t [s]');
ylabel('Angolo \theta [deg]');
grid on;

% ripeto nuovamente l'analisi con tolleranze maggiori
ODE_obj=ode;
ODE_obj.ODEFcn = @(t, z) lab1P_f2(t, z, @lab1P_input, lab1P); 
ODE_obj.InitialValue=z0;
ODE_obj.Solver='ode89';

ODE_obj.AbsoluteTolerance=1e-12;
ODE_obj.RelativeTolerance=1e-6;

ODEResults_obj=solve(ODE_obj, t_0, t_f);

t2_d=ODEResults_obj.Time;
z2_d=ODEResults_obj.Solution;

% replico lo stesso procedimento per avere le stesse dimensioni dei vettori
z2_allineato = interp1(t_d, z_d(3,:), t2_d, 'spline');

% rappresentazione differenza 2 ode
figure('Name','Differenza con ode 89')
subplot(2, 1, 1);
plot(t2_d, abs(rad2deg(z2_d(3,:)-z2_allineato)));
xlabel('Tempo t [s]');
ylabel('Differenza tra gli angoli [deg]');
grid on;

z2_1allineato = interp1(t_d, z_d(1,:), t2_d, 'spline');

subplot(2, 1, 2);
plot(t2_d, abs((z2_d(1,:)-z2_1allineato)));
xlabel('Tempo t [s]');
ylabel('Differenza tra le posizioni [m]');
grid on;

% studio errore singolo ode
% risolvo nuovamente il sistema variando la tolleranza
% ridefinisco il blocco ODE (in base all'analisi cambio il solver
% selezionato)

ODE_obj=ode;
ODE_obj.ODEFcn = @(t, z) lab1P_f2(t, z, @lab1P_input, lab1P);

ODE_obj.InitialValue=z0;
ODE_obj.Solver='ode23';

% definisco un vettore di tolleranze relative e assolute
rel_tolvect=[1e-2,1e-3,1e-4, 1e-5, 1e-6, 1e-7];
abs_tolvect=rel_tolvect.^2;

% uso lo stesso vettore dei tempi per risolvere la ode
t_grid = linspace(t_0, t_f, 2000);

ODEResults_obj=solve(ODE_obj, t_grid);
zz=ODEResults_obj.Solution;

% definisco dei parametri base
testtol_x=zz(1, :);
testtol_theta=zz(3, :);

% definisco un vettore tempo di esecuzione
t_soluzione=zeros(length(rel_tolvect), 1);
% definisco i vettori errore
max_errx=zeros(length(rel_tolvect), 1);
max_errtheta=zeros(length(rel_tolvect), 1);

% risolvo la Ode per ogni tolleranza
for j=1:length(rel_tolvect)
    
    tic
    ODE_obj.AbsoluteTolerance=abs_tolvect(j);
    ODE_obj.RelativeTolerance=rel_tolvect(j);
    
    ODEResults_obj=solve(ODE_obj, t_grid);
    z_succ=ODEResults_obj.Solution;
    t_succ=ODEResults_obj.Time;

    % definisco l'errore a ogni passo
    max_errx(j)=max(abs(testtol_x-z_succ(1, :)));
    max_errtheta(j)=max(abs(testtol_theta-z_succ(3, :)));

    % sostituisco i parametri base
    testtol_x=z_succ(1, :);
    testtol_theta=z_succ(3, :);
    t_soluzione(j)=toc;

end

% rappresento le differenze tra le varie tolleranze
figure('Name','Errore tra ode');
subplot(2, 1, 1);
loglog(rel_tolvect, max_errx, '-o');
xlabel('Tolleranza');
ylabel('Errore massimo su x');
set(gca, 'XDir', 'reverse');
grid on;

subplot(2, 1, 2);
loglog(rel_tolvect, rad2deg(max_errtheta), '-o');
xlabel('Tolleranza');
ylabel('Errore massimo su theta');
set(gca, 'XDir', 'reverse');
grid on;


% 3.1 SIMULINK

% condizioni inziali nulle
z03=[0; 0; 0; 0];

% tolleranze
tol_rel=1e-3;
tol_abs=1e-6;


% ottengo i dati da simulink
ex=sim("model_40_01_1.slx");

% plot del disturbo
figure('Name','Disturbo');
plot(ex.tout,ex.d, 'LineWidth', 2);
xlabel('Tempo t [s]');
ylabel('Disturbo d');
grid on, box on

% plot della posizione
figure ('Name','Posizione');
yyaxis left
plot(ex.tout, squeeze(ex.x), 'LineWidth', 1.5);
ylabel('Posizione x [m]');
yyaxis right
plot(ex.tout,ex.d, '--');
xlabel('Tempo t [s]');
ylabel('Disturbo d');
grid on, box on

% plot dell'angolo
figure ('Name','Angolo');
yyaxis left
plot(ex.tout, squeeze(rad2deg(ex.theta)), 'LineWidth', 1.5);
ylabel('Angolo \theta [deg]');
yyaxis right
plot(ex.tout,ex.d, '--');
xlabel('Tempo t [s]');
ylabel('Disturbo d');
grid on, box on


% rappresentazione differenza con codice matlab
figure('Name','Differenza con MATLAB')
subplot(2, 1, 1);
plot(ex.tout, abs(rad2deg(z2_d(3, :)-squeeze(ex.theta)')));
xlabel('Tempo t [s]');
ylabel('Differenza tra gli angoli [deg]');
grid on;


subplot(2, 1, 2);
plot(ex.tout, abs(z2_d(1, :)-squeeze(ex.x)'));
xlabel('Tempo t [s]');
ylabel('Differenza tra la posizione [m]');
grid on;

% 3.2 
% testo diversi risolutori su simulink

% differenza con simulink usando un passo fisso
passo=0.02;
t2_d=0:passo:50; % definisco il vettore dei tempi con il passo scelto
ex=sim("model_40_01_1.slx");
model = 'model_40_01_1.slx';
simIn = Simulink.SimulationInput(model);
simIn = simIn.setModelParameter('SolverType', 'Fixed-step');
simIn = simIn.setModelParameter('Solver', 'FixedStepAuto');
simIn = simIn.setModelParameter('FixedStep', 'passo');

% Eseguo la simulazione per vedere la differenza tra passo fisso e passo
% variabile
ex2 = sim(simIn);
figure('Name','Differenza passo fisso e variabile')
subplot(2, 1, 1);
plot(ex.tout, abs(rad2deg(squeeze(ex2.theta)-squeeze(ex.theta))));
xlabel('Tempo t [s]');
ylabel('Differenza tra gli angoli [deg]');
grid on;

subplot(2, 1, 2);
plot(ex.tout, abs(squeeze(ex2.x)-squeeze(ex.x)));
xlabel('Tempo t [s]');
ylabel('Differenza tra la posizione [m]');
grid on;

% differenza con ode23
ex=sim("model_40_01_1.slx");

model23 = 'model_40_01_1.slx';
% Creo un oggetto per rifare la simulazione
sim23 = Simulink.SimulationInput(model23);
% Imposto il solutore desiderato per questo caso
sim23 = sim23.setModelParameter('Solver', 'ode23');

% svolgo la simulazione
ex2 = sim(sim23); 

% rappresento le differenze
figure('Name','Differenza ode 23-45')
subplot(2, 1, 1);
plot(ex.tout, abs(rad2deg(squeeze(ex2.theta)-squeeze(ex.theta)))); % calcolo la differenza su theta tra  ode23 e ode 45 
xlabel('Tempo t [s]');
ylabel('Differenza tra gli angoli [deg]');
grid on;

subplot(2, 1, 2);
plot(ex.tout, abs(squeeze(ex2.x)-squeeze(ex.x))); % calcolo la differenza su x tra  ode23 e ode 45 
xlabel('Tempo t [s]');
ylabel('Differenza tra la posizione [m]');
grid on;

% 4.2
% rappresento la risposta del sistema linearizzato alle stesse condizioni 
% inziali del punto 2.1

% definisco un denominatore comune come nei calcoli svolti
den=(lab1P.I0*lab1P.I2 - lab1P.I1^2 + lab1P.I2*lab1P.M);

% definisco le matrici del sistema linearizzato
A=[ 0 1 0 0;
    0 -lab1P.c*lab1P.I2/den lab1P.I1^2*lab1P.g/den -lab1P.b*lab1P.I1/den;
    0 0 0 1;
    0 -lab1P.c*lab1P.I1/den lab1P.I1*lab1P.g*(lab1P.I0+lab1P.M)/den -lab1P.b*(lab1P.I0+lab1P.M)/den];

B=[ 0 0;
    lab1P.alpham*lab1P.I2/den (lab1P.alpha1*lab1P.I1-lab1P.alpha0*lab1P.I2)/den;
    0 0;
    lab1P.alpham*lab1P.I1/den (lab1P.alpha1*(lab1P.I0+lab1P.M)-lab1P.alpha0*lab1P.I1)/den];

C=eye(4); % ho tenuto 4 output, si potrebbe fare solo con 2 (in un analisi 
% preliminare è risultato interessante osservare anche le accelerazioni)

D=[ 0 0;  
    0 0;
    0 0;
    0 0];

% definisco il sistema usando il comando ss
sys=ss(A, B, C, D);

% impongo le condizioni iniziali
z0=[0; 0; deg2rad(1); 0];

% definisco il vettore dei tempi
t=linspace(0, 100, 1000);
t_f=t(end);

% creo la risposta alle condizioni inziali
[y, t_lin]=initial(sys, z0, t_f);

% rappresento la risposta alle condizioni iniziali
figure ('Name','risposta sistema linearizzato');

subplot(2,1,1); 
plot(t_lin, y(:,1),'LineWidth',2);
grid on;
xlabel('Tempo [s]');
ylabel('Posizione x [m]');
ylim([-1 100])

subplot(2,1,2);
plot(t_lin, y(:,3),'LineWidth',2);
grid on;
ylabel('Angolo \theta [rad]');
xlabel('Tempo [s]');
ylim([-1 100])

% confronto sistema non lineare con sistema linearizzato
% risulta più interessante per osservare il punto di divergenza
[y_lin, t_lin] = initial(sys, z0, 2.5); 

% rappresento la differenza lineare e non lineare
figure('Name','confronto lineare con non lineare'); 

% rappresento la posizione
subplot(2,1,1)
plot(t_ci, z(1,:), 'LineWidth', 1.5); hold on; % Non lineare
plot(t_lin, y_lin(:,1), '--', 'LineWidth', 1.5); % Lineare
grid on;
title('Confronto Posizione Carrello $x(t)$', 'Interpreter', 'latex');
ylabel('$x$ [m]', 'Interpreter', 'latex');
legend('Non Lineare', 'Lineare');
xlim([0 2.5]);
ylim([0 5]);

% rappresento l'angolo
subplot(2,1,2)
plot(t_ci, rad2deg(z(3,:)), 'LineWidth', 1.5); hold on; % Non lineare
plot(t_lin, rad2deg(y_lin(:,3)), '--', 'LineWidth', 1.5); % Lineare
grid on;
title('Confronto Angolo Pendolo $\theta(t)$', 'Interpreter', 'latex');
ylabel('$\theta$ [deg]', 'Interpreter', 'latex');
xlabel('Tempo [s]', 'Interpreter', 'latex');
legend('Non Lineare', 'Lineare');
xlim([0 2.5]);
ylim([-10 100]); 

% 4.3 MODELLO SENZA ATTRITO
% rappresento ora il modello senza attrito per poter calcolare le funzioni
% di trasferimento

% impongo b e c uguali a zero (assenza di attrito)
% rappresento le stesse matrici con i nuovi b e c
A_nof=[ 0 1 0 0;
        0 0 lab1P.I1^2*lab1P.g/den 0;
        0 0 0 1;
        0 0 lab1P.I1*lab1P.g*(lab1P.I0+lab1P.M)/den 0];

Bi_nof=[0; 
    lab1P.alpham*lab1P.I2/den; 
    0; 
    lab1P.alpham*lab1P.I1/den];

Bd_nof=[0; 
    (lab1P.alpha1*lab1P.I1-lab1P.alpha0*lab1P.I2)/den; 
    0; 
    (lab1P.alpha1*(lab1P.I0+lab1P.M)-lab1P.alpha0*lab1P.I1)/den];

B_nof=[Bi_nof Bd_nof];

% creo il nuovo sistema
sys_nof=ss(A_nof, B_nof, C, D);


% 4.4 FUNZIONI DI TRASFERIMENTO
% creo le funzioni di trasferimento che mi serviranno sia per il confronto
% con i risultati ottenuti a mano, che per la rappresentazione con pzmap

tf_sys=tf(sys_nof); 
% scelgo le funzioni di trasferimento di nostro interesse
tf_x_i=tf_sys(1,1);
tf_x_d=tf_sys(1,2);
tf_theta_i=tf_sys(3,1);
tf_theta_d=tf_sys(3,2);

% 4.5 MAPPA POLI ZERI
% rappresento la mappa dei poli e zeri per ognuna delle funzioni di
% trasferimento ottenute nel punto precedente

% creo la base della mappa
figure();
image1 = tiledlayout(2, 2);
title(image1, '\fontsize{20}Pole-Zero Map', 'FontWeight', 'bold');

% Grafico 2: x, i
nexttile;
[p1, z1] = pzmap(tf_x_i); 
pzmap(tf_x_i);            
sgrid;                    
hold on;
plot(real(z1), imag(z1), 'bo', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'w'); 
plot(real(p1), imag(p1), 'rx', 'MarkerSize', 12, 'LineWidth', 2); 
title('x, i');
hold off;

% Grafico 2: x, d 
nexttile;
[p2, z2] = pzmap(tf_x_d);
pzmap(tf_x_d);
sgrid;
hold on;
plot(real(z2), imag(z2), 'bo', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'w');
plot(real(p2), imag(p2), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
title('x, d');
hold off;

% Grafico 3: theta, i 
nexttile;
[p3, z3] = pzmap(tf_theta_i);
pzmap(tf_theta_i);
sgrid;
hold on;
plot(real(z3), imag(z3), 'bo', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'w');
plot(real(p3), imag(p3), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
title('\theta, i');
hold off;

% Grafico 4: theta, d 
nexttile;
[p4, z4] = pzmap(tf_theta_d);
pzmap(tf_theta_d);
sgrid;
hold on;
plot(real(z4), imag(z4), 'bo', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'w');
plot(real(p4), imag(p4), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
title('\theta, d');
hold off;

% 5.2-5 CONTROLLO
% creo inzialmente il controllore PD come da task 5.4

% PD Controller R = kd + S*kp

% controllore
% definisco i valori scelti nel punto 5.4
kp=7.76;
kd=0.985;

% creo il controllore
K_PD=pid(kp, 0, kd);

% definisco la funzione di trasferimeto come nel punto 5.2
Gc_PD = tf_theta_d * feedback(1, tf_theta_i * K_PD);
Gc_PD = minreal(Gc_PD);

% definisco vettore dei tempi
t=linspace(0, 10, 1000);

% definisco disturbo costante
dis=1*ones(size(t)); 

% simulo la risposta del sistema lineare a un disturbo costante
[theta_pd, t_pd] = lsim(Gc_PD, dis, t);
theta_pd = rad2deg(theta_pd);

% rappresento l'angolo
figure('Name', 'Angolo a disturbo costante con controllo PD');

plot(t_pd, theta_pd, 'b', 'LineWidth', 2);
title('Risposta del Pendolo \theta(t)');
xlabel('Tempo [s]');
ylabel('Angolo [gradi]');
grid on;

% 5.8-10
% creo il controllore pid come da task 5.10

% PID Controller R = kd + S*kp + ki/S
% definisco il termine ki del controllore PID
ki=2;
% creo il controllore
K_PID=pid(kp, ki, kd);

% rappresento la funzione di trasferimento in anello chiuso usando un PID
Gc_PID = tf_theta_d * feedback(1, tf_theta_i * K_PID);
Gc_PID = minreal(Gc_PID);

% simulo la risposta del sistema lineare a un disturbo costante
[theta_pid, t_pid] = lsim(Gc_PID, dis, t);
theta_pid = rad2deg(theta_pid);

% rappresento nuovamente l'angolo theta
figure('Name', 'Angolo a disturbo costante con controllo PID');

plot(t_pid, theta_pid, 'b', 'LineWidth', 2);
title('Risposta del Pendolo \theta(t)');
xlabel('Tempo [s]');
ylabel('Angolo [gradi]');
grid on;

% impongo il disturbo variabile
% inizializzo il vettore disturbo

dis=zeros(size(t));

% disturbo variabile
for j=1:length(t)

   dis(j)=lab1P_input(t(j), lab1P);
   
end

% 5.6 
% rappresento la risposta del sistema non linearizzato senza attriti 
% con il controllore PD

% definisco le condizioni iniziali
z0=[0; 0; 0; 0];
% definisco il vettore dei tempi
t=linspace(0, 10, 1000);
t_0=t(1);
t_f=t(end);

% risolvo il sistema con una ode
ODE_obj=ode;
ODE_obj.ODEFcn = @(t, z) lab1P_fcontrollo_noatt(t, z, [kp, kd], @lab1P_input, lab1P); 
ODE_obj.InitialValue=z0;
ODE_obj.Solver='ode45';

ODEResults_obj=solve(ODE_obj, t_0, t_f);

% ottengo i risultati
t_pd=ODEResults_obj.Time;
z_pd=ODEResults_obj.Solution;

% definisco il valore della corrente antitrasformando 
ii_pd=-kp*z_pd(3,:)-kd*z_pd(4, :);

% rappresento d, x, theta, i
figure('Name', 'Sistema non lineare senza attrito con controllo PD');
subplot(2,2,1);
plot(t, dis, 'r', 'LineWidth', 1.5);
title('Input Costante: Disturbo d(t)');
ylabel('Ampiezza [N]');
grid on;

subplot(2,2,2);
plot(t_pd, rad2deg(z_pd(3, :)), 'b', 'LineWidth', 2);
title('Risposta del Pendolo \theta(t)');
xlabel('Tempo [s]');
ylabel('Angolo [gradi]');
grid on;

subplot(2,2,3);
plot(t_pd, ii_pd,'b', 'LineWidth', 2);
title('Risposta del motore I(t)');
xlabel('Tempo [s]');
ylabel('Corrente');
grid on;

subplot(2,2,4)
plot(t_pd, z_pd(1, :),'b', 'LineWidth', 2);
title('Spostamento del carrello x(t)');
xlabel('Tempo [s]');
ylabel('Posizione [m]');
grid on;

% 5.7 
% rappresemto il sistema non linearizzato con attriti sottoposto a un
% controllore PID

% risolvo l'Ode nel caso degli attriti
ODE_obj=ode;
ODE_obj.ODEFcn = @(t, z) lab1P_fcontrollo(t, z, [kp, kd], @lab1P_input, lab1P); 
ODE_obj.InitialValue=z0;
ODE_obj.Solver='ode45';

ODEResults_obj=solve(ODE_obj, t_0, t_f);
t_pd=ODEResults_obj.Time;
z_pd=ODEResults_obj.Solution;

% definisco la corrente come prima
ii_pd=-kp*z_pd(3,:)-kd*z_pd(4, :);

% rappresento d, x, theta, i
figure('Name', 'Sistema non lineare con attrito con controllo PD');
subplot(2,2,1);
plot(t, dis, 'r', 'LineWidth', 1.5);
title('Input Costante: Disturbo d(t)');
ylabel('Ampiezza [N]');
grid on;

subplot(2,2,2);
plot(t_pd, rad2deg(z_pd(3, :)), 'b', 'LineWidth', 2);
title('Risposta del Pendolo \theta(t)');
xlabel('Tempo [s]');
ylabel('Angolo [gradi]');
grid on;

subplot(2,2,3);
plot(t_pd, ii_pd,'b', 'LineWidth', 2);
title('Risposta del motore I(t)');
xlabel('Tempo [s]');
ylabel('Corrente');
grid on;

subplot(2,2,4)
plot(t_pd, z_pd(1, :),'b', 'LineWidth', 2);
title('Spostamento del carrello x(t)');
xlabel('Tempo [s]');
ylabel('Posizione [m]');
grid on;

% Task 5.11
% svolgo un analisi comparativa al variare di ogni k

% Definiamo i tre valori di Kp da testare
kp_values = [4, 7.76, 25]; 
colori = ['r', 'b', 'g']; 

figure('Name', 'Analisi Sensibilità Guadagno Integrale Kp', 'NumberTitle', 'off');

for j = 1:length(kp_values)
    % Assegniamo il valore di Kp corrente
    kp = kp_values(j);
    
    % Manteniamo ki e kd fissi su valori stabili 
    ki = 2; 
    kd = 0.985; 
    
    % Esecuzione della simulazione Simulink
    sim_data = sim("Model_40_01_2.slx");
    
    % rappresento theta
    subplot(2,1,1);
    plot(sim_data.tout, rad2deg(sim_data.theta_pid), colori(j), 'LineWidth', 2);
    hold on;
    title('Effetto di K_p sull''angolo \theta(t)');
    ylabel('Angolo [gradi]');
    grid on;
    
    % rappresento I(t) 
    subplot(2,1,2);
    plot(sim_data.tout, sim_data.i_pid, colori(j), 'LineWidth', 2);
    hold on;
    title('Effetto di K_p sulla Corrente I(t)');
    xlabel('Tempo [s]');
    ylabel('Corrente');
    grid on;
end

% legenda
subplot(2,1,1);
legend(['Kp = ', num2str(kp_values(1))], ['Kp = ', num2str(kp_values(2))], ['Kp = ', num2str(kp_values(3))]);

subplot(2,1,2);
legend(['Kp = ', num2str(kp_values(1))], ['Kp = ', num2str(kp_values(2))], ['Kp = ', num2str(kp_values(3))]);

% Definiamo i tre valori di Ki da testare
ki_values = [1, 5, 20 ]; 
colori = ['r', 'b', 'g']; 

figure('Name', 'Analisi Sensibilità Guadagno Integrale Ki', 'NumberTitle', 'off');

for j = 1:length(ki_values)
    % Assegniamo il valore di Ki corrente
    ki = ki_values(j);
    
    % Manteniamo kp e kd fissi su valori stabili 
    kp = 7.76; 
    kd = 0.985; 
    
    % Esecuzione della simulazione Simulink
    sim_data = sim("Model_40_01_2.slx");
    
    % rappresento theta(t) 
    subplot(2,1,1);
    plot(sim_data.tout, rad2deg(sim_data.theta_pid), colori(j), 'LineWidth', 2);
    hold on;
    title('Effetto di K_i sull''angolo \theta(t)');
    ylabel('Angolo [gradi]');
    grid on;
    
    % rappresento I(t) 
    subplot(2,1,2);
    plot(sim_data.tout, sim_data.i_pid, colori(j), 'LineWidth', 2);
    hold on;
    title('Effetto di K_i sulla Corrente I(t)');
    xlabel('Tempo [s]');
    ylabel('Corrente');
    grid on;
end

% legenda
subplot(2,1,1);
legend(['Ki = ', num2str(ki_values(1))], ['Ki = ', num2str(ki_values(2))], ['Ki = ', num2str(ki_values(3))]);

subplot(2,1,2);
legend(['Ki = ', num2str(ki_values(1))], ['Ki = ', num2str(ki_values(2))], ['Ki = ', num2str(ki_values(3))]);

% Definiamo i tre valori di Kd da testare
kd_values = [0.1, 0.985, 20 ]; 
colori = ['r', 'b', 'g']; 

figure('Name', 'Analisi Sensibilità Guadagno Integrale Kd', 'NumberTitle', 'off');

for j = 1:length(ki_values)
    % Assegniamo il valore di Kd corrente
    kd = kd_values(j);
    
    % Manteniamo kp e ki fissi su valori stabili
    kp = 7.76; 
    ki = 1; 
    
    % Esecuzione della simulazione Simulink
    sim_data = sim("Model_40_01_2.slx");
    
    % rappresento theta(t)
    subplot(2,1,1);
    plot(sim_data.tout, rad2deg(sim_data.theta_pid), colori(j), 'LineWidth', 2);
    hold on;
    title('Effetto di K_d sull''angolo \theta(t)');
    ylabel('Angolo [gradi]');
    grid on;
    
    % rappresento I(t) 
    subplot(2,1,2);
    plot(sim_data.tout, sim_data.i_pid, colori(j), 'LineWidth', 2);
    hold on;
    title('Effetto di K_d sulla Corrente I(t)');
    xlabel('Tempo [s]');
    ylabel('Corrente');
    grid on;
end

% legenda
subplot(2,1,1);
legend(['Kd = ', num2str(kd_values(1))], ['Kd = ', num2str(kd_values(2))], ['Kd = ', num2str(kd_values(3))]);

subplot(2,1,2);
legend(['Kd = ', num2str(kd_values(1))], ['Kd = ', num2str(kd_values(2))], ['Kd = ', num2str(kd_values(3))]);


% 6.1 
den=(lab1P.I0*lab1P.I2 - lab1P.I1^2 + lab1P.I2*lab1P.M);

% definisco le matrici del sistema linearizzato senza attriti, come fatto
% nel punto 4.3 
A_nof=[ 0 1 0 0;
        0 0 lab1P.I1^2*lab1P.g/den 0;
        0 0 0 1;
        0 0 lab1P.I1*lab1P.g*(lab1P.I0+lab1P.M)/den 0];
% separo la matrice B del task 4.3 in due vettori, uno che riguarda la dinamica della
% corrente senza attriti, l'altro che descrive il disturbo.
Bi_nof=[0; 
    lab1P.alpham*lab1P.I2/den; 
    0; 
    lab1P.alpham*lab1P.I1/den];

Bd_nof=[0; 
    (lab1P.alpha1*lab1P.I1-lab1P.alpha0*lab1P.I2)/den; 
    0; 
    (lab1P.alpha1*(lab1P.I0+lab1P.M)-lab1P.alpha0*lab1P.I1)/den];

% verifico la controllabilità costurendo la matrice Co di controllabilità e
% confrontando il suo rango con la dimensione delle righe della matrice A_nof 
Co=ctrb(A_nof,Bi_nof);
n=size(A_nof,1);
isControllable= (rank(Co)==n);

% inizio la scelta dei poli partendo dai due dominanti, imponendo i valori
% di smorzamento e omega_n ipotizzati adeguati per soddisfare le
% caratteristiche adeguate
xi=0.8;
omega_n=2;
omega_d=omega_n*sqrt(1-xi^2);
Re_pC=xi*omega_n;
Im_pC=omega_d;
% per i primi due poli uso l'approssimazione del secondo ordine 
pC_1=-Re_pC+1i*Im_pC;
pC_2=-Re_pC- 1i*Im_pC;
% scelgo gli altri due lontani da quelli dominanti e a parte reale negativa
% cosi ottengo il vettore dei poli
pC=[pC_1 pC_2 -5 -8];

K=place(A_nof,Bi_nof,pC);% ottengo la matrice dei guadagni tramite place
% verifico che gli autovalori di Ac siano uguali ai poli scelti 
Ac=A_nof-Bi_nof*K;
eig(Ac);

% 6.3 
% verifico che sia osservabile
Cy=[1 0 0 0;
    0 0 1 0]; 
% poichè posso misurare solo posizione e angolo Cy ha due righe 

Ob = obsv(A_nof, Cy); %costruisco la matrice di osservabilità
n = size(A_nof, 1);% salvo la dimensione delle righe di A
isObservable = (rank(Ob) == n);% confronto se il rango di Ob coincide con il numero di righe di A

% scelgo i poli 2-6 volte più veloci di quelli del controllore. Per
% scegliere quelli dominanti ipotizzo dei valori di smorzamento e
% pulsazione adeguati
xi_0 = 0.5;
omega_0 = 5;
Re_p0 = xi_0*omega_0;
Im_p0 = omega_0*sqrt(1-xi_0^2);
p0_1 = -Re_p0 + 1i*Im_p0;
p0_2 = -Re_p0 - 1i*Im_p0;
p0 = [-24 -15 p0_2 p0_1];

% Pole placement (observer)

LT = place(A_nof', Cy', p0);
L=LT';%uso il trasposto per la far tornare le dimensioni nel prodotto L*Cy 
AO=A_nof-L*Cy;
eig(AO); % verifico che gli autovalori di Ao siano uguali ai poli dell'osservatore



% 6.5
% dall' equazione in forma matriciale ottengo le matrici del sistema
%aumentato
Atot = [A_nof,        -Bi_nof*K; 
        L*Cy,  A_nof - L*Cy - Bi_nof*K];

% Matrice di ingresso per il disturbo (8x1)
Btot = [Bd_nof; 
        zeros(4,1)];

% Matrice di uscita (misuriamo x e theta reali)
Ctot = [Cy, zeros(2,4)];

% Creazione del sistema State-Space
sys_comp = ss(Atot, Btot, Ctot, 0);
lab1P.K = K;       % Matrice del controllore
lab1P.Atot = Atot;
x0_real = [0; 0; 0; 0];
x0_stim = [0; 0; 0; 0]; % condizione iniziale nulla per vedere il comportamento dell'osservatore
z0_aug = [x0_real; x0_stim];

% Tempi di simulazione
t_0 = 0;
t_f = 50;

% Risoluzione ODE 

ODE_obj = ode;% Creazione dell'oggetto

ODE_obj.ODEFcn = @(t, z) lab1P_comp_nonlinear(t, z, @lab1P_input, lab1P); %definisco la funzione che devo integrare, che riceve in input 
% tempo, stato, la funzione descrivente il disturbo e i paremetri


ODE_obj.InitialValue = z0_aug; %scelgo la condizione iniziale
ODE_obj.Solver = 'ode45'; %scelgo il tipo di solver


% ODE_obj.AbsoluteTolerance = 1e-12;
% ODE_obj.RelativeTolerance = 1e-6;

ODEResults_obj = solve(ODE_obj, t_0, t_f); % risolvo dal tempo iniziale al tempo finale 


t_out = ODEResults_obj.Time'; % salvo il vettore trasposto del tempo impiegato nella risoluzione
z_out = ODEResults_obj.Solution'; % salvo la matrice trasposta dei risultati contenente tutte le componenti


x_real = z_out(:, 1:4); % salvo in una matrice le prime quattro righe dei risultati reali
x_cap  = z_out(:, 5:8); % salvo in una matrice le ultime quattro righe dei risultati stimati



figure('Name', ' Reale vs Stimato');

%  Plot della posizione del carrello x(t)
subplot(2,1,1);
plot(t_out, x_real(:,1), 'b', 'LineWidth', 2); 
hold on;
plot(t_out, x_cap(:,1), 'r--', 'LineWidth', 2); 
grid on;
title('Posizione Carrello ');
ylabel('x [m]');
xlabel('Tempo [s]');

legend('Reale ', 'Stimata ', 'Location', 'Best');

% Plot dell'angolo del pendolo theta(t)
subplot(2,1,2);
plot(t_out, rad2deg(x_real(:,3)), 'b', 'LineWidth', 2); 
hold on;
plot(t_out, rad2deg(x_cap(:,3)), 'r--', 'LineWidth', 2); 
grid on;
title('theta');
ylabel('\theta [deg]'); 
xlabel('Tempo [s]');
legend('Reale ', 'Stimata ', 'Location', 'Best');

% plot della corrente i(t)

i_t = -x_cap * K'; % uso il vettore stima per calcolare a posteriori la corrente, 
% poichè è l'unico che conosco interamente (dimensione N x 4),
% moltiplicato per la matrice dei guadagni trasposta (4 x 1)


figure('Name', ' Corrente di Controllo i(t)');
plot(t_out, i_t, 'g', 'LineWidth', 2);
grid on;

title('Sforzo di Controllo: Corrente al Motore i(t)');
ylabel('Corrente [A]');
xlabel('Tempo [s]');

% 6.6 Risoluzione su simulink

% faccio girare il modello su simulink
ex2=sim("model_40_01_3.slx");

% plot
figure();

plot(ex2.tout,squeeze(ex2.x_r));
xlabel('t [s]');
ylabel('x [m]');
grid on, box on, hold on
plot(ex2.tout,squeeze(ex2.x_s),'r--');
legend('x_real','x_sim');
title('andamento di x_real e x_sim')
figure ();
plot(ex2.tout,squeeze(rad2deg(ex2.theta_r)));
xlabel('t [s]');
ylabel('theta [deg]');
grid on, box on, hold on
plot(ex2.tout,squeeze(rad2deg(ex2.theta_s)),'r--');
legend('theta_real','theta_sim');
title('andamento di theta_real e theta_sim')

figure()

x_cap_sim =          ([squeeze(ex2.x_s)'; ...
                     squeeze(ex2.x_s_dot)'; ...
                     squeeze(ex2.theta_s)'; ...
                     squeeze(ex2.theta_s_dot)']);% salvo le componenti stimate in un' unica matrice trasposta 4 x N

i_t = -K*x_cap_sim ; % pre moltiplico per K (1 x 4) e ottengo i_t 1 x N
plot(ex2.tout,i_t,'LineWidth',2);
title('andamento di i(t)')
xlabel('t[s]')
ylabel('i[A]')
grid on
xlim([0 15])% limito l'asse x per avere maggiore zoom nella regione del transitorio


%% FUNCTIONS 

function dz=lab1P_f1(z, lab1P)
% lab1P_f1 Calcola la derivata del vettore di stato per il sistema non
% lineare quando esso è sottoposto a disturbo costante

% Il compito di questa funzione è fornire il valore della derivata nel tempo
% del vettore di stato z (le 4 variabili di stato del sistema) ad ogni 
% istante di tempo in cui viene chiamata dal solver ODE.

% INPUT
% Name      Type        Size
% z         vector      4x1
% lab1P     struct      -
%
% OUTPUT
% Name      Type        Size
% dz        vector      4x1

% disturbo costante
d=0;

% no controllo, corrente nulla
ii=0;

% sistema del punto 1.3
den=(lab1P.I0*lab1P.I2 - lab1P.I1^2*cos(z(3))^2 + lab1P.I2*lab1P.M);
dz=[z(2);
    -(- lab1P.g*sin(z(3))*lab1P.I1^2*cos(z(3)) + lab1P.I2*sin(z(3))*lab1P.I1*z(4)^2 + lab1P.b*lab1P.I1*z(4)*cos(z(3)) - lab1P.alpha1*d*lab1P.I1*cos(z(3))^2 + lab1P.I2*lab1P.alpha0*d - lab1P.I2*lab1P.alpham*ii + lab1P.I2*lab1P.c*z(2))/den;
    z(4);
    -(lab1P.I0*lab1P.b*z(4) + lab1P.M*lab1P.b*z(4) - lab1P.I0*lab1P.I1*lab1P.g*sin(z(3)) - lab1P.I1*lab1P.M*lab1P.g*sin(z(3)) - lab1P.I0*lab1P.alpha1*d*cos(z(3)) + lab1P.I1*lab1P.alpha0*d*cos(z(3)) - lab1P.M*lab1P.alpha1*d*cos(z(3)) - lab1P.I1*lab1P.alpham*ii*cos(z(3)) + lab1P.I1*lab1P.c*z(2)*cos(z(3)) + lab1P.I1^2*z(4)^2*cos(z(3))*sin(z(3)))/den];


end

function dz=lab1P_f2(t, z, input_fun, lab1P)
% lab1P_f2 Calcola la derivata del vettore di stato per il sistema non
% lineare quando esso è sottoposto a disturbo variabile
% 
% Fornisce il valore della derivata nel tempo del vettore di stato z 
% ad ogni istante t valutato dal solver ODE. A differenza del caso base,
% valuta dinamicamente un disturbo dipendente dal tempo passatogli tramite 
% l'handle di funzione input_fun.
%
% INPUT
% Name          Type                Size
% t             scalar              1x1
% z             vector              4x1
% input_fun     function_handle     -
% lab1P         struct              -
%
% OUTPUT
% Name          Type                Size
% dz            vector              4x1


% definisco disturbo 
d=input_fun(t, lab1P);

% no controllo
ii=0;

% sistema del punto 1.3
den=(lab1P.I0*lab1P.I2 - lab1P.I1^2*cos(z(3))^2 + lab1P.I2*lab1P.M);
dz=[z(2);
    -(- lab1P.g*sin(z(3))*lab1P.I1^2*cos(z(3)) + lab1P.I2*sin(z(3))*lab1P.I1*z(4)^2 + lab1P.b*lab1P.I1*z(4)*cos(z(3)) - lab1P.alpha1*d*lab1P.I1*cos(z(3))^2 + lab1P.I2*lab1P.alpha0*d - lab1P.I2*lab1P.alpham*ii + lab1P.I2*lab1P.c*z(2))/den;
    z(4);
    -(lab1P.I0*lab1P.b*z(4) + lab1P.M*lab1P.b*z(4) - lab1P.I0*lab1P.I1*lab1P.g*sin(z(3)) - lab1P.I1*lab1P.M*lab1P.g*sin(z(3)) - lab1P.I0*lab1P.alpha1*d*cos(z(3)) + lab1P.I1*lab1P.alpha0*d*cos(z(3)) - lab1P.M*lab1P.alpha1*d*cos(z(3)) - lab1P.I1*lab1P.alpham*ii*cos(z(3)) + lab1P.I1*lab1P.c*z(2)*cos(z(3)) + lab1P.I1^2*z(4)^2*cos(z(3))*sin(z(3)))/den];

end

function u=lab1P_input(t, lab1P) 
% lab1P_input Definisce l'andamento del disturbo d
%
% Genera un segnale a gradino (o impulso rettangolare) che assume il 
% valore d_max nell'intervallo di tempo [T1, T2) e valore nullo altrove.
%
% INPUT
% Name          Type        Size
% t             scalar      1x1
% lab1P         struct      -
%
% OUTPUT
% Name          Type        Size
% u             scalar      1x1
 
if t<lab1P.T1
    u=0;
elseif t<lab1P.T2
        u=lab1P.d_max;
else 
    u=0;
end

end

function lab1P=lab1P_parameters()
% lab1P_parameters Inizializza i parametri fisici e geometrici del sistema.
% 
% La funzione definisce i parametri costanti del sistema (massa, densità, 
% lunghezze, costanti di attrito) e calcola i parametri derivati necessari 
% per le equazioni della dinamica, restituendoli all'interno di una struct.
%
% INPUT
% Name      Type        Size
% -         -           -
%
% OUTPUT
% Name      Type        Size
% lab1P     struct      -

% parametri con densità lineare costante e disturbo costante nello spazio
M = 0.57;
rho = 0.36;
l = 0.64;
g = 9.81;
c = 0.10;
b = 0.005;
r = 0.02;
kt = 0.05;
d0 = 0.30;
I0=rho*l;
I1=1/2 * rho * l^2;
I2=1/3 * rho * l^3;
alpha0=d0*l;
alpha1=1/2 * d0 * l^2;
T1=1;
T2=3;
d_max=1;

lab1P.M=M;
lab1P.rho=rho;
lab1P.l=l;
lab1P.g=g;
lab1P.c=c;
lab1P.b=b;
lab1P.r=r;
lab1P.kt=kt;
lab1P.d0=d0;
lab1P.I0=I0;
lab1P.I1=I1;
lab1P.I2=I2;
lab1P.alpha0=alpha0;
lab1P.alpha1=alpha1;
lab1P.alpham=kt/r;
lab1P.T1=T1;
lab1P.T2=T2;
lab1P.d_max=d_max;
     

end

  
function dz = lab1P_comp_nonlinear(t, z, input_fun, lab1P)

% lab1P_comp_nonlinear Calcola la derivata dello stato aumentato (reale + stimato).
% 
% Simula la dinamica non lineare del sistema senza attriti in anello chiuso. 
% Riceve in ingresso un vettore di stato aumentato z (8x1), in cui la prima metà 
% rappresenta lo stato reale dell'impianto (x, x_dot, theta, theta_dot) e
% la seconda metà lo stato stimato (x_cap, x_cap_dot, theta_cap, theta_cap_dot)
% dall'osservatore. 
%
% INPUT
% Name          Type                Size
% t             scalar              1x1
% z             vector              8x1     
% input_fun     function_handle     -
% lab1P         struct              -
%
% OUTPUT
% Name          Type                Size
% dz            vector              8x1 

%  Valutazione del disturbo d(t)
d = input_fun(t, lab1P);

% Il controllore calcola la corrente 'ii' non sullo stato reale, 
% ma sulla stima dell'osservatore.
ii = - lab1P.K * z(5:8);
 
den = (lab1P.I0*lab1P.I2 - lab1P.I1^2*cos(z(3))^2 + lab1P.I2*lab1P.M);
   
dz_reale = [
    z(2);  % ricordando che z_real contiene le velocità di x alla riga 2 e di
% theta alla riga 4, derivo lo stato z in riga due e ottengo x_dot_dot
    
    -(- lab1P.g*sin(z(3))*lab1P.I1^2*cos(z(3)) + lab1P.I2*sin(z(3))*lab1P.I1*z(4)^2 + lab1P.b*lab1P.I1*z(4)*cos(z(3)) - lab1P.alpha1*d*lab1P.I1*cos(z(3))^2 + lab1P.I2*lab1P.alpha0*d - lab1P.I2*lab1P.alpham*ii + lab1P.I2*lab1P.c*z(2))/den;
        
    z(4); % derivando lo stato z in riga 4 ottengo theta_dot_dot
        
        -(lab1P.I0*lab1P.b*z(4) + lab1P.M*lab1P.b*z(4) - lab1P.I0*lab1P.I1*lab1P.g*sin(z(3)) - lab1P.I1*lab1P.M*lab1P.g*sin(z(3)) - lab1P.I0*lab1P.alpha1*d*cos(z(3)) + lab1P.I1*lab1P.alpha0*d*cos(z(3)) - lab1P.M*lab1P.alpha1*d*cos(z(3)) - lab1P.I1*lab1P.alpham*ii*cos(z(3)) + lab1P.I1*lab1P.c*z(2)*cos(z(3)) + lab1P.I1^2*z(4)^2*cos(z(3))*sin(z(3)))/den
    ];

   
dz_stimato = lab1P.Atot(5:8, :) * z;

% ottengo il vettore dz totale concatenando i dz real e dz stim
dz = [dz_reale; dz_stimato];

end

function dz=lab1P_fcontrollo_noatt(t, z, K, input_fun, lab1P)

% lab1P_fcontrollo_noatt si comporta come lab1P_f2 ma questa volta è
% presente l'azione di controllo i(t)
% 
% Calcola la derivata del vettore di stato valutando il sistema controllato 
% tramite una legge di retroazione di stato parziale (sugli stati z(3) e z(4))
% considerando nulli gli attriti.
%
% INPUT
% Name          Type                Size
% t             scalar              1x1
% z             vector              4x1
% K             vector              1x2     
% input_fun     function_handle     -
% lab1P         struct              -
%
% OUTPUT
% Name          Type                Size
% dz            vector              4x1


% disturbo variabile
d=input_fun(t, lab1P);

% controllo
ii=-K(1)*z(3)-K(2)*z(4);

% sistema del punto 1.3
den=(lab1P.I0*lab1P.I2 - lab1P.I1^2*cos(z(3))^2 + lab1P.I2*lab1P.M);
dz=[z(2);
    -(- lab1P.g*sin(z(3))*lab1P.I1^2*cos(z(3)) + lab1P.I2*sin(z(3))*lab1P.I1*z(4)^2  - lab1P.alpha1*d*lab1P.I1*cos(z(3))^2 + lab1P.I2*lab1P.alpha0*d - lab1P.I2*lab1P.alpham*ii)/den;
    z(4);
    -(- lab1P.I0*lab1P.I1*lab1P.g*sin(z(3)) - lab1P.I1*lab1P.M*lab1P.g*sin(z(3)) - lab1P.I0*lab1P.alpha1*d*cos(z(3)) + lab1P.I1*lab1P.alpha0*d*cos(z(3)) - lab1P.M*lab1P.alpha1*d*cos(z(3)) - lab1P.I1*lab1P.alpham*ii*cos(z(3)) + lab1P.I1^2*z(4)^2*cos(z(3))*sin(z(3)))/den];

end

function dz=lab1P_fcontrollo(t, z, K, input_fun, lab1P)

% lab1P_fcontrollo si comporta come lab1P_f2 ma questa volta è
% presente l'azione di controllo i(t)
% 
% Calcola la derivata del vettore di stato valutando il sistema controllato 
% tramite una legge di retroazione di stato parziale (sugli stati z(3) e z(4))
% considerando la presenza di attriti.
%
% INPUT
% Name          Type                Size
% t             scalar              1x1
% z             vector              4x1
% K             vector              1x2     
% input_fun     function_handle     -
% lab1P         struct              -
%
% OUTPUT
% Name          Type                Size
% dz            vector              4x1

% disturbo variabile
d=input_fun(t, lab1P);

% controllo
ii=-K(1)*z(3)-K(2)*z(4);

% sistema del punto 1.3
den=(lab1P.I0*lab1P.I2 - lab1P.I1^2*cos(z(3))^2 + lab1P.I2*lab1P.M);
dz=[z(2);
    -(- lab1P.g*sin(z(3))*lab1P.I1^2*cos(z(3)) + lab1P.I2*sin(z(3))*lab1P.I1*z(4)^2 + lab1P.b*lab1P.I1*z(4)*cos(z(3)) - lab1P.alpha1*d*lab1P.I1*cos(z(3))^2 + lab1P.I2*lab1P.alpha0*d - lab1P.I2*lab1P.alpham*ii + lab1P.I2*lab1P.c*z(2))/den;
    z(4);
    -(lab1P.I0*lab1P.b*z(4) + lab1P.M*lab1P.b*z(4) - lab1P.I0*lab1P.I1*lab1P.g*sin(z(3)) - lab1P.I1*lab1P.M*lab1P.g*sin(z(3)) - lab1P.I0*lab1P.alpha1*d*cos(z(3)) + lab1P.I1*lab1P.alpha0*d*cos(z(3)) - lab1P.M*lab1P.alpha1*d*cos(z(3)) - lab1P.I1*lab1P.alpham*ii*cos(z(3)) + lab1P.I1*lab1P.c*z(2)*cos(z(3)) + lab1P.I1^2*z(4)^2*cos(z(3))*sin(z(3)))/den];


end
toc