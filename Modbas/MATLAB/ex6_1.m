%% MATLAB file zu Aufgabe 6.1 und 7.1
clear all;

%% Parameter
% Streckenparameter
Tt = 100e-3;            % Totzeit [s]
T = 316e-3;             % Zeitkonstante der Strecke [s]
k = 2.51;               % Verstärkung der Strecke [m/s]

% Anforderungen an den Geschwindigkeitsregler
phires_soll = 65;       % Phasenrand [°]
Tm_soll = 1;            % Überschwingzeit [s]
omegaD = pi/Tm_soll;    % Durchtrittskreisfrequenz [rad/s]

% Geschwindigkeitsreglerparameter
Ti = tan(phires_soll/180*pi + omegaD*Tt + atan(-1/omegaD/T) - 2*pi)/omegaD;
                        % Zeitkonstante des Geschwindigkeitsreglers [s]
kr = Ti/k * sqrt(omegaD^2 + omegaD^4 * T^2) / sqrt(Ti^2*omegaD^2 + 1);
                        % Verstärung des Geschwindigkeitsreglers [s/m]
% Positionsreglerparameter                        
kp = 1;                 % Verstärkung des Positionsregler [1/s]  
% Abtastung
tA = 20e-3;

%% Übertragungsfunktionen
s = tf('s');
% Geschwindigkeitsregler
G_S = tf(k, [T, 1], 'Inputdelay', Tt);  % Strecke
G_R = kr*(Ti*s+1)/Ti/s;                 % Regler
G_0 = G_S*G_R;                          % offene Strecke
G_W = minreal(G_0/(1+G_0));             % Führungsübertragung
% Approximierter Geschwindigkeitsregler
G_S_ap = tf(k,[T*Tt, (T+Tt), 1]);       % Strecke
G_R_ap = G_R;                           % Regler
G_0_ap = minreal(G_S_ap*G_R_ap);        % offene Strecke
G_W_ap = minreal(G_0_ap/(1+G_0_ap));    % Führungsübertragung
% Positionsregler
G_Sp = minreal(G_W_ap/s);               % Strecke
G_Rp = tf(kp);                          % Regler
G_0p = minreal(G_Sp*G_Rp);              % offene Strecke
G_Wp = minreal(G_0p/(1+G_0p));          % Führungsübertragung
% Vorsteuerung des Positionreglers
G_Vp1 = tf( [1, 0], [Ti, 1]);           % Online-Vorsteuerung
G_V = minreal(G_Vp1*G_Sp/(1+G_0p));     % Vorsteuerung

%% Bodediagramm der offenen Strecke Geschwindigkeitsregler
figure(1);
clf;
margin(G_0);            % Bodediagramm mit Phasen- und Amplitudenrand
grid on;
%% Sprungantwort Führungsverhalten Geschwindigkeitsregler
figure(2);
clf;
step(G_W), grid on;  
title('Sprungantwort des Geschwindigkeitsreglers');
xlabel('t ');
ylabel('v (meter/second)');
hold on;
% Analyse der Sprungantwort
si = stepinfo(G_W);
Tm = si.PeakTime       % Überschwingzeit [s]
em = si.Overshoot      % Überschwingweite [%]
plot(Tm, 1 + em / 100, 'kx', 'MarkerSize', 8);
legend('v(t)', 'peak');
%% Rampenantwort Positionsregler
v_s = 0.1;              % Soll Geschwindigkeit [m/s]
tsim = 0:0.01:10;       % Zeitintervall für die Rampe
u = v_s*tsim;           % Definition der Rampe als Eingangssignal

% Plot der Position y im Vergleich zum Eingangssignal u
figure(3);
clf;
lsim(G_Wp, u, tsim); grid on;
title('Rampenantwort des Positionsreglers');
xlabel('t ');
ylabel('yp (meter)');
legend('yp(t)');

%% Stabilität
figure(4);
clf;
rlocus(G_0p); grid on;
title('Wurzelortskurven');
% Berechnung der maximalen Verstärkung kp, sodass G_Wp noch stabil ist
[r,gain] = rlocus(G_0p);
for i = 1:length(gain)
    if real(r(3,i)) > 0 % Prüfen des Realteils der Polstelle
       kp1 = gain(i-1); % Untere Grenze
       kp2 = gain(i);   % Obere Grenze
       break;
    end 
end
% Erneute Berechnung der maximalen Verstärkung
kp12 = kp1:0.01:kp2;    % Abtastung 0.01
r1 = rlocus(G_0p, kp12);
for i = 1:length(kp12)
    if real(r1(2,i)) > 0 % Prüfen des Realteils der Polstelle
       kpmax = kp12(i-1) % Maximale Verstärkung kpmax [1/s]
       break;
    end 
end
%% Generierung des Führungssignals und Vorsteuersignal des Positionsreglers
vmax = 0.5;     % Maximale Soll-Geschwindigkeit v* [m/s]
x0 = 0;         % Startposition [m]
xs = 1;         % Soll-Bogenlänge x* [m]

% Berechnung der Koeffizienten c für das Führungssignal wp und
% der Endzeit te
[c,te] = cd_refpoly_vmax(vmax, x0, xs);
% Berechnung der Koeffizienten cff für die Online-Vorsteuerung uvp1
cff = cd_refpoly_ff(c, k, T, Tt, kr, Ti);

t = 0:tA:te;    % Definiton des Zeitintervalls

% Bestimmung der Polynome
uvp1 = polyval(cff, t);
wp = polyval(c, t);
cd = polyder(c);    % Koeffizienten von wpd
wpd = polyval(cd, t);
cdd = polyder(cd);  % Koeffizienten von wpdd
wpdd = polyval(cdd, t);

% Plot für Signal uvp1 für die Online-Vorsteuerung
figure(5);
clf;
plot(t,uvp1); grid on;
title('Führungssignal der Vorsteuerung uVp1');
xlabel('t (seconds)');
ylabel('uvp1 (meter)');
legend('uvp1(t)');
% Plot für das Führungssignal wp und die Ableitungen wpd und wpdd
figure(6);
clf;
plot(t,wp); grid on;
title('Führungssignal wp');
xlabel('t (seconds)');
ylabel('uvp1 (meter)');
legend('wp(t)');
figure(7);
clf;
plot(t,wpd); grid on;
title('Erste Ableitung des Führungssignal wpd');
xlabel('t (seconds)');
ylabel('wpd (meter/second)');
legend('wpd(t)');
figure(8);
clf;
plot(t,wpdd); grid on;
title('Zweite Ableitung des Führungssignal wpd');
xlabel('t (seconds)');
ylabel('wpdd (meter/second^2)');
legend('wpdd(t)');

% Plot des Position y im Vergleich zum Führungsignal wp ohne Vorsteuerung
figure(9);
clf;
lsim(G_Wp, wp, t); grid on;
legend('y(t)');

% Plot des Position y im Vergleich zum Vorsteuerungssignal uvp1 ohne
% Führungssignal
figure(10);
clf;
lsim(G_V, uvp1, t); grid on;
legend('y(t)');

% Plot der Position y mit Führungssignal wp und Vorsteuerung
y1 = lsim(G_V, uvp1, t);
y2 = lsim(G_Wp, wp, t);
figure(11);
clf;
yp = y1+y2;
ep = wp-transpose(yp);
plot(t ,yp, t, wp, '--', t, ep, '--'); grid on;
title('Regelgröße yp');
xlabel('t (seconds)');
ylabel('yp (meter)');
legend('yp(t)', 'wp(t)', 'ep(t)');





