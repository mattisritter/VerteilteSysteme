%% MATLAB file zu Aufgabe 6.1 und 7.1
clear all;
%% Anforderungen an den Geschwindigkeitsregler
phires_soll = 65;       % Phasenrand [°]
Tm_soll = 1;            % Überschwingzeit [s]
omegaD = pi/Tm_soll;    % Durchtrittskreisfrequenz [rad/s]

%% Parameter
% Streckenparameter
Tt = 100e-3;            % Totzeit [s]
T = 316e-3;             % Zeitkonstante der Strecke [s]
k = 2.51;               % Verstärkung der Strecke [m/s]

% Geschwindigkeitsreglerparameter
Ti = tan(phires_soll/180*pi + omegaD*Tt + atan(-1/omegaD/T) - 2*pi)/omegaD;
                        % Zeitkonstante des Geschwindigkeitsreglers [s]
kr = Ti/k * sqrt(omegaD^2 + omegaD^4 * T^2) / sqrt(Ti^2*omegaD^2 + 1);
                        % Verstärung des Geschwindigkeitsreglers [s/m]
% Positionsreglerparameter                        
kp = 1;                 % Verstärkung des Positionsregler [1/s]  
                        
%% Übertragungsfunktionen
s = tf('s');
% Geschwindigkeitsregler
G_S = tf(k, [T, 1], 'Inputdelay', Tt);
G_R = kr*(Ti*s+1)/Ti/s;
G_0 = G_S*G_R;
G_W = minreal(G_0/(1+G_0));
% Approximierter Geschwindigkeitsregler
G_S_ap = tf(k,[T*Tt, (T+Tt), 1]);       % Strecke
G_R_ap = G_R;                           % Regler
G_0_ap = minreal(G_S_ap*G_R_ap);        % offene Strecke
G_W_ap = minreal(G_0_ap/(1+G_0_ap));    % Führungsübertragung
% Positionsregler
G_Sp = minreal(G_W_ap/s);               % Strecke
G_Rp = kp;                              % Regler
G_0p = minreal(G_Sp*G_Rp);              % offene Strecke
G_Wp = minreal(G_0p/(1+G_0p));          % Führungsübertragung

%% Bodediagramm der offenen Strecke Geschwindigkeitsregler
figure(1);
clf;
margin(G_0);            % Bodediagramm mit Phasen- und Amplitudenrand
%% Sprungantwort Führungsverhalten Geschwindigkeitsregler
figure(2);
clf;
step(G_W), grid on;   
% Analyse der Sprungantwort
si = stepinfo(G_W);
Tm = si.PeakTime;       % Überschwingzeit [s]
em = si.Overshoot;      % Überschwingweite [%]
%% Rampenantwort Positionsregler
v_s = 0.1;              % Soll Geschwindigkeit [m/s]
tsim = 0:0.01:10;       % Zeitintervall für die Rampe
u = v_s*tsim;           % Definition der Rampe als Eingangssignal

% Plot der Position im Vergleich y zum Eingangssignal
figure(3);
clf;
lsim(G_Wp, u, tsim); grid on;
legend('y(t)');

%% Stabilität
[r,gain] = rlocus(G_Wp);
for i = 1:length(gain)
    if real(r(2,i)) > 0 
       disp('kpmax = ');
       disp(gain(i-1));
       break;
    end 
end

%% Generierung des Führungssignals und Vorsteuersignal des Positionsreglers
vmax = 0.5; % Maximale Soll-Geschwindigkeit v* [m/s]
x0 = 0;     % Startposition [m]
xs = 1;     % Soll-Bogenlänge x* [m]

% Berechnung der Koeffizienten c für das Führungssignal wp und
% der Endzeit te
[c,te] = cd_refpoly_vmax(vmax, x0, xs);
% Berechnung der Koeffizienten cff für die Online-Vorsteuerung uvp1
cff = cd_refpoly_ff(c, k, T, Tt, kr, Ti);

t = 0:0.05:te; % Definiton des Zeitintervalls

% Bestimmung der Polynome
uvp1 = polyval(cff, t);
wp = polyval(c, t);
cd = polyder(c);    % Koeffizienten von wpd
wpd = polyval(cd, t);
cdd = polyder(cd);  % Koeffizienten von wpdd
wpdd = polyval(cdd, t);

% Plot für Signal uvp1 für die Online-Vorsteuerung
figure(4);
clf;
plot(t,uvp1); grid on;
legend('uvp1(t)');
% Plot für das Führungssignal wp und die Ableitungen wpd und wpdd
figure(5);
clf;
plot(t,wp, t,wpd, t,wpdd); grid on; 
legend('wp(t)', 'wpd(t)', 'wpdd(t)');

% Plot des Position y im Vergleich zum Führungsignal
figure(6);
clf;
lsim(G_Wp, wp, t); grid on;
legend('y(t)');

