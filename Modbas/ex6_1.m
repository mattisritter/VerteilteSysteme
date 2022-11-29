%% Aufgabe 6.1 Entwurf des Geschwindigkeitsregler
clear all

%% Regleranforderungen
phires_soll = 65;       % Phasenrand [°]
Tm_soll = 1;            % Überschwingzeit [s]
omegaD = pi/Tm_soll;    % Durchtrittskreisfrequenz [rad/s]

%% Parameter
% Streckenparameter
Tt = 100e-3;            % Totzeit [s]
T = 316e-3;             % Zeitkonstante der Strecke [s]
k = 2.51;               % Verstärkung der Strecke [m/s]

% Reglerparameter
Ti = tan(phires_soll/180*pi + omegaD*Tt + atan(-1/omegaD/T) - 2*pi)/omegaD;
                        % Zeitkonstante des Reglers [s]
kr = Ti/k * sqrt(omegaD^2 + omegaD^4 * T^2) / sqrt(Ti^2*omegaD^2 + 1);
                        % Verstärung des Reglers [s/m]

%% Übertragungsfunktion der offenen Strecke                        
s = tf('s');
G_0 = k*kr/Ti * tf([Ti, 1], [T, 1, 0], 'Inputdelay', Tt);

figure(1)
margin(G_0);            % Bodediagramm mit Phasen- und Amplitudenrand

%% Führungsverhalten
G_W = minreal(G_0/(1+G_0));      % Führungsübertragung 

figure(2)
step(G_W), grid on;     % Sprungantwort

si = stepinfo(G_W);
Tm = si.PeakTime;       % Überschwingzeit [s]
em = si.Overshoot;      % Überschwingweite [%]