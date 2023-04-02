function cff = cd_refpoly_ff(c, k, T, Tt, kr, Ti)
% Berechnet die Koeffizienten für des Führungssignal uVp1 der Vorsteuerung 
% Parameter Typ         Beschreibung
% c         6x1 double  Koeffizienten des Führungssignals wp
% k         1x1 double  Verstärkung der Strecke [m/s]
% T         1x1 double  Zeitkonstante der Stecke [s]
% Tt        1x1 double  Totzeit [s]
% kr        1x1 double  Verstärung des Geschwindigkeitsreglers [s/m]
% Ti        1x1 double  Zeitkonstante des Geschwindigkeitsreglers [s]
% Rückgabe  Typ         Beschreibung
% cff       6x1 double  Koeffizienten des Führungssignal uVp1
cff = zeros(6,1);
a1 = Ti * (1/(k*kr) + 1);
a2 = Ti*(T+Tt)/(k*kr);
a3 = T*Ti*Tt/(k*kr);
cff = [c(1);...
       c(2) + 5*c(1)*a1;...
       c(3) + 4*c(2)*a1 + 20*c(1)*a2;...
       c(4) + 3*c(3)*a1 + 12*c(2)*a2 + 60*c(1)*a3;...
       c(5) + 2*c(4)*a1 +  6*c(3)*a2 + 24*c(2)*a3;...
       c(6) +   c(5)*a1 +  2*c(4)*a2 +  6*c(3)*a3];
end

