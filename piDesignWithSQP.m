% defining the plant
s = tf('s');
G = 0.007143/(3.8e-07*s^3 + 9.38e-05*s^2 + 0.5111*s);

% desired settling time and overshoot
Ts = 2;
OS = 0;

% conversion of damping ratio and natural frequency
zeta = abs(log(OS/100))/sqrt(pi^2 + (log(OS/100))^2);
wn = 4/(zeta*Ts);

objective = @(P) system_characteristics(P, G, s, zeta, wn);

% init guess
P0 = [150; 2];

options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
P = fmincon(objective, P0, [], [], [], [], [], [], [], options);

Kp = P(1);
Ki = P(2);
fprintf('Kp = %f\n', Kp);
fprintf('Ki = %f\n', Ki);

function error = system_characteristics(P, G, s, zeta, wn)
    % Define the controller
    Kp = P(1);
    Ki = P(2);
    C = Kp + Ki/s;

    L = G*C;
    [mag, phase, w] = bode(L);
    mag = squeeze(mag);
    phase = squeeze(phase);

    zeta_est = -cosd(phase(w >= wn & phase < -180, 1));
    wn_est = w(mag >= 1 & phase < -180, 1);

   
    if isempty(zeta_est) || isempty(wn_est)
        error = 1e10;
    else
       
        error = (zeta - zeta_est(1)).^2 + (wn - wn_est(1)).^2;
    end
end
