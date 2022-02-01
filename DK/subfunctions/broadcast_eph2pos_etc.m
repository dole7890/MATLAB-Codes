function [health, x, bsv, relsv, xdot] = broadcast_eph2pos_etc(alm, t_input, prn)

% INPUT:               Description                                  Units
%
%  ephem_all    - matrix of gps satellite orbit parameters           (nx25)
%
%                  col1: prn, PRN number of satellite
%                  col2: M0, mean anomaly at reference time, rad
%                  col3: delta_n, mean motion difference from computed value, rad/s
%                  col4: ecc, eccentricity of orbit
%                  col5: sqrt_a, square root of semi-major axis, m^0.5
%                  col6: Loa, longitude of ascending node of orbit plane at weekly epoch, rad
%                  col7: incl, inclination angle at reference time, rad
%                  col8: perigee, argument of perigee, rad
%                  col9: ra_rate, rate of change of right ascension, rad/s
%                 col10: i_rate, rate of change of inclination angle, rad/s
%                 col11: Cuc, amplitude of the cosine harmonic correction term to the argument of latitude
%                 col12: Cus, amplitude of the sine harmonic correction term to the argument of latitude
%                 col13: Crc, amplitude of the cosine harmonic correction term to the orbit radius
%                 col14: Crs, amplitude of the sine harmonic correction term to the orbit radius
%                 col15: Cic, amplitude of the cosine harmonic correction term to the angle of inclination
%                 col16: Cis, amplitude of the cosine harmonic correction term to the angle of inclination
%                 col17: Toe, reference time ephemeris (seconds into GPS week)
%                 col18: IODE, issue of data (ephemeris)
%                 col19: GPS_week, GPS Week Number (to go with Toe)
%                 col20: Toc, time of clock
%                 col21: Af0, satellite clock bias (sec)
%                 col22: Af1, satellite clock drift (sec/sec)
%                 col23: Af2, satellite clock drift rate (sec/sec/sec)
%                 col24: Timing Group Delay (TGD), seconds
%                 col25: health, satellite health (0=good and usable)
%
%  t_input      - GPS times to calculate values at                 [WN TOW] (nx2)
%  prn          - PRN to compute values for (one satellite only)
%
%
%
% OUTPUT:
%
%  health       - health of satellite (0=good)                              (nx1)
%  x            - position of satellite (ECEF)                  [x y z]   m (nx3)
%


% SAME INPUTS AND OUTPUTS AS THE GIVEN FUNCTION
%==========================================================================
% Load GPS Accepted WGS-84 Constants
%==========================================================================
muE = 3.986005e14;     % WGS-84 value, m^3/s^2
wE  = 7.2921151467e-5; % WGS-84 value, rad/s
c   = 2.99792458e8;    % GPS acceptd speed of light, m/s

%==========================================================================
% Initialize Output Variables for Speed
%==========================================================================
sz         = size(t_input,1);
x          = ones(sz,3) * NaN;
health     = ones(sz,1) * NaN;


%==========================================================================
% Pull Out Correct Ephemerides
%==========================================================================

% Pull out ephemerides for PRN in question
kk  = find(alm(:,1) == prn);  % kk is vector containing row numbers of ephem_all that are for sat.no. 'index'
sat_ephem = alm(kk,:);        % sat_ephem is matrix of all ephem data for each entry of sat.no. 'index'


% No matching PRN found, returning data will be NaNs
if isempty(kk),return,end




%==========================================================================
% Start Main Calculation Loop
%==========================================================================

% Compute elapsed times of each ephemeris epoch wrt first entry, seconds
dt_ephem = (sat_ephem(:,19) - sat_ephem(1,19))*604800 + (sat_ephem(:,17) - sat_ephem(1,17));


% Compute elapsed times of each input time wrt first ephemeris entry, seconds
dt_input = (t_input(:,1) - sat_ephem(1,19))*604800 + (t_input(:,2) - sat_ephem(1,17));



for tt = 1:sz % loop through all input times
    
    
    % Pull out most recent ephemeris values
    %     jj = max( find(dt_input(tt) >= dt_ephem) ); % sat_ephem(:,17) = toe (sec into GPS week) of each entry
    % jj = row of specific sat. ephem. data with epoch closest to input time
    
    % Pull out nearest ephemeris values
    [mn,jj] = min(abs( dt_input(tt) - dt_ephem ));
    
    
    
    if isempty(jj),continue,end  % no matching ephemeris time found. continue to next input time
    
    
    % Pull out common variables from the ephemeris matrix
    %======================================================================
    %toe = sat_ephem(jj,17);           % time of ephemeris
    dt  = dt_input(tt) - dt_ephem(jj); % seconds difference from epoch
    
    a   = sat_ephem(jj,5)^2;           % semimajor axis, sqrt(a) = gps_ephem_all(:,5) (meters)
    ecc = sat_ephem(jj,4);             % eccentricity
    n0  = sqrt(muE/a^3);               % nominal mean motion (rad/s)
    n   = n0 + sat_ephem(jj,3);        % corrected mean motion, delta_n = gps_ephem_all(:,3)
    M   = sat_ephem(jj,2) + n*dt;      % mean anomaly, M0 = gps_ephem_all(:,2)
    
    
    % Compute perigee, true and eccentric anomaly...
    %======================================================================
    
    % Load argument of perigee to a local variable and add perigee rate, rad
    perigee  = sat_ephem(jj,8); % + perigee_rate * dt;
    
    % Compute Eccentric Anomaly, rad
    E    = mean2eccentric(M,ecc);
    cosE = cos(E);
    sinE = sin(E);
    
    % Compute true anomaly, rad
    nu    = atan2( sqrt(1 - ecc*ecc).*sinE,  cosE-ecc );
    
    % Compute the argument of latitude, rad
    u = nu + perigee;  % true anomaly + argument of perigee
    
    %                 col11: Cuc, amplitude of the cosine harmonic correction term to the argument of latitude
    %                 col12: Cus, amplitude of the sine harmonic correction term to the argument of latitude
    %                 col13: Crc, amplitude of the cosine harmonic correction term to the orbit radius
    %                 col14: Crs, amplitude of the sine harmonic correction term to the orbit radius
    %                 col15: Cic, amplitude of the cosine harmonic correction term to the angle of inclination
    %                 col16: Cis, amplitude of the cosine harmonic correction term to the angle of inclination
    Cuc = sat_ephem(jj,11);
    Cus = sat_ephem(jj,12);
    Crc = sat_ephem(jj,13);
    Crs = sat_ephem(jj,14);
    Cic = sat_ephem(jj,15);
    Cis = sat_ephem(jj,16);
    
    % Compute corrections
    du = Cus * sin(2 * u) + Cuc * cos(2 * u);
    dr = Crs * sin(2 * u) + Crc * cos(2 * u);
    di = Cis * sin(2 * u) + Cic * cos(2 * u);
    
    
    % Compute radius and inclination
    %======================================================================
    
    r   = a * (1 - ecc*cosE) ;                        % corrected radius
    inc = sat_ephem(jj,7) ;   %  inclination
    i_dot = sat_ephem(jj,10);
    
    % Apply corrections
    u = u + du;
    r = r + dr;
    inc = inc + di + i_dot*dt;
    
    cosu = cos(u);
    sinu = sin(u);
    
    % Compute satellite position in orbital plane (Eq. 13)
    %======================================================================
    xo = r * cosu;    % satellite x-position in orbital plane
    yo = r * sinu;    % satellite y-position in orbital plane
    
    % Corrected longitude of ascending node for node rate and Earth rotation
    %======================================================================
    % Ascending node = ephem_all(jj,6)
    node = sat_ephem(jj,6) + (sat_ephem(jj,9) - wE)*dt -  (wE * sat_ephem(jj,17)); % Toe = gps_ephem_all(jj,17)
    
    % Calculate GPS Satellite Position in ECEF (m)
    %======================================================================
    cosi = cos(inc);    sini = sin(inc);
    coso = cos(node);   sino = sin(node);
    
    
    % Satellite position in ECEF (m)
    x(tt,1) = xo*coso - yo*cosi*sino;  %x-position
    
    x(tt,2) = xo*sino + yo*cosi*coso;  %y-position
    
    x(tt,3) = yo*sini;                 %z-position
    
    
    % Keep track of health of each satellite
    %======================================================================
    health(tt,1) = sat_ephem(jj,25); % satellite health (0.00 is useable)
    
    % clock error
    %                 col17: Toe, reference time ephemeris (seconds into GPS week)
    %                 col20: Toc, time of clock
    %                 col21: Af0, satellite clock bias (sec)
    %                 col22: Af1, satellite clock drift (sec/sec)
    %                 col23: Af2, satellite clock drift rate (sec/sec/sec)
    Af0 = sat_ephem(jj,21);
    Af1 = sat_ephem(jj,22);
    toe = sat_ephem(jj,17);
    dts = Af0 + Af1*(dt);
    bsv(tt) = dts * c;
    
    % relitivistic error
    mu = n^2 * a^3;
    tau = 2 / c^2 * sqrt(a*mu) * ecc * sin(E);
    relsv(tt) = tau * c;
    
    
    M_dot = n;
    E_dot = M_dot/(1.0 - sat_ephem(jj,4)*cosE);
    nu_dot=sinE*E_dot*(1.0+sat_ephem(jj,4)*cos(nu))/(sin(nu)*(1.0-sat_ephem(jj,4)*cosE));
    u_dot=nu_dot+2.0*(sat_ephem(jj,12)*cos(2.0*u)-sat_ephem(jj,11)*sin(2.0*u))*nu_dot;
    r_dot = a*sat_ephem(jj,4)*sin(E)*n/(1.0-sat_ephem(jj,4)*cos(E)) + ...
        2.0*(sat_ephem(jj,14)*cos(2.0*u)-sat_ephem(jj,13)*sin(2.0*u))*nu_dot;
    idot = sat_ephem(jj,10) + (sat_ephem(jj,16)*cos(2.0*u)-sat_ephem(jj,15)*sin(2.0*u))*2.0*nu_dot;
    
    xodot=r_dot*cosu-yo*u_dot;
    yodot=r_dot*sinu+xo*u_dot;
    odot = (sat_ephem(jj,9) - wE);
    
    % Satellite velocity in ECEF (m)
    xdot(tt,1) = xodot*coso - yodot*cosi*sino + yo*sini*sino*idot - x(tt,2)*odot;
    xdot(tt,2) = xodot*sino + yodot*cosi*coso - yo*sini*idot*coso + x(tt,1)*odot;
    xdot(tt,3) = yodot*sini+yo*cosi*idot;
end % END of t_input loop =================================================
%==========================================================================



