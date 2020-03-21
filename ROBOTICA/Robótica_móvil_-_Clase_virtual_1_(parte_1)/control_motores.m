close all,clear all,clc
Kd = 0.7;
taud1 = 0.04;
taud11 = 0.008;
motord = tf(Kd,conv([taud1  1],[taud11  1]));
T = 0.01;
motordd = c2d(motord,T); % discretizacion
step(motordd),hold on
%step(motordd)
Ki = 0.75;
taui1 = 0.05;
taui11 = 0.008;
motori = tf(Ki,conv([taui1  1],[taui11  1]));
motorid = c2d(motori,T);
step(motori)
step(motorid)
% Diseño para el motor derecho
zeta = 0.99; % definitivo 0.99
tsd = 0.248;  % definitivo 0.25 d y 
wn = 6.5/(tsd*zeta); % 6.5 para zeta=>0.8
sysdd = tf(wn^2,[1  2*zeta*wn  wn^2]);
figure(1),step(sysdd)
pd = -zeta*wn + j*wn*sqrt(1-zeta^2);
zd = exp(pd*T);
ang_G = 180/pi*angle(evalfr(motordd,zd));
ang_C = 180 - ang_G;
if ang_C>180 & ang_C<=360
    ang_C = ang_C - 360;
end
% diseno PI o PID
if ang_C<0
    ang_a = ang_C + 180/pi*angle(zd-1);
    a = real(zd) + imag(zd)/tand(180-ang_a);% para angulos > 90
    %a = real(zd) - imag(zd)/tand(ang_a); %para angulos < 90, con el cero en el semiplano izquierdo
    %a = -real(zd) + imag(zd)/tand(ang_a); %para angulos < 90, con el cero en el semiplano derecho
    ctrld = zpk([a],[1],1,T);
    K = 1/abs(evalfr(motordd*ctrld,zd));
    ctrld = zpk([a],[1],K,T);
else
    ang_zeros = ang_C + 180/pi*angle(zd-1) + 180/pi*angle(zd);
    if ang_zeros>360
        ang_zeros = ang_zeros - 360;
    end
    ang_a = ang_zeros/2;
    %a = real(zd) + imag(zd)/tand(180-ang_a);% para angulos > 90
    %a = real(zd) - imag(zd)/tand(ang_a); %para angulos < 90, con el cero en el semiplano izquierdo
    a = -real(zd) + imag(zd)/tand(ang_a); %para angulos < 90, con el cero en el semiplano derecho
    b = a;
    ctrld = zpk([a b],[1 0],1,T);
    K = 1/abs(evalfr(motordd*ctrld,zd));
    ctrld = zpk([a b],[1 0],K,T);
end
pole(feedback(motordd*ctrld,1));
ctrldd = ctrld;
figure(1),step(feedback(motordd*ctrldd,1)), hold on
figure(2),rlocus(motordd*ctrldd), hold on, plot(real(zd),imag(zd),'x')
figure(3),step(feedback(motordd*ctrldd,1)), hold on
% Diseño para el motor izquierdo
zeta = 0.99; % definitivo 0.99
tsd = 0.2704;  % definitivo 0.28 d y 
wn = 6.25/(tsd*zeta);
sysid = tf(wn^2,[1  2*zeta*wn  wn^2]);
figure(1),step(sysid)
pd = -zeta*wn + j*wn*sqrt(1-zeta^2);
zd = exp(pd*T);
ang_G = 180/pi*angle(evalfr(motorid,zd));
ang_C = 180 - ang_G;
if ang_C>180 & ang_C<=360
    ang_C = ang_C - 360;
end
% diseno PI o PID
if ang_C<0
    ang_a = ang_C + 180/pi*angle(zd-1);
    a = real(zd) + imag(zd)/tand(180-ang_a);% para angulos > 90
    %a = real(zd) - imag(zd)/tand(ang_a); %para angulos < 90, con el cero en el semiplano izquierdo
    %a = -real(zd) + imag(zd)/tand(ang_a); %para angulos < 90, con el cero en el semiplano derecho
    ctrld = zpk([a],[1],1,T);
    K = 1/abs(evalfr(motorid*ctrld,zd));
    ctrld = zpk([a],[1],K,T);
else
    ang_zeros = ang_C + 180/pi*angle(zd-1) + 180/pi*angle(zd);
    if ang_zeros>360
        ang_zeros = ang_zeros - 360;
    end
    ang_a = ang_zeros/2;
    %a = real(zd) + imag(zd)/tand(180-ang_a);% para angulos > 90
    %a = real(zd) - imag(zd)/tand(ang_a); %para angulos < 90, con el cero en el semiplano izquierdo
    a = -real(zd) + imag(zd)/tand(ang_a); %para angulos < 90, con el cero en el semiplano derecho
    b = a;
    ctrld = zpk([a b],[1 0],1,T);
    K = 1/abs(evalfr(motorid*ctrld,zd));
    ctrld = zpk([a b],[1 0],K,T);
end
pole(feedback(motorid*ctrld,1));
figure(1),step(feedback(motorid*ctrld,1))
figure(2),rlocus(motorid*ctrld), hold on, plot(real(zd),imag(zd),'x')
figure(3),step(feedback(motorid*ctrld,1))
ctrlid = ctrld;
figure(1),legend('motordd','motori','motorid','sysdd','cl_motord','sysid','cl_motori','Location','SouthEast')
%% respuesta de los lazos de los motores a cualquier entrada
wRd = 1*ones(1,41);  % entrada al motor derecho
wLd = 1*ones(1,41);  % entrada al motor izquierdo

%valores iniciales del motor derecho
wRkm1 = 0;
wRkm2 = 0;
uRkm1 = 0;
uRkm2 = 0;
ewRkm1 = 0;

%valores iniciales del motor izquierdo
wLkm1 = 0;
wLkm2 = 0;
uLkm1 = 0;
uLkm2 = 0;
ewLkm1 = 0;

for i=1:length(wRd)
    % calculamos velocidad, error y control para el motor derecho
    c1d=0.06869;
    c2d=0.04179;
    d1d=-1.065;
    d2d=0.2231;
    kd=1.2198;
    ad=0.7761;
    wR(i) = -d1d*wRkm1 - d2d*wRkm2 + c1d*uRkm1 + c2d*uRkm2;
    ewR(i) = wRd(i) - wR(i);
    uR(i) = uRkm1 + kd*ewR(i) - kd*ad*ewRkm1;
    
    % actualizamos variables para el motor derecho
    wRkm2 = wRkm1;
    wRkm1 = wR(i);
    uRkm2 = uRkm1;
    uRkm1 = uR(i);
    ewRkm1 = ewR(i);

    % calculamos velocidad, error y control para el motor izquierdo
    c1i=0.05992;
    c2i=0.03708;
    d1i=-1.105;
    d2i=0.2346;
    ki=1.3574;
    ai=0.8144;
    wL(i) = -d1i*wLkm1 - d2i*wLkm2 + c1i*uLkm1 + c2i*uLkm2;
    ewL(i) = wLd(i) - wL(i);
    uL(i) = uLkm1 + ki*ewL(i) - ki*ai*ewLkm1;   
    
    % actualizamos variables para el motor izquierdo
    wLkm2 = wLkm1;
    wLkm1 = wL(i);
    uLkm2 = uLkm1;
    uLkm1 = uL(i);
    ewLkm1 = ewL(i);

end
% verificamos que los valores calculados coincidan con la respuesta al
% escalon y, de ser asi, estamos listos para que wRd y wLd tomen cualquier
% conjunto de valores
figure(3), plot(0:0.01:0.4,wR,'o-b')
figure(3), plot(0:0.01:0.4,wL,'o-r')