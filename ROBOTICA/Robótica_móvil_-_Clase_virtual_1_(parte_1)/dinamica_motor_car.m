close all,clear all,clc
T = 0.01;                 %PERIODO DE MUESTREO DE LOS MOTORES
Tf = 10;                   %TIEMPO DE EJECUCIÓN DE LA SIMULACIÓN
t1 = 0:T:Tf;
Wl = 0.1167;              %VELOCIDADES LINEA RECTA FUERZA DE 10N
Wg = 0.1067;              %VELOCIDADES DESCRIBIR CIRCULO  0.05Nm
wRd = Wl*ones(size(t1));  %ENTRADA MOTOR DERECHO
wLd = Wl*ones(size(t1));  %ENTRADA MOTOR IZQUIERDO
%VALORES INICIALES MOTOR DERECHO
wRkm1 = 0;
wRkm2 = 0;
uRkm1 = 0;
uRkm2 = 0;
ewRkm1 = 0;
%VALORES INICIALES MOTOR IZQUIERDO
wLkm1 = 0;
wLkm2 = 0;
uLkm1 = 0;
uLkm2 = 0;
ewLkm1 = 0;
%PARAMETROS MODELO DEL CARRO
m = 4;          %Masa del vehiculo en (Kg)
L = 0.235;      %Distancia entre las ruedas del vehiculo en (m)
rr = 0.035;     %Radio de las ruedas del vehiculo en (m)
R = 0.2;        %Radio de la base del vehiculo en (m)
Iz = 1/4*m*R^2; %Inercia al rededor del eje Z en (Kg-m^2)
Tc = 0.1;       %PERIODO DE MUESTREO DEL VEHICULO
t2 = 0:Tc:Tf+Tc;
%PARAMETROS DINÁMICOS DEL VEHICULO
Kt = 1.5;
Xf = 13;      
Xr = 1.64;  
xG = 0;    
Nf = 0.233;  
Nr = 0.008;
% Inicializar las velocidades, Posición y Orientación del vehiculo
x(1) = 0;
y(1) = 0;
theta(1) = 0;
u(1) = 0;
r(1) = 0;
%PARAMETROS MOTOR DERECHO DISCRETIZADO
c1d=0.06869;
c2d=0.04179;
d1d=-1.065;
d2d=0.2231;
%PARAMETROS CONTROL PI MOTOR DERECHO
kd=1.2198;
ad=0.7761;
%PARAMETROS MOTOR IZQUIERDO DISCRETIZADO
c1i=0.05992;
c2i=0.03708;
d1i=-1.105;
d2i=0.2346;
%PARAMETROS CONTROL PI MOTOR IZQUIERDO
ki=1.3574;
ai=0.8144;
for i=1:length(wRd)
    %CALCULAR VELOCIDAD, ERROR Y CONTROL MOTOR DERECHO
    wR(i) = -d1d*wRkm1 - d2d*wRkm2 + c1d*uRkm1 + c2d*uRkm2;
    ewR(i) = wRd(i) - wR(i);
    uR(i) = uRkm1 + kd*ewR(i) - kd*ad*ewRkm1;
    %ACTUALIZAR VARIABLES MOTOR DERECHO
    wRkm2 = wRkm1;
    wRkm1 = wR(i);
    uRkm2 = uRkm1;
    uRkm1 = uR(i);
    ewRkm1 = ewR(i);
    %CALCULAR VELOCIDAD, ERROR Y CONTROL MOTOR IZQUIERDO
    wL(i) = -d1i*wLkm1 - d2i*wLkm2 + c1i*uLkm1 + c2i*uLkm2;
    ewL(i) = wLd(i) - wL(i);
    uL(i) = uLkm1 + ki*ewL(i) - ki*ai*ewLkm1;   
    %ACTUALIZAR VARIABLES MOTOR IZQUIERDO
    wLkm2 = wLkm1;
    wLkm1 = wL(i);
    uLkm2 = uLkm1;
    uLkm1 = uL(i);
    ewLkm1 = ewL(i);
    %SE EJECUTA LA DINÁMICA DEL CARRO PARA i=1 Y PARA i MULTIPLO ENTERO DEL PERIODO DE MUESTREO DEL CARRO
    if i==1
        %DINÁMICA
        u(i+1) = u(i) + Tc*1/m*(Kt*wR(i)/rr + Kt*wL(i)/rr - Xf*u(i) - Xr*u(i)^2 + xG*r(i)^2);
        r(i+1) = r(i) + Tc*1/Iz*(0.5*Kt*L*wR(i)/rr - 0.5*Kt*L*wL(i)/rr - Nf*r(i) - Nr*r(i)^2 - m*xG*u(i)*r(i));
        %POSICIÓN DEL VEHÍCULO
        theta(i+1) = theta(i) + 180/pi*Tc*r(i+1);
        x(i+1) = x(i) + Tc*u(i+1)*cosd(theta(i+1));
        y(i+1) = y(i) + Tc*u(i+1)*sind(theta(i+1));
    elseif mod(i/(Tc/T),1)==0 
        j = i/(Tc/T)+1; % indice de las variables del carro
        %DINÁMICA
        u(j+1) = u(j) + Tc*1/m*(Kt*wR(i)/rr + Kt*wL(i)/rr - Xf*u(j) - Xr*u(j)^2 + xG*r(j)^2);
        r(j+1) = r(j) + Tc*1/Iz*(0.5*Kt*L*wR(i)/rr - 0.5*Kt*L*wL(i)/rr - Nf*r(j) - Nr*r(j)^2 - m*xG*u(j)*r(j));
        %POSICIÓN DEL VEHÍCULO
        theta(j+1) = theta(j) + 180/pi*Tc*r(j+1);
        x(j+1) = x(j) + Tc*u(j+1)*cosd(theta(j+1));
        y(j+1) = y(j) + Tc*u(j+1)*sind(theta(j+1));
    end
end
%GRÁFICA DE LA TRAYECTORIA EN EL PLANO (x,y)
nn=0.05;
for i = 1:length(x)
    figure(1),plot([x(i)  x(i)+nn*cosd(theta(i))],[y(i) y(i)+nn*sind(theta(i))],'LineWidth',2), hold on
    %pause(0.2)
end
axis equal,grid on
figure,subplot(1,2,1),plot(t2,u,'lineWidth',2),grid on,title("VELOCIDAD DE AVANCE u(t)")
subplot(1,2,2),plot(t2,r,'r','lineWidth',2),grid on,title("VELOCIDAD ANGULAR r(t)")