close all,clear all,clc
T = 0.01;                 %PERIODO DE MUESTREO DE LOS MOTORES
Tf = 5;                   %TIEMPO DE EJECUCIÓN DE LA SIMULACIÓN
t1 = 0:T:Tf;
Wl = 0.1167;              %VELOCIDADES LINEA RECTA FUERZA DE 10N
Wg = 0.1067;              %VELOCIDADES DESCRIBIR CIRCULO  0.05Nm
wRd = Wl*ones(size(t1));  %ENTRADA MOTOR DERECHO
wLd = Wl*ones(size(t1));  %ENTRADA MOTOR IZQUIERDO
%VALORES INICIALES MOTOR DERECHO
wR = zeros(3,1);
uR = zeros(3,1);
ewR = zeros(3,1);
%VALORES INICIALES MOTOR IZQUIERDO
wL = zeros(3,1);
uL = zeros(3,1);
ewL = zeros(3,1);
%PARAMETROS MODELO DEL CARRO
m = 4;          %Masa del vehiculo en (Kg)
L = 0.235;      %Distancia entre las ruedas del vehiculo en (m)
rr = 0.035;     %Radio de las ruedas del vehiculo en (m)
R = 0.2;        %Radio de la base del vehiculo en (m)
Iz = 1/4*m*R^2; %Inercia al rededor del eje Z en (Kg-m^2)
Tc = 0.1;       %PERIODO DE MUESTREO DEL VEHICULO
t2 = 0:Tc:Tf+Tc;
%%%%%%%%%%%%%%%
Kt = 1.5;
Xf = 13;      
Xr = 1.64;  
xG = 0;    
Nf = 0.233;  
Nr = 0.008;
% Inicializar las velocidades, Posición y Orientación del vehiculo
x = zeros(3,1);
y = zeros(3,1);
theta = zeros(3,1);
u = zeros(3,1);
r = zeros(3,1);
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
j=3;
actualizar=1;
for i=3:length(wRd)
    %CALCULAR VELOCIDAD, ERROR Y CONTROL MOTOR DERECHO
    wR(i) = -d1d*wR(i-1) - d2d*wR(i-2) + c1d*uR(i-1) + c2d*uR(i-2);
    ewR(i) = wRd(i) - wR(i);
    uR(i) = uR(i-1) + kd*ewR(i) - kd*ad*ewR(i-1);
    %CALCULAR VELOCIDAD, ERROR Y CONTROL MOTOR IZQUIERDO
    wL(i) = -d1i*wL(i-1) - d2i*wL(i-2) + c1i*uL(i-1) + c2i*uL(i-2);
    ewL(i) = wLd(i) - wL(i);
    uL(i) = uL(i-1) + ki*ewL(i) - ki*ai*ewL(i-1);
    %SE EJECUTA LA DINÁMICA DEL CARRO PARA i=1 Y PARA i MULTIPLO ENTERO DEL PERIODO DE MUESTREO DEL CARRO
    if mod((i-2)/(Tc/T),1)==0
        j=j+1;
        actualizar=1;
    end
    if(actualizar==1)
        actualizar=0;
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
figure,subplot(1,2,1),plot(0:0.1:0.1*(length(u)-1),u,'lineWidth',2),grid on,title("VELOCIDAD DE AVANCE u(t)")
subplot(1,2,2),plot(0:0.1:0.1*(length(r)-1),r,'r','lineWidth',2),grid on,title("VELOCIDAD ANGULAR r(t)")