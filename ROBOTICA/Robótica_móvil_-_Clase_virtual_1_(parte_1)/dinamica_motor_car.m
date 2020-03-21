close all,clear all,clc

T = 0.01;
Tf = 4;     %TIEMPO DE EJECUCIÓN DE LA SIMULACIÓN
t1 = 0:T:4;
wRd = 0.1167*ones(size(t1));  % entrada al motor derecho
wLd = 0.1167*ones(size(t1));  % entrada al motor izquierdo

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

% parametros del modelo del carro, definidos previamente
m = 3.79; %en kg
L = 0.235; %en m
rr = 0.035; %en m
R = 0.35; %en m
Iz = 1/4*m*R^2; % en kg-m^2
Tc = 0.1;
t2 = 0:Tc:Tf+Tc;

Kt = 1.5;
Xf = 20;
Xr = 10;
xG = 0;
Nf = 0.3;
Nr = 0.1;

% inicializamos las velocidades, posicion y orientacion del vehiculo
x(1) = 0;
y(1) = 0;
theta(1) = 0;
u(1) = 0;
r(1) = 0;

for i=1:length(wRd)
    % calculamos velocidad, error y control para el motor derecho
    wR(i) = 0.9541*wRkm1 - 0.1108*wRkm2 + 0.0948*uRkm1 + 0.04626*uRkm2;
    ewR(i) = wRd(i) - wR(i);
    uR(i) = uRkm1 + 1.4366*ewR(i) - 1.4366*0.8092*ewRkm1;
    
    % actualizamos variables para el motor derecho
    wRkm2 = wRkm1;
    wRkm1 = wR(i);
    uRkm2 = uRkm1;
    uRkm1 = uR(i);
    ewRkm1 = ewR(i);

    % calculamos velocidad, error y control para el motor izquierdo
    wL(i) = 1.035*wLkm1 - 0.1599*wLkm2 + 0.06838*uLkm1 + 0.03746*uLkm2;
    ewL(i) = wLd(i) - wL(i);
    uL(i) = uLkm1 + 1.5885*ewL(i) - 1.5885*0.8367*ewLkm1;   
    
    % actualizamos variables para el motor izquierdo
    wLkm2 = wLkm1;
    wLkm1 = wL(i);
    uLkm2 = uLkm1;
    uLkm1 = uL(i);
    ewLkm1 = ewL(i);
    
    % se ejecuta la dinamica del carro para i=1 y para i multiplo entero 
    % del periodo de muestreo del carro
    if i==1
        u(i+1) = u(i) + Tc*1/m*(Kt*wR(i)/rr + Kt*wL(i)/rr - Xf*u(i) - Xr*u(i)^2 + xG*r(i)^2);
        r(i+1) = r(i) + Tc*1/Iz*(0.5*Kt*L*wR(i)/rr - 0.5*Kt*L*wL(i)/rr - Nf*r(i) - Nr*r(i)^2 - m*xG*u(i)*r(i));
        
        theta(i+1) = theta(i) + 180/pi*Tc*r(i+1);
        x(i+1) = x(i) + Tc*u(i+1)*cosd(theta(i+1));
        y(i+1) = y(i) + Tc*u(i+1)*sind(theta(i+1));
    elseif mod(i/(Tc/T),1)==0 
        j = i/(Tc/T)+1; % indice de las variables del carro
        u(j+1) = u(j) + Tc*1/m*(Kt*wR(i)/rr + Kt*wL(i)/rr - Xf*u(j) - Xr*u(j)^2 + xG*r(j)^2);
        r(j+1) = r(j) + Tc*1/Iz*(0.5*Kt*L*wR(i)/rr - 0.5*Kt*L*wL(i)/rr - Nf*r(j) - Nr*r(j)^2 - m*xG*u(j)*r(j));
        
        theta(j+1) = theta(j) + 180/pi*Tc*r(j+1);
        x(j+1) = x(j) + Tc*u(j+1)*cosd(theta(j+1));
        y(j+1) = y(j) + Tc*u(j+1)*sind(theta(j+1));
    end
end

% grafica de la trayectoria en el plano (x,y)
nn=0.05;
for i = 1:length(x)
    figure(1),plot([x(i)  x(i)+nn*cosd(theta(i))],[y(i) y(i)+nn*sind(theta(i))],'LineWidth',2), hold on
    pause(0.2)
end
axis equal

figure,plot(t2,u)
figure,plot(t2,r)
