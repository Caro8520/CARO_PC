clear all,close all,clc
% suponiendo que podemos fijar las velocidades angulares de los motores
% parametros de entrada
m = 4;          %masa del vehiculo en kg
L = 0.235;      %distancia entre las ruedas del vehiculo en m
rr = 0.035;     %radio de las ruedas del vehiculo en m
R = 0.2;        %radio de la base del vehiculo en m
Iz = 1/4*m*R^2; %inercia al rededor del eje Z en kg-m^2
Kt = 1.5;
% maxima velocidad del Qbot 2 => 70 cm/s
% wL y wR se calculan de forma tal que tengamos una fuerza de 10N y un
% torque de 1N-m dejando wR fija para el calculo de 10N
Wd_l=(10*rr)/(2*Kt);            %Velocidades de las llantas para ir en linea recta
Wi_g=Wd_l-((2*0.5*rr)/(L*Kt));  %Velocidades de la llanta iz para describir un circulo
wL = Wd_l*ones(1,40); %rad/s 
wR = Wd_l*ones(1,40); %rad/s
% wL = Wi_g*ones(1,40); %rad/s 
% wR = Wd_l*ones(1,40); %rad/s
% wL = Wd_l*ones(1,40); %rad/s 
% wR = Wi_g*ones(1,40); %rad/s
% aproximando la derivada con un T = 0.1 s
Tc = 0.1;
u = zeros(1,length(wL)+1);
r = zeros(1,length(wL)+1);
theta = zeros(1,length(u)+1);
x = zeros(1,length(u)+1);
y = zeros(1,length(u)+1);
% velocidades, orentacion y posicion inicial del carro
u(1) = 0;
r(1) = 0;
x(1) = 0;
y(1) = 0;
theta(1) = 0;
% Evolución de las velocidades, orientación y posicioón del carro a partir
% de las ecuaciones dinámicas y cinemáticas
% La dinámica de la velocidad de avance y de giro esta dada por
% dot(u) = 1/m*(Kt*wR/rr + Kt*wL/rr - Xf*u - Xr*u^2 + xG*r^2);
% dot(r) = 1/Iz*(Kt*L/2*wR/rr + Kt*L/2*wL/rr - Nf*r - Nr*r^2 - m*xG*u*r^2);
% Nf = 0.0066;Nr = 0.0092; Nr=1;para el tiempo de establecimiento
% Xf = 1;      
% Xr = 19.05;  
% xG = 0;    
% Nf = 0.0066;   
% Nr = 0.0092;
Xf = 13;      
Xr = 1.64;  
xG = 0;    
Nf = 0.233;  
Nr = 0.008;
% las ecuaciones de la cinematica estan dadas por
% dot(x) = V*cos(theta)
% dot(y) = V*Sin(theta)
% dot(theta) = w
for i = 2:length(wL)+1
    u(i) = u(i-1) + Tc*1/m*(Kt*wR(i-1)/rr + Kt*wL(i-1)/rr - Xf*u(i-1) - Xr*u(i-1)^2 + xG*r(i-1)^2);
    r(i) = r(i-1) + Tc*(1/Iz)*(((Kt*L)/(2*rr))*wR(i-1) - ((Kt*L)/(2*rr))*wL(i-1) - Nf*r(i-1) - Nr*r(i-1)^2 - m*xG*u(i-1)*r(i-1));
    theta(i) = theta(i-1) + Tc*r(i-1);
    x(i) = x(i-1) + Tc*u(i-1)*cos(theta(i));
    y(i) = y(i-1) + Tc*u(i-1)*sin(theta(i));
end
% grafico de la evolucion de la posicion y orientacion del vehiculo
nn = 0.05;
for i = 1:length(u)+1
    plot([x(i)  x(i)+nn*cos(theta(i))],[y(i) y(i)+nn*sin(theta(i))],'LineWidth',2), hold on
    text(x(i),y(i),sprintf('%d',i));
    %pause(0.1)
end
axis equal,grid on
% grafico de u y r vs. t
figure,plot(0:0.1:0.1*(length(u)-1),u),grid on,hold on
%plot(0.5,0.686,'rx','MarkerSize',20,'lineWidth',5)
figure,plot(0:0.1:0.1*(length(u)-1),r),grid on