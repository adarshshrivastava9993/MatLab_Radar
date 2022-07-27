%
% B747 lateral dynamics
% from Etkin and Reid
% Jonathan P. How
% 16.333 Fall 2004
%
% note Reid definition of positive aileron is different than nelson's on
% (6-4) - Reid defines positive Aileron as right A down (pg 86)
if 1
   
   Yv=-1.61e4;Yp=0;Yr=0;
   Lv=-3.062e5;Lp=-1.076e7;Lr=9.925e6;
   Nv=2.131e5;Np=-1.33e6;Nr=-8.934e6;
   
   g=9.81;theta0=0;S=511;cbar=8.324;b=59.64;
   U0=235.9;
   m=2.83176e6/g;cbar=8.324;rho=0.3045;
   Iyy=.449e8;Ixx=.247e8;Izz=.673e8;Ixz=-.212e7;
   
else
   
   Yv=-1.103e3;Yp=0;Yr=0;
   Lv=-6.885e4;Lp=-7.934e6;Lr=7.321e6;
   Nv=4.79e4;Np=-9.809e5;Nr=-6.590e6;
   
   g=32.2;U0=774;Iyy=.331e8;m=636636/g;cbar=27.31;rho=0.0005909;
   theta0=0;S=5500;b=195.7;
   Ixx=.183e8;Izz=.497e8;Ixz=-.156e7;
   
end

%
% nondimensional control derivatives
%
Cyda=0;Cydr=.1146;
Clda=-1.368e-2;Cldr=6.976e-3;
Cnda=-1.973e-4;Cndr=-.1257;

QdS=1/2*rho*U0^2*S;
Yda=QdS*Cyda;Ydr=QdS*Cydr;Lda=QdS*b*Clda;Ldr=QdS*b*Cldr;
Nda=QdS*b*Cnda;Ndr=QdS*b*Cndr;

Ixxp=(Ixx*Izz-Ixz^2)/Izz;
Izzp=(Ixx*Izz-Ixz^2)/Ixx;
Ixzp=Ixz/(Ixx*Izz-Ixz^2);
%
% full lateral derivatives
%
A=[Yv/m Yp/m (Yr/m-U0) g*cos(theta0);
   (Lv/Ixxp + Ixzp*Nv) (Lp/Ixxp + Ixzp*Np) (Lr/Ixxp + Ixzp*Nr) 0;
   (Ixzp*Lv + Nv/Izzp) (Ixzp*Lp + Np/Izzp) (Ixzp*Lr + Nr/Izzp) 0;
   0 1 tan(theta0) 0];

B=[1/m 0 0;0 1/Ixxp Ixzp;0 Ixzp 1/Izzp;0 0 0]*[Yda Ydr;Lda Ldr;Nda Ndr];

%
% reduced models
%
A_roll=(Lp/Ixxp + Ixzp*Np);
A_dr=[Yv/m -U0;(Ixzp*Lv + Nv/Izzp) (Ixzp*Lr + Nr/Izzp)];

[V,ev]=eig(A);ev=diag(ev);D=V;
%rifd(ev)
[ev_roll]=eig(A_roll);
%rifd(ev_roll)
[ev_dr]=eig(A_dr);
%rifd(ev_dr)

% nondimensionalization
V(1,:)=V(1,:)/U0;
V(2,:)=V(2,:)/(2*U0/b);
V(3,:)=V(3,:)/(2*U0/b);
for i=1:4
   V(:,i)=V(:,i)/V(4,i);
end

%return

C=[eye(4)];
freq=logspace(-2,1,400);
sys=ss(A,B,C,zeros(4,2));
G=freqresp(sys,freq);

TT=[0:.05:30]';
%sys_a=ss(A,[B(:,1)/180*pi],diag([180/pi/U0 180/pi 180/pi 180/pi]),zeros(4,1));
%[Ya,Ta]=impulse(sys_a,200);
sys_a=ss(A,[B(:,1)/180*pi],diag([1/U0 1 1 1]),zeros(4,1));
[Ya,Ta]=step(sys_a,30);
ii=find(TT <= 2);
Uda=0*TT;Uda(ii)=1*ones(size(ii));
[Ya,Ta]=lsim(sys_a,Uda,TT');
Udr=Uda;

%sys_r=ss(A,[B(:,2)/180*pi],diag([180/pi/U0 180/pi 180/pi 180/pi]),zeros(4,1));
%[Yr,Tr]=impulse(sys_r,200);
sys_r=ss(A,[B(:,2)/180*pi],diag([1/U0 1 1 1]),zeros(4,1));
[Yr,Tr]=step(sys_r,30);
%[Yr,Tr]=lsim(sys_r,Udr,TT');

figure(5);clf;
%plot(Tr,Yr(:,1),'-',Tr,Yr(:,2),'--',Tr,Yr(:,3),'^',Tr,Yr(:,4),'o')
%axis([0 30 -.1 .1])
%ylabel('angles in rad')
%legend('Beta','P','R','Phi')
%xlabel('time sec')
%title('Rudder Impulse')
subplot(411)
plot(Tr,Yr(:,1),'-','LineWidth',2);grid
ylabel('\beta rad')
title('Rudder 1 deg step')
subplot(412)
plot(Tr,Yr(:,2),'-','LineWidth',2);grid
ylabel('p rad/sec')
subplot(413)
plot(Tr,Yr(:,3),'-','LineWidth',2);grid
ylabel('r rad/sec')
subplot(414)
plot(Tr,Yr(:,4),'-','LineWidth',2);grid
ylabel('\phi rad')
%plot(Ta,Ya(:,1),'-',Ta,Ya(:,2),'--',Ta,Ya(:,3),'^',Ta,Ya(:,4),'o')
%axis([0 20 -.25 .1])
%axis([0 30 -.02 .1]);grid
%legend('Beta','P','R','Phi')
xlabel('time sec')
print -depsc ac2_fig2

figure(6);clf;
subplot(411)
plot(Ta,Ya(:,1),'-','LineWidth',2);grid
ylabel('\beta rad')
title('Aileron 1 deg Impulse - 2sec on then off')
subplot(412)
plot(Ta,Ya(:,2),'-','LineWidth',2);grid
ylabel('p rad/sec')
legend('\delta_a > 0 ==> right wing up')
subplot(413)
plot(Ta,Ya(:,3),'-','LineWidth',2);grid
ylabel('r rad/sec')
legend('Initial adverse yaw ==> RY coupling')
subplot(414)
plot(Ta,Ya(:,4),'-','LineWidth',2);grid
ylabel('\phi rad')
%plot(Ta,Ya(:,1),'-',Ta,Ya(:,2),'--',Ta,Ya(:,3),'^',Ta,Ya(:,4),'o')
%axis([0 20 -.25 .1])
%axis([0 30 -.02 .1]);grid
%legend('Beta','P','R','Phi')
xlabel('time sec')
print -depsc ac2_fig2a

clear Gvda Gpda Grda Gvdr Gpdr Grdr Gphda Gphdr
Gvda(:,1)=G(1,1,:);
Gpda(:,1)=G(2,1,:);
Grda(:,1)=G(3,1,:);
Gphda(:,1)=G(4,1,:);
Gvdr(:,1)=G(1,2,:);
Gpdr(:,1)=G(2,2,:);
Grdr(:,1)=G(3,2,:);
Gphdr(:,1)=G(4,2,:);

figure(1);clf
orient('landscape')
subplot(231)
loglog(freq,abs(Gvda),min(abs(ev))*[1 1],[.1 1e4],'--',max(abs(ev))*[1 1],[.1 1e4],'--','LineWidth',2)
axis([.01 5 1e-2 1e2]) 
ylabel('|G_\beta_d_a|')
xlabel('Freq (rad/sec)')
subplot(232)
loglog(freq,abs(Gpda),min(abs(ev))*[1 1],[.1 1e4],'--',max(abs(ev))*[1 1],[.1 1e4],'--','LineWidth',2)
axis([.01 5 1e-2 1e2]) 
ylabel('|G_p_d_a|')
xlabel('Freq (rad/sec)')
title('Transfer function from aileron to flight variables')
subplot(233)
loglog(freq,abs(Grda),min(abs(ev))*[1 1],[.1 1e4],'--',max(abs(ev))*[1 1],[.1 1e4],'--','LineWidth',2)
ylabel('|G_r_d_a|')
xlabel('Freq (rad/sec)')
axis([.01 5 .01 1e2]) 
subplot(234)
semilogx(freq,180/pi*phase(Gvda.'),min(abs(ev))*[1 1],[-360 0],'--',max(abs(ev))*[1 1],[-360 0],'--','LineWidth',2)
ylabel('arg G_\beta_d_a')
xlabel('Freq (rad/sec)')
axis([.01 5 -200 200]) 
subplot(235)
semilogx(freq,180/pi*phase(Gpda.'),min(abs(ev))*[1 1],[-360 0],'--',max(abs(ev))*[1 1],[-360 0],'--','LineWidth',2)
ylabel('arg G_p_d_a')
xlabel('Freq (rad/sec)')
axis([.01 5 -360 0]) 
subplot(236)
semilogx(freq,180/pi*phase(Grda.'),min(abs(ev))*[1 1],[-360 0],'--',max(abs(ev))*[1 1],[-360 0],'--','LineWidth',2)
ylabel('arg G_r_d_a')
xlabel('Freq (rad/sec)')
axis([.01 5 -200 200]) 
print -depsc ac2_fig1

figure(2);clf
orient('landscape')
subplot(231)
loglog(freq,abs(Gvdr),min(abs(ev))*[1 1],[.1 1e4],'--',max(abs(ev))*[1 1],[.1 1e4],'--','LineWidth',2)
axis([.01 5 1 1e4]) 
ylabel('|G_\beta_d_a|')
xlabel('Freq (rad/sec)')
subplot(232)
loglog(freq,abs(Gpdr),min(abs(ev))*[1 1],[.1 1e4],'--',max(abs(ev))*[1 1],[.1 1e4],'--','LineWidth',2)
axis([.01 5 1e-2 1e2]) 
ylabel('|G_p_d_a|')
xlabel('Freq (rad/sec)')
title('Transfer function from rudder to flight variables')
subplot(233)
loglog(freq,abs(Grdr),min(abs(ev))*[1 1],[.1 1e4],'--',max(abs(ev))*[1 1],[.1 1e4],'--','LineWidth',2)
ylabel('|G_r_d_a|')
xlabel('Freq (rad/sec)')
axis([.01 5 .01 100]) 
subplot(234)
semilogx(freq,180/pi*phase(Gvdr.'),min(abs(ev))*[1 1],[-360 0],'--',max(abs(ev))*[1 1],[-360 0],'--','LineWidth',2)
ylabel('arg G_\beta_d_a')
xlabel('Freq (rad/sec)')
axis([.01 5 -200 200]) 
subplot(235)
semilogx(freq,180/pi*phase(Gpdr.'),min(abs(ev))*[1 1],[-360 0],'--',max(abs(ev))*[1 1],[-360 0],'--','LineWidth',2)
ylabel('arg G_p_d_a')
xlabel('Freq (rad/sec)')
axis([.01 5 -540 0]) 
subplot(236)
semilogx(freq,180/pi*phase(Grdr.'),min(abs(ev))*[1 1],[-360 0],'--',max(abs(ev))*[1 1],[-360 0],'--','LineWidth',2)
ylabel('arg G_r_d_a')
xlabel('Freq (rad/sec)')
axis([.01 5 -360 200]) 
print -depsc ac2_fig1a
%
%return

!eps2pdf /f="ac2_fig1.eps"
!mv c:\ac2_fig1.pdf ./ac2_fig1.pdf
!eps2pdf /f="ac2_fig1a.eps"
!mv c:\ac2_fig1a.pdf ./ac2_fig1a.pdf
!eps2pdf /f="ac2_fig2.eps"
!mv c:\ac2_fig2.pdf ./ac2_fig2.pdf
!eps2pdf /f="ac2_fig2a.eps"
!mv c:\ac2_fig2a.pdf ./ac2_fig2a.pdf
return
