clear;clc;close;
addpath(genpath("./"));
%%
Ns = 12;
dt = 0.025;
Nsim = ceil(12 / dt);

param.Nx = 12; % dimension x
param.Nu = 12; % dimension of u
param.Nt = 20; % horizon length
param.dt = dt; % step time
param.xmax = inf(param.Nx, 1); % state constraints
param.xmin = -param.xmax;

Ib = eye(3);
Mb = eye(3) * 20;
I = [Ib, zeros(3);
    zeros(3), Mb];

param.umax = ones(param.Nu, 1) * 4000; % input constraints
param.umin = -param.umax;
param.I = I; % body inertial tensor

%% generate reference trajectory
q0_ref = [1;0;0;0];
p0_ref = [0;0;0];
w0_ref = [0;0;0] * 1;
v0_ref = [0.01;0;0] * 1;

x0_ref = [q0_ref;p0_ref];
xid_ref = [w0_ref;v0_ref];

X = eye(4);
X_ref = x0_ref';
xi_ref = xid_ref';

for i = 1:Nsim
    xid_ref_rt = xid_ref';
    Xi = [skew(xid_ref_rt(1:3)), xid_ref_rt(4:6)';...
        [0,0,0,0]];
    X = X * expm(Xi * dt);
    X_ref = [X_ref; [rotm2quat(X(1:3,1:3)), X(1:3,4)']];
    xi_ref = [xi_ref; xid_ref_rt];
end

param.X_ref = X_ref;
param.xi_ref = xi_ref;
%%
Logger1 = struct('X',zeros(13, Nsim - 11),'Err',zeros(6, Nsim - 12),'U',zeros(6, Nsim - 12),'Err2', zeros(Nsim - 12, 4));

q0 = [1;0;0;0];
p0 = [0;0;0];
w0 = [0;0;0];
v0 = [0;0;0];

Logger1 = corgi_sim(q0, p0, w0, v0, dt, Nsim, 1, param);

%%
figure(1)
axis_font_size = 16;
k = 2;
logger = Logger1;
title("Corgi MPC");
lenn = 220;

h1 = plot3(X_ref(1:lenn,5), X_ref(1:lenn,6), X_ref(1:lenn,7),'-.','color','k' ,"LineWidth",2);
hold on
h2 = plot3(logger.X(5, :), logger.X(6, :), logger.X(7, :), "-",'color',[1,1,1]/2,"LineWidth", 3);

arraw = 0.05;
lw = 2;
for kk = 1:30:length(logger.X)
    R = quat2rotm(logger.X(1:4,kk)');
    p = logger.X(5:7, kk);
    px = p + R(:,1) * arraw;
    py = p + R(:,2) * arraw;
    pz = p + R(:,3) * arraw;
    plot3([p(1), px(1)], [p(2), px(2)], [p(3), px(3)], "r-","LineWidth",lw)
    hold on
    plot3([p(1), py(1)], [p(2), py(2)], [p(3), py(3)], "g-","LineWidth",lw)
    hold on
    plot3([p(1), pz(1)], [p(2), pz(2)], [p(3), pz(3)], "b-","LineWidth",lw)
    hold on
end
box on
grid on

lw = 2;
for kk = 1:30:length(X_ref) - 30
    R = quat2rotm(X_ref(kk,1:4));
    p = X_ref(kk,5:7)';
    px = p + R(:,1) * arraw;
    py = p + R(:,2) * arraw;
    pz = p + R(:,3) * arraw;
    plot3([p(1), px(1)], [p(2), px(2)], [p(3), px(3)], "-.","color",'r', "LineWidth",lw)
    hold on
    plot3([p(1), py(1)], [p(2), py(2)], [p(3), py(3)], "-.","color",'g',"LineWidth",lw)
    hold on
    plot3([p(1), pz(1)], [p(2), pz(2)], [p(3), pz(3)], "-.","color",'b',"LineWidth",lw)
    hold on
end

daspect([1,1,1])
xlabel("$x$", "interpreter", "latex")
ylabel('$y$', "interpreter", "latex")
zlabel('$z$', "interpreter", "latex")
% ylim([-0.2,4])
set(gca,'FontSize',axis_font_size)

