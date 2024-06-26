function u = eMPC(x0, xd, xid, rs, param)
Xd = [quat2rotm(xd(1:4)), xd(5:7)';...
    0,0,0,1];
X00 = [quat2rotm(x0(1:4)'), x0(5:7);...
    0,0,0,1];

% The inverse of Left error defined in paper is equivalent. 
X00 =  X00^(-1) * Xd; 
X0 = logm(X00);

p0 = [X0(3,2); X0(1,3); X0(2,1); X0(1:3,4);x0(8:end)]; %% error

[A, bmin, bmax] = corgi_constraint(xid, p0, x0, rs, param);

param.P = idare(-A(1:param.Nx, 1:param.Nx), -A(1:param.Nx, param.Nx * (param.Nt + 1)+1:param.Nx * (param.Nt + 1)+param.Nu),param.Q,param.R,[], []);
[M, q] = corgi_cost(param.Q, param.R, param.P, xid, param);

solver = osqp;
solver.setup(M, q, A, bmin, bmax,'verbose', 0);
sol = solver.solve;
u = sol.x((param.Nt+1)*param.Nx+1:(param.Nt+1)*param.Nx+param.Nu);
end
