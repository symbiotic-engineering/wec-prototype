function [A,B,K,gamma,rho,g,w] = extractData(dof)
% dof: 3 for heave, 5 for surge
    if dof == 5
        wamit_file = 'oswec.out';
    elseif dof == 3
        wamit_file = 'rm3.out';
    end

    hydro = struct();
    hydro = readWAMIT(hydro,wamit_file,[]); % function from WECSim

    rho = hydro.rho;
    g = hydro.g;
    w = hydro.w;
    A = hydro.A(dof,dof,:);
    A = A(:)'.*(rho);
    B = hydro.B(dof,dof,:);
    B = B(:)'.*(rho.*w);
    K = hydro.Khs(dof,dof,1);
    K = K(:).*(rho.*g);
    gamma = hydro.ex_ma(dof,1,:);
    gamma = gamma(:)'.*(rho.*g);
end