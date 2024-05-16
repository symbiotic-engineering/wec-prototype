function [A,B,K,gamma,rho,g,w] = extractData(hydro)
    rho = hydro.rho;
    g = hydro.g;
    w = hydro.w;
    A = hydro.A(5,5,:);
    A = A(:)'.*(rho);
    B = hydro.B(5,5,:);
    B = B(:)'.*(rho.*w);
    K = hydro.Khs(5,5,1);
    K = K(:).*(rho.*g);
    gamma = hydro.ex_ma(5,1,:);
    gamma = gamma(:)'.*(rho.*g);
end