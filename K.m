function k = K (u, sigma)
    run("grandeurs.m")
    AsinCL = [0, 1, 0;
        0, 0, u;
        0, 0, 0];
    
    BsinCL = [0; 0; -K4/K5];

    p = [1, sqrt(6) * sigma, sqrt(6) * sigma^2, sigma^3];
    k = place(AsinCL, BsinCL, roots(p));
end
