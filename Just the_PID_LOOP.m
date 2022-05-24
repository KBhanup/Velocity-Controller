P_d = 0.6;
for j= 2:1781
    e_k = P_d - P_c(j);
    e_p(j) = e_k(j-1);
    

end