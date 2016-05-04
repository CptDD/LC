clear all
close all
clc
K=35;
N=5;
adp_example;
exectimes=[]
exectimes=[exectimes exectime];
i1=0;
   for i=linspace(-pi,pi,K)
       i1=i1+1;
       j1=0;
       for j=linspace(-15*pi,15*pi,K)
           j1=j1+1;
           k1=0;
           for k=linspace(-3,3,M)
               k1=k1+1;
                x=[i1,j1];
                u=k;
                
                Qopt(i1,j1,k1) = optapprox.q(optapprox, opttheta, x, u);
                Hopt(i1,j1) = optapprox.h(optapprox, opttheta, x);
                
           
           end
       end
   end

  Qres=[];
  Hres=[];
  N1=15:20;

    
for N=N1
   
   adp_example;
   exectimes=[exectimes exectime];
   close all;
   i1=0;
   for i=linspace(-pi,pi,K)
       i1=i1+1;
       j1=0;
       for j=linspace(-15*pi,15*pi,K)
           j1=j1+1;
           k1=0;
           for k=linspace(-3,3,M)
               k1=k1+1;
                x=[i1,j1];
                u=k;
                
                Qcalc(i1,j1,k1) = fzapprox.q(fzapprox, fztheta, x, u);
                Hcalc(i1,j1) = fzapprox.h(fzapprox, fztheta, x);
                %(Qcalc(i,j,k)-Qopt(i,j,k))^2;
           
           end
       end
   end
   Qdiff=(Qcalc-Qopt);
   Hdiff=(Hcalc-Hopt);
   Qres=[Qres 1/(K^2*M)*(sum(sum(sum(Qdiff.^2))))];
   Hres=[Hres 1/(K^2*M)*(sum(sum(Hdiff.^2)))];
    
end

figure
plot(N1,Qres);
title('Q distance')
grid
figure
plot(N1,Hres);
title('h distance')
grid
figure
plot(1:length(exectimes),exectimes)
title('Execution times')
grid