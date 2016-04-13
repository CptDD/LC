%claculate the differences between the Q and Q optim

gridnav_nearoptsol
diffseq=0;
a=size(Qstar);
for i=1:conf.T
    diff=Qseq{i}-Qstar;
    
    Qdi=Qdiff(diff,a(3));
    diffseq=[diffseq Qdi];
end
diffseq=diffseq(2:end);
plot(diffseq);
grid;