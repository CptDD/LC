function error = Qdiff( Q,gridSize )
error=0;
for i=1:gridSize(1)
    
 error=error + norm(squeeze(Q(i,:,:)));
end


end
