function error = Qdiff( Q,nrOfActions )
error=0;
for i=1:nrOfActions
    
 error=error + norm(squeeze(Q(:,:,i)));
end


end
