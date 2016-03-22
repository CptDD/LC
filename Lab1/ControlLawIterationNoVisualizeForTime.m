function  ControlLawIterationNoVisualizeForTime( gamma,gridSize,nrOfActions, error, model)
CurrentErr=10;
h= ones(gridSize(1),gridSize(2));
while( CurrentErr>error)
    %initial control law
    CurrentQErr=10;
    Q=zeros(gridSize(1),gridSize(2),nrOfActions);
    while(CurrentQErr>error)
        for k=1:gridSize(1)
                for m=1:gridSize(2)
                     for j=1:nrOfActions
                      
                         [xplus, rplus, terminal] = gridnav_mdp(model, [k;m], j);
                         k_=xplus(1);
                         m_=xplus(2);
                         Qnew(k,m,j)=rplus+gamma*Q(k_,m_,h(k_,m_));
                     end
                end
        end
        CurrentQErr=Qdiff(Q-Qnew,gridSize);
        Q=Qnew;
        
    end
    %find the new control law
    for k=1:gridSize(1)
        for m=1:gridSize(2)
            
            [val,indx]=max(Q(k,m,:));
            hnew(k,m)=indx;
        end
    end
    CurrentErr=norm(h-hnew);
    h=hnew;
    
end
end


