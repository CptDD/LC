function [hmat,Q] = QIteration( gamma,gridSize,nrOfActions, error, model)
    
    Q=zeros(gridSize(1),gridSize(2),nrOfActions);
    CurrentErr=10;
    while(CurrentErr>error)

            for k=1:gridSize(1)
                for m=1:gridSize(2)
                     for j=1:nrOfActions
                         [xplus, rplus, terminal] = gridnav_mdp(model, [k;m], j);
                         k_=xplus(1);
                         m_=xplus(2);
                         Qnew(k,m,j)=rplus+gamma*max(Q(k_,m_,:));

                     end
                end
            end
     pause(0.01);
     %Qnew
     %pause
     CurrentErr=Qdiff(Q-Qnew,gridSize);
     Q=Qnew;
     global viscfg
     viscfg.Q = Q;
     viscfg.gview = gridnav_visualize(viscfg);  
    end
    
    hmat=zeros(gridSize(1),gridSize(2));
    for k=1:gridSize(1)
        for m=1:gridSize(2)
                [val,ind]=max(Q(k,m,:));
                [xplus, rplus, terminal] = gridnav_mdp(model,[k;m], ind);
                hmat(k,m)=ind;
        end
    end

end

