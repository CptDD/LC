function NaiveFunction(x,maxIter,nrOfActions,model) 

    global viscfg;

    %for i=1:maxIter
    terminal=0;
        while(terminal==0)
        action=randi(nrOfActions);
        [xplus, rplus, terminal] = gridnav_mdp(model,x,action);
        terminal
        viscfg.x=xplus;
        viscfg.gview=gridnav_visualize(viscfg);
        x=xplus;
        pause(0.1);
        if(terminal==1)
           break;
        end
    end
end