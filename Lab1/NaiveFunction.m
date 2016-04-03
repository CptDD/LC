function NaiveFunction(x,maxIter,nrOfActions,model) 

    global viscfg;

    for i=1:maxIter
        action=randi(nrOfActions);
        [xplus, rplus, terminal] = gridnav_mdp(model,x,action);
        viscfg.x=xplus;
        viscfg.gview=gridnav_visualize(viscfg);
        x=xplus;
        pause(0.1);
        if(terminal==1)
           break;
        end
    end
end