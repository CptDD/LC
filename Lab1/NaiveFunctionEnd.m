function NaiveFunctionEnd(x,nrOfActions,model) 

    global viscfg;

    	terminal=0;
        while(terminal==0)
        action=randi(nrOfActions);
        [xplus, rplus, terminal] = gridnav_mdp(model,x,action);
        viscfg.x=xplus;
        viscfg.gview=gridnav_visualize(viscfg);
        x=xplus;
        pause(0.1);
    end
end