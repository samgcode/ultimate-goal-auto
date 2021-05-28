package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.auto.actions.*;

public class Scheduler {
    private Action[] actions;
    public int index = 0;
    private Action action;
    
    
    public void start() {
        action = actions[index];
        action.init();
    }
    
    public void setActions(Action[] _actions) {
        actions = _actions;
    }
    
    public void loop() {
        if(action != null) {
            action.loop();
            if(action.isFinished()) {
                action.end();
                if(index < actions.length-1) {
                    index++;
                    action = actions[index];
                    action.init();
                } else {
                    action = null;
                }
            }
        }
    }
}


























