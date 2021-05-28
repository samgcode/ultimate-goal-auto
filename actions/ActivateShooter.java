package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.auto.RobotMap;


public class ActivateShooter extends Action {
    
    private RobotMap robot;
    private boolean finished = false;
    
    public ActivateShooter(RobotMap _robot) {
        robot = _robot;
    }
    
    @Override
    public void init() {
        robot.shooterSpeed = 1f;
    }
    
    @Override
    public void loop() {
        finished = true;
    }
    
    @Override
    public boolean isFinished() {
        return finished;
    }
    
    @Override
    public void end() {

    }
}















