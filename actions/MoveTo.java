package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.auto.RobotMap;


public class MoveTo extends Action {
    
    private RobotMap robot;
    private VectorF position;
    private float heading;
    private boolean finished = false;
    
    public MoveTo(VectorF _position, float _heading, RobotMap _robot) {
        robot = _robot;
        position = _position;
        heading = _heading;
    }
    
    @Override
    public void init() {
        robot.targetPosition = position;
        robot.targetHeading = heading;
    }
    
    @Override
    public void loop() {
        finished = robot.atTargetPosition;
    }
    
    @Override
    public boolean isFinished() {
        return finished;
    }
    
    @Override
    public void end() {
        
    }
}















