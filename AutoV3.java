package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import org.firstinspires.ftc.teamcode.auto.VisionV2;
import org.firstinspires.ftc.teamcode.auto.RobotMap;
import org.firstinspires.ftc.teamcode.auto.Scheduler;
import org.firstinspires.ftc.teamcode.auto.actions.*;

@Autonomous

public class AutoV3 extends OpMode{
    private VisionV2 vision;
    private RobotMap robot;
    private Scheduler scheduler;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo centerDrive = null;
    private DcMotor intake = null;
    private DcMotor shooter = null;
    private Servo camZServo = null;
    private Servo camYServo = null;
    
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    
    private float ticksPerRev = 80f;
    private float ratio = 0.25f;
    private float camTickPerRev = ticksPerRev/ratio;
    private float camDegreesPerTick = 360/camTickPerRev;
    
    private int zone = 1;
    
    float x = 0;
    float y = 0;
    float heading = 0;
    float camZ = 0f;
    float camY = 0f;
    
    private float speed = 1f;
    private float xSpeed = 0f;
    private float ySpeed = 0f;
    
    private float prevTimeX = 0f;
    private float prevTimeY = 0f;
    private float prevTimeH = 0f;
    private float prevTimeCam = 0f;
    
    private float prevErrorX = 0f;
    private float prevErrorY = 0f;
    private float prevErrorH = 0f;
    private float prevErrorCZ = 0f;
    private float prevErrorCY = 0f;
    
    private float kPX = 0.2f;
    private float kDX = 0f;
    
    private float kPY = 0.2f;
    private float kDY = 0.00075f;
    
    private float kPH = 0.02f;
    private float kDH = 0.000004f;
    
    private float kPCZ = 0.029f;
    private float kDCZ = 0.000008f;
    private float kPCY = 0.029f;
    private float kDCY = 0.000008f;
    
    private float rotationPowerOffset = 0;
    
    private int ringCount = 0;
    
    private float acceptableError = 5f;
    private float acceptableHeading = 5f;
    private float acceptableCam = 5f;
    private int positionIndex = 0;
    
    private float t = 22.75f;
    
    private float acceptableJump = 10f;
    private float deltaX = 0f;
    private float deltaY = 0f;
    private float prevX = 0f;
    private float prevY = 0f;
    
    VectorF center = new VectorF(0f, 0f);
    VectorF test1 = new VectorF(50f, -36f);
    VectorF test2 = new VectorF(38f, -27f);
    VectorF homeA = new VectorF((float)-2.5f*t, (float)-0.5*t);
    VectorF homeB = new VectorF((float)-2.5f*t, (float)-t);
    VectorF home = new VectorF((float)-2.5f*t, (float)-1.5*t);
    VectorF homeC = new VectorF((float)-2.5f*t, (float)-2*t);
    VectorF homeD = new VectorF((float)-2.5f*t, (float)-2.5*t);
    VectorF line = new VectorF((float)0.5*t, (float)-t);
    VectorF highGoal = new VectorF((float)0f, (float)-1.5f*t);
    VectorF boxA = new VectorF((float)0f, (float)-2.5f*t);
    VectorF boxB = new VectorF((float)t, (float)-1.5f*t);
    VectorF boxC = new VectorF((float)2f*t, (float)-2.5f*t);
    VectorF powerA = new VectorF((float)0f, (float)-3f);
    VectorF powerB = new VectorF((float)0f, (float)-11f);
    VectorF powerC = new VectorF((float)0f, (float)-19f);
    VectorF towerTarg = new VectorF((float)3*t, (float)-1.5*t);
    VectorF wallTarg = new VectorF((float)0f, (float)-3*t);
    VectorF backTarg = new VectorF((float)-3*t, (float)0f);
    
    @Override 
    public void init() {
        vision = new VisionV2(hardwareMap, telemetry);
        robot = new RobotMap();
        scheduler = new Scheduler();

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        centerDrive = hardwareMap.get(Servo.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "center_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        // centerDrive.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        
        // shooter = hardwareMap.get(Servo.class, "shooter");
        camZServo = hardwareMap.get(Servo.class, "cam_servo_x");
        camYServo = hardwareMap.get(Servo.class, "cam_servo_y");
    }
    
    @Override
    public void init_loop() {
        ringCount = vision.updateObjectDetection();
    }
    
    @Override
    public void start() {
        if(ringCount == 4) {
            setActionsA();
        } else if(ringCount == 1) {
            setActionsA();
        } else {
            setActionsA();
        }
        scheduler.start();
    }
    
    @Override
    public void loop() {
        camZ = -(rightDrive.getCurrentPosition())*camDegreesPerTick;
        camY = (shooter.getCurrentPosition())*camDegreesPerTick;

        vision.phoneZRotate = -90+camZ;
        vision.phoneYRotate = camY;
        vision.updatePosition();
        if(vision.targetVisible) {
            if(vision.translation != null) {
                VectorF position = vision.translation;
                Orientation rotation = vision.rotation;
                if(prevX == 0 || prevY == 0) {
                    x = position.get(0)/mmPerInch;
                    y = position.get(1)/mmPerInch;
                } else {
                    deltaX = position.get(0)/mmPerInch - prevX;
                    deltaY = position.get(1)/mmPerInch - prevY;
                    if(Math.abs(deltaX) <= acceptableJump) {
                        x = position.get(0)/mmPerInch;
                    }
                    if(Math.abs(deltaY) <= acceptableJump) {
                        y = position.get(1)/mmPerInch;
                    }
                }
                heading = rotation.firstAngle;
            }
            telemetry.addData("cam rot: ", "%.1f, %.1f", camZ, camY);
            
            telemetry.addData("target visible", vision.targetVisible);
            telemetry.addData("rings", ringCount);
            telemetry.addData("pos: ", "%.1f, %.1f", x, y);
            telemetry.addData("prev pos: ", "%.1f, %.1f", prevX, prevY);
            telemetry.addData("heading: ", heading);
            telemetry.addData("phone rotate", "%.1f, %.1f, %.1f", vision.phoneXRotate, vision.phoneYRotate, vision.phoneZRotate);
            
            robot.x = x;
            robot.y = y;
            robot.heading = heading;
    
            zone = getZone(x, y);
    
            VectorF targetPosition = robot.targetPosition;
            float targX = targetPosition.get(0);
            float targY = targetPosition.get(1);
            // float targX = 0f;
            // float targY = -22.75f;
            float targHeading = robot.targetHeading;
            float camXPos = vision.camTranslation.get(0);
            float camYPos = vision.camTranslation.get(1);
            float camZPos = vision.camTranslation.get(2);
            updateCamRotation(camXPos, camYPos, camZ, camY, heading, camZPos);
    
            moveToPosition(targX, targY, 0, x, y, heading);
            // moveToPosition(0, -22.75f, 0, x, y, heading);
            
            
            telemetry.addData("target: ", "%.1f, %.1f", targX, targY);
            telemetry.addData("targHeading", targHeading);
            telemetry.addData("speed: ", "%.1f, %.1f", xSpeed, ySpeed);
            telemetry.addData("heading pid: ", rotationPowerOffset);
            telemetry.addData("Error: ", "%.1f %.1f", prevErrorX, prevErrorY);
            telemetry.addData("action: ", scheduler.index);
            telemetry.addData("at target: ", robot.atTargetPosition);
            
            
            scheduler.loop();
            if(robot.atTargetPosition) {
                robot.atTargetPosition = false;
            }
        } else {
            xSpeed = -1;
            ySpeed = 0;
            rotationPowerOffset = 0;
            camZServo.setPosition(0.5);
        }
        
        if(gamepad1.a) {
            robot.atTargetPosition = false;
        }
        // leftDrive.setPower(xSpeed);
        // rightDrive.setPower(xSpeed);
        // xSpeed = 0f;
        // ySpeed = 0f;
        // rotationPowerOffset = 0;
        
        ySpeed = mapF(ySpeed, -1f, 1f, 0f, 1f);
        leftDrive.setPower((xSpeed+rotationPowerOffset));
        rightDrive.setPower(-(xSpeed-rotationPowerOffset));
        centerDrive.setPosition(ySpeed);
        intake.setPower(robot.intakeSpeed);
        // shooter.setPosition(robot.shooterSpeed);
        
        prevX = x;
        prevY = y;
    }
    
    public void moveToPosition(float x, float y, float rotation, float posX, float posY, float currHeading) {
        if(!robot.atTargetPosition) {
            float xDist = posX - x;
            float yDist = posY - y;
            if(Math.abs(xDist) <= acceptableError && 
               Math.abs(yDist) <= acceptableError
               ) {
                robot.atTargetPosition = true;
            }
            
            float PIDX = getPID(xDist, prevErrorX, prevTimeX, kPX, 0f, kDX);
            float PIDY = getPID(yDist, prevErrorY, prevTimeY, kPY, 0f, kDY);
            normalizeSpeed(currHeading, PIDX, PIDY);
            prevErrorX = xDist;
            prevErrorY = yDist;
            prevTimeX = (float)runtime.time();
            prevTimeY = (float)runtime.time();
        } else {
            xSpeed = 0;
            ySpeed = 0;
        }
        if(rotation != currHeading) {
            float error = currHeading - rotation;
            if(Math.abs(error) > acceptableHeading) {
                rotationPowerOffset = getPID(error, prevErrorH, prevTimeH, kPH, 0, kDH);
                prevTimeH = (float)runtime.time();
                prevErrorH = error;   
            } else {
                rotationPowerOffset = 0;
            }
        }
    }
    
    public void updateCamRotation(float x, float y, float camZ, float camY, float heading, float camHeight) {
        VectorF targ = towerTarg;
        if(zone == 2) {
            targ = wallTarg;
        } else if(zone == 3) {
            targ = backTarg;
        }
        float camTargetZ = getCamAngleX(x, y, targ);
        float camTargetY = getCamAngleY(x, y, targ, camHeight);
        float zDiff = camTargetZ-heading;
        float zError = zDiff-camZ;
        float yError = camTargetY - camY;
        
        float zServoSpeed = getPID(zError, prevErrorCZ, prevTimeCam, kPCZ, 0f, kDCZ);
        float yServoSpeed = getPID(yError, prevErrorCY, prevTimeCam, kPCY, 0f, kDCY);
        zServoSpeed = mapF(zServoSpeed, -1f, 1f, 1f, 0f);
        zServoSpeed = Range.clip(zServoSpeed, 0.25f, 0.75f);
        yServoSpeed = mapF(yServoSpeed, -1f, 1f, 1f, 0f);
        yServoSpeed = Range.clip(yServoSpeed, 0.25f, 0.75f);
        if(Math.abs(zError) > acceptableCam) {
            camZServo.setPosition(zServoSpeed);
        } else {
            camZServo.setPosition(0.5);
        }
        
        if(Math.abs(yError) > acceptableCam) {
            camYServo.setPosition(yServoSpeed);
        } else {
            camYServo.setPosition(0.5);
        }
        prevErrorCZ = zDiff;
        prevErrorCY = yError;
        prevTimeCam = (float)runtime.time();
    }
    
    public float getCamAngleX(float x, float y, VectorF picture) {
        float xDist = x - picture.get(0);
        float yDist = y - picture.get(1);
        
        float angle = (float)Math.toDegrees(Math.atan(yDist/xDist));
        
        return angle;
    }
    
    public float getCamAngleY(float x, float y, VectorF picture, float camHeight) {
        float targHeight = 5.5f;
        float dist = getDist(x, y, picture);
        
        float angle = (float)Math.toDegrees(Math.atan(dist/(camHeight - targHeight)));
        return angle;
    }
    
    public float getPID(float dist, float prevError, float prevTime, float kP, float kI, float kD) {
        float time = (float)runtime.time();
        float deltaTime = time - prevTime;
        float error = dist;

        float p = kP * error;
        
        float i = kI * (error * deltaTime);
        i = Range.clip(i, -speed, speed);
        
        float d = kD * (error - prevError) / deltaTime;

        float pid = p + i + d;
        return Range.clip(pid, -speed, speed);
    }
    
    public void normalizeSpeed(float heading, float PIDX, float PIDY) {
        float angle = (float)Math.toRadians(heading);
        ySpeed = (float)(-(PIDX * Math.sin(angle)) + (PIDY * Math.cos(angle)));
        xSpeed = (float)((PIDX * Math.cos(angle)) + (PIDY * Math.sin(angle)));
    }
    
    public void setActionsA() {
        // Action[] actions = {
        //     //wobble goal a
        //     new MoveTo(homeC, 90, robot),
        //     new MoveTo(homeD, 90, robot),
        //     new MoveTo(boxA, 90, robot),
        //     //power shot a
        //     new MoveTo(powerA, 90, robot),
        //     new ActivateShooter(robot),
        //     new DeactivateShooter(robot),
        //     //power shot b
        //     new MoveTo(powerB, 90, robot),
        //     new ActivateShooter(robot),
        //     new DeactivateShooter(robot),
        //     //power shot c
        //     new MoveTo(powerC, 90, robot),
        //     new ActivateShooter(robot),
        //     new DeactivateShooter(robot),
        //     //wobble goal b
        //     new MoveTo(homeA, 90, robot),
        //     new MoveTo(homeB, 90, robot),
        //     new MoveTo(boxA, 112, robot),
        //     //line
        //     new MoveTo(line, 90, robot)
        // };
        Action[] actions = {
            new MoveTo(homeC, 90, robot),
        };
        
        scheduler.setActions(actions);
    }
    
    public float mapF(float value, float from1, float to1, float from2, float to2) {
        return (value-from1) / (to1-from1) * (to2 - from2) + from2;
    }
    
    public float getDist(float x, float y, VectorF target) {
        float xDist = target.get(0) - x;
        float yDist = target.get(1) - y;
        float dist = (float)Math.sqrt(((xDist * xDist) - (yDist * yDist)));
        return dist;
    }
    
    public int getZone(float x, float y) {
        float towerDist = getDist(x, y, towerTarg);
        float wallDist = getDist(x, y, wallTarg);
        float backDist = getDist(x, y, backTarg);
        
        if(towerDist < 2.5 && towerDist > 1) {
            return 1;
        } else if(wallDist < 2 && wallDist > 1) {
            return 2;
        } else if(backDist < 2 && backDist > 1) {
            return 3;    
        } else {
            return smallest(towerDist, wallDist, backDist);
        }
    }
    
    public int smallest(float a, float b, float c) {
        if(a <= b) {
            if(a <= c) {
                return 1;
            } else {
                return 3;
            }
        } else {
            if(b <= c) {
                return 2;
            } else {
                return 3;
            }
        }
    }
}










































