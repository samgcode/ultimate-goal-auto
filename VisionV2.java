package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class VisionV2 {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    
    private static final String VUFORIA_KEY = "ARm96BH/////AAABmTXgrHDpFUg0qMuyUDdgsziLB1ffqazgVimlxAptfeja/QOXm2UA6gHRsSpCmtSD69Xa7XDhufpeLFwjYtVQYzI9fpWydJHvNlttOxJVzNz8Pwr4BafdUKwUHY0ve5BciCn32jHaBHk9YZFNCkE7M0X7zFdsW8EYsg7S4iwO+tHLoufMmhewsnHOzH5+GnxEsq8hgcOdjlW4TJveiOvk59SB+w1wQ1C/syFdS0H6GNNzxQ86l8MChRCwnczcEo9Dkx9dhL7nAl4NTXQ/cUd1jtC0UvlxG0c/U6WpYh1X3wgwsB4MPPAaU5yYwolxlQvvdnUqfp7UxaTPEGrNB5TXlWt1ohmzR9RBk0hlWkExj5N7";

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (5.25f) * mmPerInch;          // the height of the center of the target image above the floor

    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    OpenGLMatrix robotFromCamera = null;
    private VuforiaLocalizer vuforia = null;

    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;

    WebcamName webcamName = null;
    
    List<VuforiaTrackable> allTrackables = null;
    VuforiaTrackables targetsUltimateGoal = null;
    
    private TFObjectDetector tfod;

    public boolean targetVisible = false;
    public float phoneXRotate = -90;
    public float phoneYRotate = 0;
    public float phoneZRotate = -90;
    
    public final float CAMERA_FORWARD_DISPLACEMENT  = 3f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
    public final float CAMERA_VERTICAL_DISPLACEMENT = 17.5f * mmPerInch;   // eg: Camera is 8 Inches above ground
    public final float CAMERA_LEFT_DISPLACEMENT = 2.5f * mmPerInch;     // eg: Camera is ON the robot's center line
    
    private int ringCount = 0;
    
    public VectorF translation;
    public Orientation rotation;
    public VectorF camTranslation;

    public VisionV2(HardwareMap _hardwareMap, Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;
        
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        initVuforia();
        initTfod();
        
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }
    }
    
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    
    public void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;

        parameters.useExtendedTracking = false;
        
        // parameters.minWebcamAspectRatio = 1.5;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
    
     

        

        robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, phoneXRotate, phoneZRotate, phoneYRotate));

        targetsUltimateGoal.activate();
    }
    
    public void updateCameraLocation(){
        robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, phoneXRotate, phoneZRotate, phoneYRotate));

    }
    
    public void updatePosition() {
        updateCameraLocation();
        OpenGLMatrix camFromRobot = robotFromCamera.inverted();
        targetVisible = false;
        OpenGLMatrix camFromTarget = null;
        VuforiaTrackable visibleTarget = null;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;
                camFromTarget = ((VuforiaTrackableDefaultListener)trackable.getListener()).getVuforiaCameraFromTarget();
                visibleTarget = trackable;
                break;
            }
        }
        if (camFromTarget != null) {
            OpenGLMatrix targetFromCam = camFromTarget.inverted();
            OpenGLMatrix fieldFromCam = visibleTarget.getFtcFieldFromTarget().multiplied(targetFromCam);
            OpenGLMatrix fieldFromRobot = fieldFromCam.multiplied(camFromRobot);
            lastLocation = fieldFromRobot;
            
            translation = lastLocation.getTranslation();
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, ZYX, DEGREES);
            camTranslation = fieldFromCam.getTranslation();
        }
    }
    
    public int updateObjectDetection() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
              
              int i = 0;
              for (Recognition recognition : updatedRecognitions) {
                String label = recognition.getLabel();
                
                if(label == "Single") {
                    ringCount = 1;
                } else if(label == "Quad") {
                    ringCount = 4;
                } else {
                    ringCount = 0;
                }
                
              }
            }
        }
        return ringCount;
    }
}
