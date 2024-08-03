
// From https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/

package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@Autonomous(name = "RR_BLUE_TEST_AUTO", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        //## From the documentation:
        // Make sure your MecanumDrive is instantiated at the correct pose.
        // If you end up using lineToX(), lineToY(), strafeTo(), splineTo(),
        // or any of their variants in your code, if the initial pose is wrong,
        // all future movements will be thrown off. 
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));

        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        Action trajectoryActionCloseOut;

        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3)
                .build();

        trajectoryAction2 = drive.actionBuilder(drive.pose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3)
                .build();

        trajectoryAction3 = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3)
                .build();

        trajectoryActionCloseOut = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(48, 12))
                .build();

        //**TODO Initialize AprilTag recognition.

        telemetry.addLine("Waiting for start ...");
        telemetry.update();

        waitForStart();

       // vision here that outputs position
        //**TODO Use enum for LEFT, CENTER, RIGHT
        //**TODO Use AprilTag enum
       int visionOutputPosition = 2; // center spike
        Action trajectoryActionChosen;
        switch (visionOutputPosition) {
            case 1: {
                trajectoryActionChosen = trajectoryAction1;
                break;
            }
            case 2: {
                trajectoryActionChosen = trajectoryAction2;
                break;
            }
            case 3: {
                trajectoryActionChosen = trajectoryAction3;
                break;
            }
            default: {
                telemetry.addData("Invalid vision position", visionOutputPosition);
                telemetry.update();
                return;
            }
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        //**TODO Need Action that gets the distance and angle to
                        // the AprilTag as well as its field coordinates.
                        // Telemetry and log.
                        trajectoryActionCloseOut
                )
        );
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        //##PY in the sample all are commented out ...
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //##PY .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                // ... these parameters are fx, fy, cx, cy.
                // ##PY for Logitech Brio from the 3DF Zephyr tool
                //.setLensIntrinsics(627.419488832, 627.419488832, 301.424062225, 234.042415697)
                //#PY for Logitech C920 from the FTC file teamwebcamcalibrations.xml
                //.setLensIntrinsics(622.001, 622.001, 319.803, 241.251)
                //##PY for Arducam 120fps Mono Global Shutter USB Camera, 720P OV9281 UVC Webcam Module
                //.setLensIntrinsics(539.024, 539.024, 316.450, 236.365)
                //##PY for Logitech C920 from the 3DF Zephyr tool
                .setLensIntrinsics(625.838, 625.838, 323.437, 240.373)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(false);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //##PY try MJEPG
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        builder.enableLiveView(false); //##PY - added

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        RobotLog.d("ConceptAprilTag", "Waiting for webcam to start streaming");
        ElapsedTime streamingTimer = new ElapsedTime();
        streamingTimer.reset(); // start
        while (streamingTimer.milliseconds() < 2000 && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(50);
        }

        VisionPortal.CameraState cameraState = visionPortal.getCameraState();
        if (cameraState != VisionPortal.CameraState.STREAMING) {
            throw new RuntimeException("Timed out waiting for webcam streaming to start");
        }

        // Start with the processor disabled.
        visionPortal.setProcessorEnabled(aprilTag, false);
    }   // end method initAprilTag()
}