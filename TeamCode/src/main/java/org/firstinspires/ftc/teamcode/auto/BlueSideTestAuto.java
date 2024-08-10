
// From https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/

package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.firstinspires.ftc.teamcode.auto.vision.AprilTagUtils;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name = "RR_BLUE_TEST_AUTO", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {

    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        //## From the documentation:
        // Make sure your MecanumDrive is instantiated at the correct pose.
        // If you end up using lineToX(), lineToY(), strafeTo(), splineTo(),
        // or any of their variants in your code, if the initial pose is wrong,
        // all future movements will be thrown off. 
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(270)));

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
                .lineToY(35)
                .setTangent(Math.toRadians(0))
                .lineToX(16)
                .waitSeconds(2)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(0))
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

        initAprilTag();

        telemetry.addLine("Waiting for start at BLUE_A4 ...");
        telemetry.update();

        waitForStart();

        // Assume that the following AprilTag id number comes from the vision system.
        AprilTagUtils.AprilTagId targetTagId = AprilTagUtils.AprilTagId.TAG_ID_2;
        Action trajectoryActionChosen;
        switch (targetTagId) {
            case TAG_ID_1: {
                trajectoryActionChosen = trajectoryAction1;
                break;
            }
            case TAG_ID_2: {
                trajectoryActionChosen = trajectoryAction2;
                break;
            }
            case TAG_ID_3: {
                trajectoryActionChosen = trajectoryAction3;
                break;
            }
            default: {
                telemetry.addData("Invalid blue backdrop AprilTag target", targetTagId);
                telemetry.update();
                return;
            }
        }

        //**TODO This SequentialAction should work.
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        new BackdropAprilTagDetection(targetTagId) //, // Action for AprilTag detection
                        //**TODO stop at backdrop for now trajectoryActionCloseOut
                )
        );

        //**TODO The next two lines do work.
        //Actions.runBlocking(trajectoryActionChosen);
        //Actions.runBlocking(detection);
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //##PY .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters,
                // the SDK will attempt to load a predefined calibration
                // for your camera.
                // ... these parameters are fx, fy, cx, cy.

                //##PY for Arducam 120fps Mono Global Shutter USB Camera, 720P OV9281 UVC Webcam Module
                //.setLensIntrinsics(539.024, 539.024, 316.450, 236.365)

                //**TODO For LCHS use the calibration for the Logitech C270 from the
                // FTC file teamwebcamcalibrations.xml
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)

                //**TODO for Logitech C920 from the FTC file teamwebcamcalibrations.xml
                //.setLensIntrinsics(622.001, 622.001, 319.803, 241.251)
                //##PY for Logitech C920 from the 3DF Zephyr tool
                //.setLensIntrinsics(625.838, 625.838, 323.437, 240.373)

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
        VisionPortal visionPortal = builder.build();

        telemetry.addLine("Waiting for webcam to start streaming");
        telemetry.update();
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
    }   // end method initAprilTag()

    private class BackdropAprilTagDetection implements Action {

        private final int targetTagId;

        public BackdropAprilTagDetection(AprilTagUtils.AprilTagId pTargetTagId) {
            targetTagId = pTargetTagId.getNumericId();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            RobotLog.d("BackdropAprilTagDetection.run() entered");

            // Step through the list of detected tags and look for a matching tag.
            List<AprilTagDetection> currentDetections = aprilTag.getFreshDetections();
            AprilTagDetection targetDetection = null;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == targetTagId) {
                    targetDetection = detection;
                    break; // don't look any further.
                }
            }

            if (targetDetection == null)
                RobotLog.d("AprilTag not found");
            else
                RobotLog.d("AprilTag found " + targetDetection.id);

            telemetryAprilTag(targetDetection);
            return false; // only run once
        }
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag(AprilTagDetection pDetection) {
        if (pDetection != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", pDetection.id, pDetection.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", pDetection.ftcPose.x, pDetection.ftcPose.y, pDetection.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", pDetection.ftcPose.pitch, pDetection.ftcPose.roll, pDetection.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", pDetection.ftcPose.range, pDetection.ftcPose.bearing, pDetection.ftcPose.elevation));

            // Also write to log.
            RobotLog.d(String.format("\n==== (ID %d) %s", pDetection.id, pDetection.metadata.name));
            RobotLog.d(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", pDetection.ftcPose.x, pDetection.ftcPose.y, pDetection.ftcPose.z));
            RobotLog.d(String.format("PRY %6.1f %6.1f %6.1f  (deg)", pDetection.ftcPose.pitch, pDetection.ftcPose.roll, pDetection.ftcPose.yaw));
            RobotLog.d(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", pDetection.ftcPose.range, pDetection.ftcPose.bearing, pDetection.ftcPose.elevation));

        } else {
            telemetry.addLine("Requested AprilTag was not detected");
            RobotLog.d("Requested AprilTag was not detected");
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        telemetry.update();

    }   // end method telemetryAprilTag()
}