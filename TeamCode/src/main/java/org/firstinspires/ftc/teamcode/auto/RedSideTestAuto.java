
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
@Autonomous(name = "RR_RED_TEST_AUTO", group = "Autonomous")
public class RedSideTestAuto extends LinearOpMode {

    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(14.76, -62.89, Math.toRadians(90)));

        Action toSubmersible = drive.actionBuilder(drive.pose)
                .splineToSplineHeading(new Pose2d(6.44, -35.0, Math.toRadians(90)), Math.toRadians(90))
                .build();

        Action hangSpecimen = drive.actionBuilder(drive.pose)
                .waitSeconds(2)
                .build();

        Action toSample1 = drive.actionBuilder(drive.pose)
                .lineToY(-48)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(37.11, -37.11, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(2)
                .build();

        //##PY Commented out for the basic Notre Dame robot without a camera.
        //initAprilTag();

        telemetry.addLine("Waiting for start at RED_F3 ...");
        telemetry.update();

        waitForStart();

        //!! Note that the first Action ina SequjentialAction does not
        // need to be nested because drive.pose at this point is set
        // be the constructor of MecanumDrive. Note also that any Action
        // that does not rely on drive.pose - such as hangSpeciment -
        // does not need to be nested. But after the first Action that
        // moves the robot *all* subsequent Actions that also move the
        // robot need to be nested.
        Actions.runBlocking(
                new SequentialAction(
                        toSubmersible,
                        hangSpecimen,
                        new NestedAction(drive, TrajectoryActionCollection.TrajectoryActionId.F4_RED_TO_SAMPLE_1)
                )
        );
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