package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.vision.AprilTagUtils;

public class TrajectoryActionCollection {

    private static final String TAG = TrajectoryActionCollection.class.getSimpleName();

    public enum  TrajectoryActionId {F4_RED_TO_SAMPLE_1 }

    public static Action buildTrajectoryAction(MecanumDrive pDrive,
                                               TrajectoryActionId pTrajectoryActionId) {
        switch (pTrajectoryActionId) {
            case F4_RED_TO_SAMPLE_1: {
                return pDrive.actionBuilder(pDrive.pose)
                        .lineToY(-48)
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(37.11, -37.11, Math.toRadians(45)), Math.toRadians(45))
                        .waitSeconds(2)
                        .build();
            }
            default: {
                //**TODO AutonomousRobotException.
                throw new RuntimeException("Unrecognized trajectory action id " + pTrajectoryActionId);
            }
        }
    }
}
