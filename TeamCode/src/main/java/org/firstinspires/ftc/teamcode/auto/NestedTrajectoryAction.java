package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.function.Function;

// This class is necessary when we need to get the ending pose from the
// previous trajectory after that trajectory has completed. That pose,
// which is stored in the MecanumDrive class, is only available at the
// time the run method below is invoked.
public class NestedTrajectoryAction implements Action {

    private final MecanumDrive drive;
    private final TrajectoryActionCollection.TrajectoryActionId trajectoryActionId;
    private Action action; // this is the nested action

    public NestedTrajectoryAction(MecanumDrive pDrive, TrajectoryActionCollection.TrajectoryActionId pTrajectoryActionId) {
        drive = pDrive;
        trajectoryActionId = pTrajectoryActionId;
    }

    private boolean initialized = false;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            initialized = true;

            //## A more generic - but maybe too generic - way of capturing a
            // reference to the actionBuilder method in MecanumDrive.
            // Function<Pose2d, TrajectoryActionBuilder> tab = drive::actionBuilder;

            RobotLog.dd("NestedAction", "Starting pose " + drive.pose);

            // We need a reference to MecanumDrive to access to the method
            // actionBuilder. We also extract the pose from the same instance -
            // needed for compatibility with other callers that may supply a
            // hard-coded pose to buildTrajectoryAction.
            TrajectoryActionCollection.buildTrajectoryAction(drive, drive.pose, trajectoryActionId);
        }

        if (!action.run(packet)) {
            RobotLog.dd("NestedAction", "Ending pose " + drive.pose);
            return false; // keep going
        }

        return true; // done
    }

}
