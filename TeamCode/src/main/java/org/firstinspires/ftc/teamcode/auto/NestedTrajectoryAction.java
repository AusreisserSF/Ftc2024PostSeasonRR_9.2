package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//**TODO Need a full explanation
// of why it's necessary - to get the ending pose from the previous
// trajectory after that trajectory has completed. The pose is
// stored in the MecanumDrive class.
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

            RobotLog.dd("NestedAction", "Starting pose " + drive.pose);
            TrajectoryActionCollection.buildTrajectoryAction(drive, drive.pose, trajectoryActionId);
        }

        if (!action.run(packet)) {
            RobotLog.dd("NestedAction", "Ending pose " + drive.pose);
            return false; // keep going
        }

        return true; // done
    }

}
