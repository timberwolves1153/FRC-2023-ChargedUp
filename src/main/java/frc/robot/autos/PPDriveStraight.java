package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.subsystems.Swerve;

public class PPDriveStraight extends PPAutoBase{
    

    public PPDriveStraight(Swerve swerve) {
        super(swerve);

        PathPlannerTrajectory path1 = PathPlanner.loadPath("DriveStraighPath", new PathConstraints(3.0, 2.0));

        addCommands(
            followTrajectoryCommand(path1, true)
        );
    }
}
