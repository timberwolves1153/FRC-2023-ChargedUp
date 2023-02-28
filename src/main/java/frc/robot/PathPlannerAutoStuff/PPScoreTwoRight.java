package frc.robot.PathPlannerAutoStuff;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.AutoBase;
import frc.robot.subsystems.Swerve;

public class PPScoreTwoRight extends PPAutoBase{
    

    public PPScoreTwoRight(Swerve swerve /*Extender extender, Pivot pivot, Collector collector */) {
        super(swerve);
        PathPlannerTrajectory path1 = PathPlanner.loadPath("Score2RightPart1", new PathConstraints(4, 3));
        PathPlannerTrajectory path2 = PathPlanner.loadPath("Score2RightPart2", new PathConstraints(4, 3));
        PathPlannerTrajectory path3 = PathPlanner.loadPath("Score2RightPart3", new PathConstraints(4, 3));
        PathPlannerTrajectory path4 = PathPlanner.loadPath("Score2RightPart4", new PathConstraints(4, 3));
        PathPlannerTrajectory path5 = PathPlanner.loadPath("Score2RightPart5", new PathConstraints(4, 3));
        addCommands(
        // Wait commands to be added when necessary
            // pivot to L3
            // extend to L3
            followTrajectoryCommand(path1, true),
            // outtake cube/cone
            // wait command
            // stop outtake
            followTrajectoryCommand(path2, false),
            // extend to ground cube/cone setpoint
            // pivot to ground setpoint
            // wait command
            followTrajectoryCommand(path3, false),
            // intake cone/cube
            // pivot down - cube/cone on bumper
            // wait command
            followTrajectoryCommand(path4, true),
            // pivot to L3
            // extend to L3
            // outtake cube/cone
            // wait command
            // extend to starting config
            // pivot to ground
            followTrajectoryCommand(path5, false)
            // remember to turn off everything in robot.java
        );
    }
}
