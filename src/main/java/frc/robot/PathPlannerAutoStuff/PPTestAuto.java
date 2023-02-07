package frc.robot.PathPlannerAutoStuff;


import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class PPTestAuto extends PPAutoBase{
    

    public PPTestAuto(Swerve swerve) {
        super(swerve);

        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("ScoreAndBalancePath1", 1.0, 1.0);
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("ScoreAndBalancePath2", 1.5, 1.0);

        PPSwerveControllerCommand autoDrive = baseSwerveCommand(trajectory1);
        PPSwerveControllerCommand autoDrive2 = baseSwerveCommand(trajectory2);

        PathPlannerState initialState = trajectory1.getInitialState();



        addCommands(
            new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(() -> swerve.resetOdometry(trajectory1.getInitialPose())),
            autoDrive,
            autoDrive2        
            );
    }
}
