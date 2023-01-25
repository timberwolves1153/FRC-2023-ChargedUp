package frc.robot.autos;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.Swerve;

public class TestAuto extends SequentialCommandGroup{

    public TestAuto(Trajectory testTrajectory, Supplier<Command> swerveControllerCommand, frc.robot.subsystems.Swerve s_Swerve) {


        addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(testTrajectory.getInitialPose())),
        swerveControllerCommand.get()
        );
    }

    
    
}
