// package frc.robot.autos;

// import java.io.IOException;
// import java.nio.file.Path;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants;
// import frc.robot.commands.AutoBalanceWithRoll;
// // import frc.robot.commands.PivotToPosition;
// // import frc.robot.subsystems.Collector;
// // import frc.robot.subsystems.Extender;
// // import frc.robot.subsystems.Pivot;
// import frc.robot.subsystems.Swerve;

// public class AutoBalanceAuto extends PPAutoBase {
    
//     //Swerve s_Swerve;
//     // private final String BALANCE_AUTO_PATH1 = "pathplanner/generatedJSON/ScoreAndBalancePath1.wpilib.json";
//     // private final String BALANCE_AUTO_PATH2 = "pathplanner/generatedJSON/ScoreAndBalancePath2.wpilib.json";
//     //private final String BALANCE_PATH = "pathplanner/generatedJSON/Score "
//     Trajectory trajectory;
//     Trajectory trajectory2;

//     // SwerveControllerCommand TrajectoryPath1 = baseSwerveCommand(trajectory);
//     // SwerveControllerCommand trajectoryPath2 = baseSwerveCommand(trajectory2);

//     public AutoBalanceAuto(Swerve swerve) {
//         super(swerve);
        
//         TrajectoryConfig config =
//             new TrajectoryConfig(
//                     Constants.AutoConstants.kMaxSpeedMetersPerSecond,
//                     Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                 .setKinematics(Constants.Swerve.swerveKinematics);




//         addCommands(
//             new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
//             baseSwerveCommand(trajectory),
//             new PivotToPosition(Constants.PivotSetpoints.HYBRID, pivot),
//             new InstantCommand(() -> collector.collectorOuttake()),
//             new WaitCommand(1),
//             baseSwerveCommand(trajectory2),
//             new AutoBalanceWithRoll(swerve, () -> true));
//     }
    
// }
