package frc.robot.autos;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForLimelight;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.Swerve;

public class TurnAuto extends PPAutoBase{
    
    private DriveForLimelight driveForLimelight;
    private Limelight limelight;
    private Swerve swerve;
    private TeleopSwerve teleopSwerve;
    private DoubleSupplier zero = () -> 0;
    private BooleanSupplier False = () -> false;
    private BooleanSupplier True = () -> true;
    private PIDPivot pidPivot;
    public TurnAuto(Swerve swerve, Limelight limelight, TeleopSwerve teleopSwerve, DriveForLimelight driveForLimelight, PIDPivot pidPivot) {
        super(swerve);
        addCommands(
            new TeleopSwerve(swerve, zero, zero, zero, False, False, True, False, False, False, pidPivot).withTimeout(0.5),
            new DriveForLimelight(limelight, swerve)
        );
    }
}
