package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class DriveForLimelightV2 extends CommandBase{
    
    private Limelight limelight;
    private Swerve swerve;
    private double degrees;
    private double sign;
    private double startingDegrees;

    public DriveForLimelightV2(double maxRunTimeSeconds, Swerve swerve, Limelight limelight) {

        sign = Math.copySign(1, degrees);
    }
}
