package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class DriveForLimelight extends CommandBase{
    

    private PIDController strafeController;
    private double headingError;
    private Limelight limelight;
    private double targetDistance;
    private Swerve swerve;

    public DriveForLimelight(Limelight limelight, Swerve swerve) {

        this.limelight = limelight;
        this.swerve = swerve;
        headingError = 1.0;

        
        strafeController = new PIDController(0.01, 0, 0);
        
        addRequirements(swerve, limelight);
    }

    @Override
    public void execute() {
        LimelightHelpers.setPipelineIndex("limelight", 1);
        targetDistance = limelight.getTX();
        if(targetDistance < -1) {
            System.out.println("moving left");
            swerve.drive(new Translation2d(0, 1), 0, true, true);
        } else if (targetDistance > 1) {
            System.out.println("moving right");
            swerve.drive(new Translation2d(0, 
            -1), 0, true, true);
        } else {
            System.out.println("stopped");
            swerve.drive(new Translation2d(0, 0), 0, true, true);
        }

    }

    
    public void end(boolean interuppted) {
        
    }
}
