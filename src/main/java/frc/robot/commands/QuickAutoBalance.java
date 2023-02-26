package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class QuickAutoBalance extends CommandBase{
    private Swerve swerve;
    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private BooleanSupplier robotCentricSup;
    private boolean haveTipped = false;
    
    public QuickAutoBalance(Swerve swerve, BooleanSupplier robotCentricSup) {
        this.swerve = swerve;
        addRequirements(swerve);
        this.robotCentricSup =  robotCentricSup;
    }

    @Override
    public void execute() {
            
            if (swerve.getRoll() < swerve.getInitRoll() - Constants.rollDeadband)  {
                //System.out.println("forwards Balancing With Roll");
                if (!haveTipped) {
                    swerve.drive(
                        new Translation2d(-0.5, 0.0), 
                    0.0, 
                    !robotCentricSup.getAsBoolean(), 
                    true
                    );
                } else {
                    swerve.drive(
                        new Translation2d(-0.4, 0.0), 
                    0.0, 
                    !robotCentricSup.getAsBoolean(), 
                    true
                    );
                }
            } else {
                //System.out.println("flat Balancing With Roll");
                swerve.drive(
                    new Translation2d(0.0, 0.0), 
                0.0, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
            }
    }

    public void end(boolean interrupted) {
        haveTipped = false;
    }
}

