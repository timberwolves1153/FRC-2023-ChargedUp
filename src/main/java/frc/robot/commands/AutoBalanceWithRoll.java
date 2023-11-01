package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;
import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalanceWithRoll extends CommandBase{
    private Swerve swerve;
    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private BooleanSupplier robotCentricSup;
    private boolean haveTipped = false;
    public static final double rollDeadband = 8;

    public AutoBalanceWithRoll(Swerve swerve, BooleanSupplier robotCentricSup) {
        this.swerve = swerve;
        addRequirements(swerve);
        this.robotCentricSup =  robotCentricSup;
    }

    @Override
    public void execute() {
            
            if (swerve.getRoll() < swerve.getInitRoll() - rollDeadband)  {
                //System.out.println("forwards Balancing With Roll");
                if (!haveTipped) {
                    swerve.drive(
                        new Translation2d(-0.615
                        , 0.0), 
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
            } else if(swerve.getRoll() > swerve.getInitRoll() + rollDeadband) {
                //System.out.println("backwards Balancing With Roll");
                haveTipped = true;
                swerve.drive(
                    new Translation2d(0.4, 0.0), 
                0.0, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
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
        swerve.xPosition(true);
    }
}

