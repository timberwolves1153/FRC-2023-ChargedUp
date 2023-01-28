package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;
import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalanceWithRoll extends CommandBase{
    private Swerve swerve;
    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private BooleanSupplier robotCentricSup;
    



    public AutoBalanceWithRoll(Swerve swerve, BooleanSupplier robotCentricSup) {

        this.swerve = swerve;
        addRequirements(swerve);
        this.robotCentricSup =  robotCentricSup;


       
    
    }
    
    

    @Override
    public void execute() {
            

            if (swerve.getRoll() < swerve.getInitRoll() - Constants.rollDeadband)  {
                System.out.println("forwards Balancing With Roll");
                swerve.drive(
                    new Translation2d(-0.38, 0.0), 
                0.0, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
            } else if(swerve.getRoll() > swerve.getInitRoll() + Constants.rollDeadband) {
                System.out.println("backwards Balancing With Roll");
                swerve.drive(
                    new Translation2d(0.38, 0.0), 
                0.0, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
            } else {
                System.out.println("flat Balancing With Roll");
                swerve.drive(
                    new Translation2d(0.0, 0.0), 
                0.0, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
            }
    }
}
