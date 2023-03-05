package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SnapToAngle extends CommandBase{

    double angle;
    Swerve swerve;
    PIDController thetaController;
 
    public SnapToAngle(double angle, Swerve swerve) {

        angle = angle;
        swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        thetaController = new PIDController(0.05, 0, 0);

        thetaController.enableContinuousInput(-180, 180);
        thetaController.setSetpoint(angle);
    }

    @Override
    public void execute() {
        double rotationVal = thetaController.calculate(-(MathUtil.inputModulus(swerve.getYaw().getDegrees(), -180, 180)), thetaController.getSetpoint());
        swerve.drive(new Translation2d(0, 0), rotationVal * Constants.Swerve.maxAngularVelocity, true, true);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
