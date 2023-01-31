package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.AutoBalanceAuto;
import frc.robot.autos.BalanceAuto;
import frc.robot.autos.LoopAuto;
import frc.robot.autos.TestAuto;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.AutoBalanceWithRoll;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    private SwerveDriveKinematics swerveKinematics;


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton opStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton yButton = new JoystickButton(driver, XboxController.Button.kY.value);
    
    /* Operator Buttons */
    private final JoystickButton opYButton = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton opAButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton xButton = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton bButton = new JoystickButton(operator, XboxController.Button.kB.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot pivot = new Pivot();
    private final Extender extender = new Extender();

    private final AutoBalanceWithRoll autoBalanceWithRoll; 

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        
        autoBalanceWithRoll = new AutoBalanceWithRoll(s_Swerve, () -> robotCentric.getAsBoolean());


        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        yButton.whileTrue(autoBalanceWithRoll);
        yButton.onFalse(new InstantCommand(() -> s_Swerve.stop())); 

        /* Operator Buttons */

        opYButton.whileTrue(new InstantCommand(() -> pivot.pivotUp()));
        opYButton.onFalse(new InstantCommand(() -> pivot.pivotStop()));

        opAButton.whileTrue(new InstantCommand(() -> pivot.pivotDown()));
        opAButton.onFalse(new InstantCommand(() -> pivot.pivotStop()));
        
        xButton.whileTrue(new InstantCommand(() -> extender.extendIn()));
        xButton.onFalse(new InstantCommand(() -> extender.stop()));
        
        bButton.whileTrue(new InstantCommand(() -> extender.extendOut()));
        bButton.onFalse(new InstantCommand(() -> extender.stop()));

        opStart.whileTrue(autoBalanceWithRoll);
        opStart.onFalse(new InstantCommand(() -> s_Swerve.stop())); 
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new TestAuto(testTrajectory, , s_Swerve);
        //return null;
        return new AutoBalanceAuto(s_Swerve);
    }
}
