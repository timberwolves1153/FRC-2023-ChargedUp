package frc.robot;

import org.opencv.video.KalmanFilter;

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
import frc.robot.commands.DefaultPivot;
import frc.robot.commands.PivotToPosition;
// import frc.robot.commands.ExtendIn;
// import frc.robot.commands.ExtendOut;
// import frc.robot.commands.ExtendToDistance;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.LEDLights;
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
    private final Joystick atari = new Joystick(2);

    private SwerveDriveKinematics swerveKinematics;


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driveStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton driveA = new JoystickButton(driver, XboxController.Button.kA.value);
    
    /* Operator Buttons */
    private final JoystickButton opYButton = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton opAButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton opXButton = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton opBButton = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton opLeftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton opRightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton opStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton opBack = new JoystickButton(operator, XboxController.Button.kBack.value);

    private final JoystickButton atariButton1 = new JoystickButton(atari, 1);
    private final JoystickButton atariButton2 = new JoystickButton(atari, 2);
    private final JoystickButton atariButton3 = new JoystickButton(atari, 3);
    private final JoystickButton atariButton4 = new JoystickButton(atari, 4);
    private final JoystickButton atariButton5 = new JoystickButton(atari, 5);
    private final JoystickButton atariButton6 = new JoystickButton(atari, 6);
    private final JoystickButton atariButton7 = new JoystickButton(atari, 7);
    private final JoystickButton atariButton8 = new JoystickButton(atari, 8);
    private final JoystickButton atariButton9 = new JoystickButton(atari, 9);
    private final JoystickButton atariButton10 = new JoystickButton(atari, 10);

    // private final JoystickButton opOneButton = new JoystickButton(operator, 1);
    // private final JoystickButton opTwoButton = new JoystickButton(operator, 2);



    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot pivot = new Pivot();
    private final Extender extender = new Extender();
    private final Collector collector = new Collector();

    //private final ExtendIn extendIn;
    //private final ExtendOut extendOut;
    private final AutoBalanceWithRoll autoBalanceWithRoll; 
    private final PivotToPosition pivotToPositionTop;
    private final PivotToPosition pivotToPositionBottom;
    private final PivotToPosition L3;
    private final PivotToPosition L2;
    private final PivotToPosition Hybrid;
    private final PivotToPosition ConeDSS;
    private final PivotToPosition ConeSSS;

    private LEDLights ledLights;

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

        pivot.setDefaultCommand(new DefaultPivot(() -> operator.getRawAxis(1), pivot));
        
        autoBalanceWithRoll = new AutoBalanceWithRoll(s_Swerve, () -> robotCentric.getAsBoolean());
        pivotToPositionTop = new PivotToPosition(0.3, pivot);
        pivotToPositionBottom = new PivotToPosition(0.55, pivot);
        L3 = new PivotToPosition(0.42, pivot);
        L2 = new PivotToPosition(0.45, pivot);
        Hybrid = new PivotToPosition(0.58, pivot);
        ConeDSS = new PivotToPosition(0.38, pivot);
        ConeSSS = new PivotToPosition(0.46, pivot);
        //extendIn = new ExtendIn(extender);
        //extendOut = new ExtendOut(extender);

        ledLights = new LEDLights();


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
        driveA.whileTrue(autoBalanceWithRoll);
        driveA.onFalse(new InstantCommand(() -> s_Swerve.stop())); 

        /* Operator Buttons */

        opYButton.whileTrue(new InstantCommand(() -> pivot.pivotUp()));
        opYButton.onFalse(new InstantCommand(() -> pivot.pivotStop()));

        opAButton.whileTrue(new InstantCommand(() -> pivot.pivotDown()));
        opAButton.onFalse(new InstantCommand(() -> pivot.pivotStop()));
        
        opXButton.whileTrue(new InstantCommand(() -> extender.extendIn()));
        opXButton.onFalse(new InstantCommand(() -> extender.stop()));
        
        opBButton.whileTrue(new InstantCommand(() -> extender.extendOut()));
        opBButton.onFalse(new InstantCommand(() -> extender.stop())); 

        opLeftBumper.whileTrue(new InstantCommand(() -> collector.collectorIntake()));
        opLeftBumper.onFalse(new InstantCommand(() -> collector.collectorStop()));

        opRightBumper.whileTrue(new InstantCommand(() -> collector.collectorOuttake()));
        opRightBumper.onFalse(new InstantCommand(() -> collector.collectorStop()));

        atariButton1.onTrue(Hybrid.withTimeout(1.75));
        atariButton2.onTrue(L2.withTimeout(1.75));
        atariButton3.onTrue(L3.withTimeout(1.75));
        atariButton5.whileTrue(new InstantCommand(() -> extender.extendOut()));
        atariButton5.onFalse(new InstantCommand(() -> extender.stop()));
        atariButton6.whileTrue(new InstantCommand(() -> extender.extendIn()));
        atariButton6.onFalse(new InstantCommand(() -> extender.stop()));
        atariButton7.whileTrue(new InstantCommand(() -> collector.collectorIntake()));
        atariButton7.onFalse(new InstantCommand(() -> collector.collectorStop()));
        atariButton8.whileTrue(new InstantCommand(() -> collector.collectorOuttake()));
        atariButton8.onFalse(new InstantCommand(() -> collector.collectorStop()));
        atariButton9.onTrue(ConeDSS.withTimeout(1.75));
        atariButton10.onTrue(ConeSSS.withTimeout(1.75));
        // opStart.onTrue(pivotToPositionTop.withTimeout(1.75));
        // opBack.onTrue(pivotToPositionBottom.withTimeout(1.75));
        //opStart.onFalse(pivotToPositionBottom());
        
        // opOneButton.whileTrue(new InstantCommand(() -> ledLights.setRGB(255, 255, 0)));
        // opOneButton.onFalse(new InstantCommand(() -> ledLights.setRGB(0, 0, 0)));

        // opTwoButton.whileTrue(new InstantCommand(() -> ledLights.setRGB(128, 0, 128)));
        // opTwoButton.onFalse(new InstantCommand(() -> ledLights.setRGB(0, 0, 0)));


        
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
