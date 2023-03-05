package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.autos.ScoreAndBalanceTest;
import frc.robot.commands.AutoBalanceWithRoll;
import frc.robot.commands.ExtendIn;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.PIDExtender;
import frc.robot.subsystems.PIDPivot;
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

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driveStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton driveA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driveB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driveX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driveY = new JoystickButton(driver, XboxController.Button.kY.value);
    
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
    private final JoystickButton atariButton12 = new JoystickButton(atari, 12);

    private final JoystickButton opOneButton = new JoystickButton(operator, 1);
    private final JoystickButton opTwoButton = new JoystickButton(operator, 2);
    private final JoystickButton opThreeButton = new JoystickButton(operator, 3);



    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
   // private final Collector collector = new Collector();
   // private final PIDExtender pidExtender = new PIDExtender();
   // private final PIDPivot pidPivot = new PIDPivot();
   // private final LEDLights ledLights = new LEDLights();


    //private final ExtendIn extendIn;
    //private final ExtendOut extendOut;
    private final AutoBalanceWithRoll autoBalanceWithRoll; 
    // private final PivotToPosition L3;
    // private final PivotToPosition L2;
    // private final PivotToPosition Hybrid;
    // private final PivotToPosition Max;
    // private final PivotToPosition ConeDSS;
    // private final PivotToPosition ConeSSS;
    // private final PivotToPosition Min;

    // private final ExtendToPosition extendToL3;
    // private final ExtendToPosition extend1inch;

    //private ScoreAndMove scoreAndMove;
   // private AutoBalanceAuto scoreAndBalance;
    private SendableChooser<Command> autoCommandChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // s_Swerve.setDefaultCommand(
        //     new TeleopSwerve(
        //         s_Swerve, 
        //         () -> -driver.getRawAxis(translationAxis), 
        //         () -> -driver.getRawAxis(strafeAxis), 
        //         () -> -driver.getRawAxis(rotationAxis) * 0.7, 
        //         () -> robotCentric.getAsBoolean()
        //     )
        // );

        // pivot.setDefaultCommand(
        //     new DefaultPivot(() -> operator.getRawAxis(1), 
        //     pivot));
        //pidPivot.setDefaultCommand(new OverridePivot(() -> operator.getRawAxis(translationAxis), pidPivot));
       
        
        autoBalanceWithRoll = new AutoBalanceWithRoll(s_Swerve, () -> robotCentric.getAsBoolean());
        // pivotToPositionTop = new PivotToPosition(0.3, pivot);
        // pivotToPositionBottom = new PivotToPosition(0.55, pivot);
        // L3 = new PivotToPosition(Constants.PivotSetpoints.L3, pivot);
        // L2 = new PivotToPosition(Constants.PivotSetpoints.L2, pivot);
        // Hybrid = new PivotToPosition(Constants.PivotSetpoints.HYBRID, pivot);
        // Max = new PivotToPosition(Constants.PivotSetpoints.MAX, pivot);
        // ConeDSS = new PivotToPosition(Constants.PivotSetpoints.CONE_DSS, pivot);
        // ConeSSS = new PivotToPosition(Constants.PivotSetpoints.CONE_SSS, pivot);
        // Min = new PivotToPosition(Constants.PivotSetpoints.MIN, pivot);
        // extendToL3 = new ExtendToPosition(extender.distanceToEncoderTicks(22.9), extender);
        // extend1inch = new ExtendToPosition(extender.distanceToEncoderTicks(1), extender);

        autoCommandChooser = new SendableChooser<Command>();
        //extendIn = new ExtendIn(extender);
        //extendOut = new ExtendOut(extender);


        //autoCommandChooser.setDefaultOption("Move and Score", scoreAndMove);
        //autoCommandChooser.addOption("Score and Balacne", scoreAndBalance);

        SmartDashboard.putData("Auto Command Chooser", autoCommandChooser);


        
        // Configure the button bindings
        configureButtonBindings();
        CameraServer.startAutomaticCapture(1);
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
        driveX.whileTrue(new InstantCommand(() -> s_Swerve.xPosition(true)));

        /* Operator Buttons */

        
        // opYButton.onTrue(Commands.runOnce(() -> {pidPivot.disable(); pidPivot.pivotUp();}));
        // opYButton.onFalse(new InstantCommand(() -> pidPivot.holdPosition()));
        
        // opAButton.onTrue(Commands.runOnce(() -> {pidPivot.disable(); pidPivot.pivotDown();}));
        // opAButton.onFalse(new InstantCommand(() -> pidPivot.holdPosition()));
       // opYButton.onTrue(Commands.runOnce(() -> pidPivot.setSetpoint(0.5), pidPivot));
        
        
        // opXButton.whileTrue(new InstantCommand(() -> pidExtender.extendOut()));
        // opXButton.onFalse(new InstantCommand(() -> pidExtender.stop()));
        
        // opBButton.whileTrue(new InstantCommand(() -> pidExtender.extendIn()));
        // opBButton.onFalse(new InstantCommand(() -> pidExtender.stop())); 

    //     opLeftBumper.whileTrue(new InstantCommand(() -> collector.collectorIntake()));
    //     opLeftBumper.onFalse(new InstantCommand(() -> collector.collectorStop()));

    //     opRightBumper.whileTrue(new InstantCommand(() -> collector.collectorOuttake()));
    //     opRightBumper.onFalse(new InstantCommand(() -> collector.collectorStop()));

    //     // atariButton1.onTrue(Hybrid.withTimeout(1.75));
    //     // atariButton2.onTrue(L2.withTimeout(1.75));
    //     // atariButton3.onTrue(L3);
    //     opXButton.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(7), pidExtender));
    //     opBButton.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(22.9), pidExtender));
    //     opStart.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(-5), pidExtender));
    //     atariButton1.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(-50), pidPivot));
    //     atariButton1.onTrue(new InstantCommand(() -> collector.slowIntake()));
    //     atariButton2.onTrue(new InstantCommand(() -> collector.slowIntake()));
    //     atariButton3.onTrue(new InstantCommand(() -> collector.slowIntake()));
    //     atariButton2.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(15), pidPivot));
    //     //atariButton2.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(7), pidExtender));
    //     atariButton3.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(19), pidPivot));
    //    // atariButton3.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(22.9), pidExtender));
    //     // atariButton3.onTrue(extendToL3);
    //    // atariButton3.onTrue(extendToL3);
    //    // atariButton4.onTrue(Max);

    //    // atariButton5.onTrue(new InstantCommand( () -> extender.incrementSetpoint()));
    //     //atariButton6.onTrue(new InstantCommand( () -> extender.decrementSetpoint()));
    //     // atariButton5.whileTrue(new InstantCommand(() -> extender.extendIn()));
    //     // atariButton5.onFalse(new InstantCommand(() -> extender.stop()));
        
    //     //atariButton6.onTrue(new InstantCommand(() -> pidExtender.setSetpoint((pidExtender.getController().getSetpoint())+1)).repeatedly());
    //     atariButton6.whileTrue(new ExtendIn(pidExtender));

    //     atariButton5.onFalse(new InstantCommand(() -> pidExtender.holdPosition()));
    //     atariButton5.onTrue(Commands.runOnce(() -> {pidExtender.disable(); pidExtender.extendOut();}));
    //     //atariButton6.onFalse(new InstantCommand(() -> pidExtender.holdPosition()));
    //     // atariButton6.whileTrue(new InstantCommand(() -> extender.extendOut()));
    //     // atariButton6.onFalse(new InstantCommand(() -> extender.stop()));

    //     atariButton7.whileTrue(new InstantCommand(() -> collector.collectorIntake()));
    //     atariButton7.onFalse(new InstantCommand(() -> collector.collectorStop()));

    //     atariButton8.whileTrue(new InstantCommand(() -> collector.collectorOuttake()));
    //     atariButton8.onFalse(new InstantCommand(() -> collector.collectorStop()));

        
    //     opYButton.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(29), pidPivot));
    //     opAButton.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(0), pidPivot));
    //     // atariButton9.onTrue(ConeDSS.withTimeout(1.75));
    //     // atariButton10.onTrue(ConeSSS.withTimeout(1.75));

    //     // atariButton12.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(-5), pidExtender));
    //     atariButton10.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(29), pidPivot));
    //     //atariButton12.onTrue(Commands.runOnce(() -> pidPivot.setSetpoint(0.5), pidPivot));
    //     //atariButton12.onTrue(extend1inch);
    //     //atariButton12.onTrue(Min.withTimeout(1.75));
    //     // opStart.onTrue(pivotToPositionTop.withTimeout(1.75));
    //     // opBack.onTrue(pivotToPositionBottom.withTimeout(1.75));
    //     //opStart.onFalse(pivotToPositionBottom());
        
    //      driveB.onTrue(new InstantCommand(() -> ledLights.setRGB(218, 165, 0)));
    //     // //opOneButton.onFalse(new InstantCommand(() -> ledLights.setRGB(0, 0, 0)));

    //      driveY.onTrue(new InstantCommand(() -> ledLights.setRGB(128, 0, 128)));
    //     // //opTwoButton.onFalse(new InstantCommand(() -> ledLights.setRGB(0, 0, 0)));
    //     driveX.onTrue(new InstantCommand(() -> ledLights.setRGB(0, 255, 0)));

        // opBack.onTrue(Commands.runOnce(() -> {
        //     pidExtender.disable();
        //     pidExtender.resetEncoder(); 
        //     pidExtender.holdPosition();
        // }, pidExtender));
        
    }

    public void resetToAbsolute() {
        s_Swerve.resetModulesToAbsolute();
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
       //return autoCommandChooser.getSelected();
      // return new ScoreAndMove(s_Swerve, pivot, collector);
      return null;//new ScoreAndBalanceTest(s_Swerve);
    }

}
