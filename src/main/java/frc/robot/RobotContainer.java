// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignToCoralStation;
import frc.robot.commands.AlignToCenterReef;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.AutoHopper;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoReverse;
import frc.robot.commands.AutoScore;
import frc.robot.commands.FeederStation;
import frc.robot.commands.HighAlgae;
import frc.robot.commands.LevelFour;
import frc.robot.commands.LevelOne;
import frc.robot.commands.LevelThree;
import frc.robot.commands.LevelTwo;
import frc.robot.commands.LowAlgae;
import frc.robot.commands.MoveClimber;
import frc.robot.commands.PushBot;
import frc.robot.commands.RemoveHighAlgae;
import frc.robot.commands.Level1FloorPosition;
import frc.robot.commands.Level1ScorePosition;
import frc.robot.commands.Level1ScorePiece;
import frc.robot.commands.Level1StowPosition;
import frc.robot.commands.RemoveLowAlgae;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Level1Arm;
import frc.robot.subsystems.Level1Intake;


public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final AlignToCoralStation alignToCoralStation = new AlignToCoralStation(drivetrain);
    private final AlignToCenterReef alignToCenterReef = new AlignToCenterReef(drivetrain);

    private final AlignToReefTagRelative alignLeft = new AlignToReefTagRelative(false, drivetrain);
    private final AlignToReefTagRelative alignRight = new AlignToReefTagRelative(true, drivetrain);    

    public final static Intake m_Intake = new Intake();

    public final static Climber m_climber = new Climber();

    public final static Arm m_Arm = new Arm();
    
    public final static Elevator m_Elevator = new Elevator();

    public final static Level1Arm m_Level1 = new Level1Arm();

    public final static Level1Intake m_Level1Intake = new Level1Intake();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("AutoHopper", new AutoHopper(m_Intake));
        NamedCommands.registerCommand("AutoIntake", new AutoIntake(m_Intake));
        NamedCommands.registerCommand("AutoReverse", new AutoReverse(m_Intake));
        NamedCommands.registerCommand("AutoScore", new AutoScore(m_Intake).withTimeout(2.5));
        NamedCommands.registerCommand("RemoveLowAlgae", new RemoveLowAlgae(m_Intake).withTimeout(2));
        NamedCommands.registerCommand("RemoveHighAlgae", new RemoveHighAlgae(m_Intake).withTimeout(2));



        NamedCommands.registerCommand("Feeder", new FeederStation(m_Arm, m_Elevator));
        NamedCommands.registerCommand("LevelOne", new LevelOne(m_Arm, m_Elevator));
        NamedCommands.registerCommand("LevelTwo", new LevelTwo(m_Arm, m_Elevator));
        NamedCommands.registerCommand("LevelThree", new LevelThree(m_Arm, m_Elevator));
        NamedCommands.registerCommand("LevelFour", new LevelFour(m_Arm, m_Elevator));
        NamedCommands.registerCommand("AlgaeLow", new LowAlgae(m_Arm, m_Elevator).withTimeout(.5));
        NamedCommands.registerCommand("AlgaeHigh", new HighAlgae(m_Arm, m_Elevator).withTimeout(.5));

        NamedCommands.registerCommand("LvlOneScorePos", new Level1ScorePosition(m_Level1));
        NamedCommands.registerCommand("LvlOneScorePiece", new Level1ScorePiece(m_Level1Intake,-0.8).withTimeout(.5));


        NamedCommands.registerCommand("AutoAlignToBranch_Left", new AlignToReefTagRelative(false, drivetrain));
        NamedCommands.registerCommand("AutoAlignToBranch_Right", new AlignToReefTagRelative(true, drivetrain));
        NamedCommands.registerCommand("AutoAlignToCoralStation", new AlignToCoralStation(drivetrain));
        NamedCommands.registerCommand("AlignToCenterReef", new AlignToCenterReef(drivetrain));
        NamedCommands.registerCommand("PushBot", new PushBot(drivetrain));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);      

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * (MaxSpeed*.8)) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * (MaxSpeed*.8)) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * (MaxAngularRate*.8)) // Drive counterclockwise with negative X (left)
            )
        );

        m_climber.setDefaultCommand(
            new MoveClimber(m_climber, 0.0) // Set to 0 so it stops moving when no input
                );

        
        // reset the field-centric heading on left bumper press
        driver.back().onTrue(drivetrain.runOnce(()-> drivetrain.seedFieldCentric()));

        // Align to LEFT branch
        driver.leftStick().onTrue(alignLeft);

        // Align to RIGHT branch
        driver.rightStick().onTrue(alignRight);

        //Align to Coral Station
        driver.leftBumper().onTrue(alignToCoralStation);

        //Align to take off Algae
        driver.rightBumper().onTrue(alignToCenterReef);
        
        // Drive forward straight
        driver.povUp().whileTrue(drivetrain.applyRequest(() ->  forwardStraight.withVelocityX(0.5).withVelocityY(0)));

        // Drive reverse straight
        driver.povDown().whileTrue(drivetrain.applyRequest(() ->  forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
        
        // Drive left robot relative
        driver.povLeft().whileTrue(drivetrain.applyRequest(() ->  forwardStraight.withVelocityX(0).withVelocityY(.5)));
        
        // Drive right robot relative
        driver.povRight().whileTrue(drivetrain.applyRequest(() ->  forwardStraight.withVelocityX(0).withVelocityY(-.5)));

        //Climber
       // driver.rightBumper().whileTrue(new MoveClimber(m_climber, .5));//Get ready to CLimb!!!
        //driver.leftBumper().whileTrue(new MoveClimber(m_climber, -1));//CLIMB!!!!!

        // Climber
        driver.rightTrigger().whileTrue(new MoveClimber(m_climber, 0.5)); // Get ready to Climb!!!
        driver.leftTrigger().whileTrue(new MoveClimber(m_climber, -1));  // CLIMB!!!!!

        
        //Climber Lock
        driver.a().onTrue(new InstantCommand(() -> m_climber.unLockClimber()));//climber can go foward and backwards
        driver.y().onTrue(new InstantCommand(() -> m_climber.lockClimber()));//climber can only go backward        

        //Lvl1 Score driver
        driver.x().whileTrue(new InstantCommand(() -> m_Level1Intake.setIntakeSpeed(-1)));
        driver.x().onFalse(new InstantCommand(() -> m_Level1Intake.stop()));

        //intake
    /*    operator.povLeft().onTrue(
            new AutoHopper(m_Intake)
                .andThen(new AutoIntake(m_Intake))
                .andThen(new InstantCommand(() -> alignToCoralStation.cancel()))
                .andThen(new AutoReverse(m_Intake))
                .andThen(new LevelOne(m_Arm, m_Elevator).withTimeout(.001)).withTimeout(10)); 
                */

        //Score
        driver.b().onTrue(
            new AutoScore(m_Intake)
                .alongWith(new InstantCommand(() -> {
                    alignLeft.cancel();
                    alignRight.cancel();
                    alignToCoralStation.cancel();
                }))
        );

        //driver.b().onTrue(new InstantCommand(() -> {
        //    alignLeft.cancel();
        //    alignRight.cancel();
        //    alignToCoralStation.cancel();
        //    new AutoScore(m_Intake).schedule();  // Explicitly schedule AutoScore
        //}));

        //Level1
        operator.back().whileTrue(new InstantCommand(() -> m_Level1Intake.setIntakeSpeed(.8)));
        operator.back().onFalse(new InstantCommand(() -> m_Level1Intake.stop())
        .andThen(new Level1ScorePosition(m_Level1)));
        
        operator.start().onTrue(new Level1ScorePosition(m_Level1));
        operator.start().whileTrue(new InstantCommand(() -> m_Level1Intake.setIntakeSpeed(-1)));
        operator.start().onFalse(new InstantCommand(() -> m_Level1Intake.stop()));

        //Manual Intake and Hopper
        operator.leftBumper().whileTrue(new InstantCommand(() -> m_Intake.runBothManual(1.0)))
                            .onFalse(new InstantCommand(() -> m_Intake.runBothManual(0)));
        operator.rightBumper().whileTrue(new InstantCommand(() -> m_Intake.runBothManual(-1.0)))
                             .onFalse(new InstantCommand(() -> m_Intake.runBothManual(0)));

        // Arm and Elevator Position Commands
        operator.povLeft().onTrue(
            new FeederStation(m_Arm,m_Elevator).withTimeout(.001)
                .andThen(new AutoHopper(m_Intake))
                .andThen(new AutoIntake(m_Intake))
                .andThen(new AutoReverse(m_Intake))
                .andThen(new LevelOne(m_Arm, m_Elevator).withTimeout(.001)).withTimeout(15)
        );

        operator.povDown().onTrue(new LevelOne(m_Arm, m_Elevator));
        operator.povRight().onTrue(new LevelTwo(m_Arm, m_Elevator));
        operator.a().onTrue(new LevelThree(m_Arm, m_Elevator));
        operator.povUp().onTrue(new LevelFour(m_Arm, m_Elevator));

        operator.b().onTrue(new Level1FloorPosition(m_Level1));
        operator.x().onTrue(new Level1StowPosition(m_Level1));
        operator.y().onTrue(new Level1ScorePosition(m_Level1));

        //Algae
        operator.leftTrigger().onTrue(new LowAlgae(m_Arm, m_Elevator));
        operator.rightTrigger().onTrue(new HighAlgae(m_Arm, m_Elevator));
    

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        //return autoChooser.getSelected();
        return new SequentialCommandGroup(
        new InstantCommand(() -> drivetrain.resetPose(drivetrain.getState().Pose)), // Reset pose before running auto
        autoChooser.getSelected() // Run the selected auto path
    );
    }

}
