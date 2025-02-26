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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignToBranch;
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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final static Intake m_Intake = new Intake();

    public final static Climber m_climber = new Climber();

    public final static Arm m_Arm = new Arm();
    public final static Elevator m_Elevator = new Elevator();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("AutoHopper", new AutoHopper(m_Intake));
        NamedCommands.registerCommand("AutoIntake", new AutoIntake(m_Intake));
        NamedCommands.registerCommand("AutoReverse", new AutoReverse(m_Intake));
        NamedCommands.registerCommand("AutoScore", new AutoScore(m_Intake));
        //NamedCommands.registerCommand("LevelOnePose", new LevelOnePose(m_Armivator).withTimeout(.1));
        //NamedCommands.registerCommand("FeederPose", new FeederPose(m_Armivator));

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
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // reset the field-centric heading on left bumper press
        driver.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Align to LEFT branch
        driver.povLeft().onTrue(new AlignToBranch(drivetrain, true));

        // Align to RIGHT branch
        driver.povRight().onTrue(new AlignToBranch(drivetrain, false));

        // Drive forward straight
        driver.povUp().whileTrue(drivetrain.applyRequest(() ->  forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        // Drive reverse straight
        driver.povDown().whileTrue(drivetrain.applyRequest(() ->  forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        //Climber
        driver.rightBumper().onTrue(new InstantCommand(() -> m_climber.climberFoward()));
        driver.rightBumper().onFalse(new InstantCommand(() -> m_climber.stopClimber()));

        driver.leftBumper().onTrue(new InstantCommand(() -> m_climber.climberBack()));
        driver.leftBumper().onFalse(new InstantCommand(() -> m_climber.stopClimber()));
        
        // Climber Lock
        driver.a().onTrue(new InstantCommand(() -> m_climber.unLockClimber()));//climber can go foward and backwards
        driver.y().onTrue(new InstantCommand(() -> m_climber.lockClimber()));//climber can only go backward

        
        //intake
        driver.x().onTrue(
            new AutoHopper(m_Intake)
            .andThen(new AutoIntake(m_Intake))
            .andThen(new AutoReverse(m_Intake))
            .andThen(new LevelOne(m_Arm, m_Elevator).withTimeout(.001)).withTimeout(5)            
        );

        //Score
        driver.b().onTrue(
            new AutoScore(m_Intake)
        );

        //Manual Intake and Hopper

        operator.leftBumper().onTrue(new InstantCommand(() -> m_Intake.runBothManual(1.0)));
        operator.leftBumper().onFalse(new InstantCommand(() -> m_Intake.runBothManual(0)));
        operator.rightBumper().onTrue(new InstantCommand(() -> m_Intake.runBothManual(-1.0)));
        operator.rightBumper().onFalse(new InstantCommand(() -> m_Intake.runBothManual(0)));
        

        // Arm and Elevator Position Commands
        operator.povLeft().onTrue(
            new InstantCommand(() -> {
                if (m_Elevator.getElevatorSetpoint()<2) {
                    new FeederStation(m_Arm, m_Elevator).schedule();
                }
            })
        );
        operator.povDown().onTrue(new LevelOne(m_Arm, m_Elevator));
        operator.povRight().onTrue(new LevelTwo(m_Arm, m_Elevator));
        operator.a().onTrue(new LevelThree(m_Arm, m_Elevator));
        operator.povUp().onTrue(new LevelFour(m_Arm, m_Elevator));


        //Algae

        operator.rightTrigger().onTrue(new LowAlgae(m_Arm, m_Elevator));
        operator.leftTrigger().onTrue(new HighAlgae(m_Arm, m_Elevator));               
    

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

}
