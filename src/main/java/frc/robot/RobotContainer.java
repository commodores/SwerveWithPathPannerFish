// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoHopper;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoReverse;
import frc.robot.commands.AutoScore;
import frc.robot.commands.ShootLvlFour;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Armivator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final static Intake m_Intake = new Intake();

    public final static Climber m_climber = new Climber();

    public final static Armivator m_Armivator = new Armivator();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Climber
        driver.rightBumper().onTrue(new InstantCommand(() -> m_climber.climberFoward()));
        driver.rightBumper().onFalse(new InstantCommand(() -> m_climber.stopClimber()));

        driver.leftBumper().onTrue(new InstantCommand(() -> m_climber.climberBack()));
        driver.leftBumper().onFalse(new InstantCommand(() -> m_climber.stopClimber()));
        
        // Climber Lock
        operator.start().onTrue(new InstantCommand(() -> m_climber.unLockClimber()));//climber can go foward and backwards
        operator.back().onTrue(new InstantCommand(() -> m_climber.lockClimber()));//climber can only go backward

        
        //intake
        driver.x().onTrue(
            (m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kFeederStation))
            .andThen(new AutoHopper(m_Intake))
            .andThen(new AutoIntake(m_Intake))
            .andThen(new AutoReverse(m_Intake))
            .andThen(m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kLevel1)).withTimeout(5)            
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
        

        //Arm
        operator.povLeft().onTrue(m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kFeederStation));
        operator.povDown().onTrue(m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kLevel1));
        operator.povRight().onTrue(m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kLevel2));
        operator.a().onTrue(m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kLevel3));
        operator.povUp().onTrue(m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kLevel4));

        //ArmSysID
        //operator.povRight().whileTrue(m_Armivator.armSysIdDynamic(Direction.kForward));
        //operator.povLeft().whileTrue(m_Armivator.armSysIdDynamic(Direction.kReverse));
        //operator.povUp().whileTrue(m_Armivator.armSysIdQuasistatic(Direction.kForward));
        //operator.povDown().whileTrue(m_Armivator.armSysIdQuasistatic(Direction.kReverse));
               
    

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
