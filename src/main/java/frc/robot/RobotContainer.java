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
import frc.robot.commands.AutoHopper;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoReverse;
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

    public final Intake m_Intake = new Intake();

    public final Climber m_climber = new Climber();

    public final Armivator m_Armivator = new Armivator();

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

        

        //Intake and Hopper buttons
        //intake lvl 1-3
       driver.b().whileTrue(new InstantCommand(() -> m_Intake.runIntakeSpeed(1)));
       driver.b().whileFalse(new InstantCommand(() -> m_Intake.runIntakeSpeed(0)));

       //intake lvl 4
       operator.y().whileTrue(new ShootLvlFour(m_Intake));
       //intake reverse
       operator.x().whileTrue(new AutoReverse(m_Intake));

        //intake
        driver.x().onTrue(
            (m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kFeederStation))
        .andThen(new AutoHopper(m_Intake))
        .andThen(new AutoIntake(m_Intake))
        .andThen(m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kLevel1)).withTimeout(5)
            
        );

        //Arm
        operator.povLeft().onTrue(m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kFeederStation));
        operator.povDown().onTrue(m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kLevel1));
        operator.povRight().onTrue(m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kLevel2));
        operator.a().onTrue(m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kLevel3));
        operator.povUp().onTrue(m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kLevel4));
               
    

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
