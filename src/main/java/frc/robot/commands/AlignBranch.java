package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignBranch extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    // PID Constants
    private static final double kP_Strafe = 0.1;

    // Tolerances & Limits
    private static final double TX_TOLERANCE = 2.0; // Stop if within this tx range
    private static final double TX_DEADBAND = 0.5; // Prevents tiny corrections
    private static final double MAX_STRAFE_SPEED = 0.6; // Prevents overshoot
    private static final double MIN_STRAFE_SPEED = 0.1; // Prevents stalling
    private static final double MAX_ACCEPTABLE_LATENCY = 100; // Ensures fresh Limelight data
    private static final int REQUIRED_STABLE_FRAMES = 5; // Frames needed to "lock in"

    // Limelight Constants
    private static final String LIMELIGHT_NAME = "limelight-front";
    private static final int LEFT_BRANCH_PIPELINE = 0;
    private static final int RIGHT_BRANCH_PIPELINE = 1;
    
    private final boolean targetLeftBranch;
    private int stableFrames = 0; // Tracks stability before stopping

    public AlignBranch(CommandSwerveDrivetrain drivetrain, boolean targetLeftBranch) {
        this.drivetrain = drivetrain;
        this.targetLeftBranch = targetLeftBranch;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Set Limelight to the correct pipeline before alignment starts
        int targetPipeline = targetLeftBranch ? LEFT_BRANCH_PIPELINE : RIGHT_BRANCH_PIPELINE;
        LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, targetPipeline);
        System.out.println("Switched to Limelight pipeline: " + targetPipeline);
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(LIMELIGHT_NAME)) {
            // No valid target detected, stop movement
            drivetrain.applyRequest(() -> 
                new SwerveRequest.RobotCentric()
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            ).schedule();
            return;
        }

        final double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        final double limelightLatency = LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME);

        if (limelightLatency > MAX_ACCEPTABLE_LATENCY) return; // Ignore outdated data

        System.out.println("TX: " + tx); // Debugging output

        // Apply deadband to prevent unnecessary small adjustments
        double computedStrafe = 0;
        if (Math.abs(tx) > TX_DEADBAND) {
            computedStrafe = Math.max(-MAX_STRAFE_SPEED, Math.min(MAX_STRAFE_SPEED, -tx * kP_Strafe));

            // Ensure minimum movement to prevent stalling
            if (Math.abs(computedStrafe) < MIN_STRAFE_SPEED) {
                computedStrafe = Math.signum(computedStrafe) * MIN_STRAFE_SPEED;
            }
        }

        final double adjustedStrafe = computedStrafe; // Ensure it's final for the lambda

        // Apply movement correction
        drivetrain.applyRequest(() -> 
            new SwerveRequest.RobotCentric()
                .withVelocityX(0)
                .withVelocityY(adjustedStrafe) // Now correctly used in lambda
                .withRotationalRate(0)
        ).schedule();
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(LimelightHelpers.getTX(LIMELIGHT_NAME)) <= TX_TOLERANCE) {
            stableFrames++;
        } else {
            stableFrames = 0;
        }
        return stableFrames >= REQUIRED_STABLE_FRAMES;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop movement when finished
        drivetrain.applyRequest(() -> 
            new SwerveRequest.RobotCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        ).schedule();
    }
}
