package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToBranch extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    // PID Constants
    private static final double kP_Rotation = 0.02;
    private static final double kP_Strafe = 0.05;

    // Tolerances & Limits
    private static final double TX_TOLERANCE = 1.0; // Stop if within this tx range
    private static final double TX_DEADBAND = 0.5; // Prevents tiny corrections
    private static final double MAX_STRAFE_SPEED = 0.6; // Prevents overshoot
    private static final double MIN_SCALING_FACTOR = 0.2; // Prevents stalling
    private static final double TX_ROTATION_THRESHOLD = 2.0; // Only rotate after reaching this tx threshold
    private static final int REQUIRED_STABLE_FRAMES = 5; // Frames needed to "lock in"

    // Limelight Constants
    private static final String LIMELIGHT_NAME = "limelight-front";
    private static final int LEFT_BRANCH_PIPELINE = 0;
    private static final int RIGHT_BRANCH_PIPELINE = 1;
    
    private final boolean targetLeftBranch;
    private int stableFrames = 0; // Tracks stability before stopping

    public AlignToBranch(CommandSwerveDrivetrain drivetrain, boolean targetLeftBranch) {
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
    if (LimelightHelpers.getTV(LIMELIGHT_NAME)) { // Ensure Limelight has a valid target
        final double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        double limelightLatency = LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME);
        if (limelightLatency > 100) return; // Ignore outdated data

        System.out.println("TX: " + tx); // Debugging output

        // Strafe correction with speed limit (robot strafes toward `tx = 0`)
        final double adjustedStrafe = Math.max(-MAX_STRAFE_SPEED, 
            Math.min(MAX_STRAFE_SPEED, -tx * kP_Strafe));

        // Use an array to allow modification inside lambda
        final double[] rotationSpeed = {0};

        if (Math.abs(tx) < TX_ROTATION_THRESHOLD) {
            // Scaling factor: reduces overshoot near target
            double scalingFactor = Math.max(MIN_SCALING_FACTOR, Math.abs(tx) / 10.0);
            rotationSpeed[0] = Math.abs(tx) > TX_DEADBAND ? scalingFactor * -tx * kP_Rotation : 0;
        }

        // Apply alignment movement
        drivetrain.applyRequest(() -> 
            new SwerveRequest.RobotCentric()
                .withVelocityX(0) // No forward movement
                .withVelocityY(adjustedStrafe) // Strafe first
                .withRotationalRate(rotationSpeed[0]) // Rotate only after lateral alignment
        ).schedule();
    }
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
