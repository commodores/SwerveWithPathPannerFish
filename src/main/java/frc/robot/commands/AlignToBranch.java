package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToBranch extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private static final double kP_Rotation = 0.02;
    private static final double kP_Strafe = 0.05;
    private static final double TX_TOLERANCE = 1.0; // Alignment tolerance
    private static final double TY_TOLERANCE = 1.0; // Strafe tolerance
    private static final double BRANCH_OFFSET_METERS = 0.1647; // 6.482 inches to meters
    private static final String LIMELIGHT_NAME = "limelight-front";
    private final boolean targetLeftBranch;

    public AlignToBranch(CommandSwerveDrivetrain drivetrain, boolean targetLeftBranch) {
        this.drivetrain = drivetrain;
        this.targetLeftBranch = targetLeftBranch;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV(LIMELIGHT_NAME)) { // Ensure Limelight has a target
            final double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
            final double ty = LimelightHelpers.getTY(LIMELIGHT_NAME);

            // Apply branch offset (left = negative, right = positive)
            final double targetOffset = targetLeftBranch ? -BRANCH_OFFSET_METERS : BRANCH_OFFSET_METERS;
            final double adjustedStrafe = (ty + targetOffset) * kP_Strafe;
            final double rotationSpeed = Math.abs(tx) < TX_TOLERANCE ? 0 : -tx * kP_Rotation;

            // Apply alignment movement
            drivetrain.applyRequest(() -> 
                new SwerveRequest.RobotCentric()
                    .withVelocityX(0) // No forward movement, only aligning
                    .withVelocityY(adjustedStrafe)
                    .withRotationalRate(rotationSpeed)
            ).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(LimelightHelpers.getTX(LIMELIGHT_NAME)) < TX_TOLERANCE 
            && Math.abs(LimelightHelpers.getTY(LIMELIGHT_NAME)) < TY_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> 
            new SwerveRequest.RobotCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        ).schedule(); // Stop movement when finished
    }
}