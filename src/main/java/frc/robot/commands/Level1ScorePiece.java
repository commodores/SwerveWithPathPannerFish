package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Level1Intake;

public class Level1ScorePiece extends Command {
    private final Level1Intake intake;
    private final double scoreSpeed; // Speed to eject the game piece
    private final double minCurrentThreshold; // Threshold to detect when the piece is gone
    private final double maxTime; // Max time to run the command (safety timeout)
    private final Timer timer = new Timer();

    public Level1ScorePiece(Level1Intake intake, double scoreSpeed, double minCurrentThreshold, double maxTime) {
        this.intake = intake;
        this.scoreSpeed = scoreSpeed;
        this.minCurrentThreshold = minCurrentThreshold;
        this.maxTime = maxTime;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeSpeed(scoreSpeed);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        // Stop if current drops below threshold (piece is ejected) OR timeout reached
        //return intake.getMotorCurrent() < minCurrentThreshold || timer.hasElapsed(maxTime);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        timer.stop();
    }
}
