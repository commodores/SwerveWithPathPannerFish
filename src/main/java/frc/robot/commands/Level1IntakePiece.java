package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Level1Intake;

public class Level1IntakePiece extends Command {
    private final Level1Intake intake;
    private final double speed;
    private final double currentThreshold; // Current threshold for detecting game piece

    public Level1IntakePiece(Level1Intake intake, double speed, double currentThreshold) {
        this.intake = intake;
        this.speed = speed;
        this.currentThreshold = currentThreshold;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeSpeed(speed);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        // Stop when the motor current exceeds the threshold (indicating a game piece is held)
        //return intake.getMotorCurrent() > currentThreshold;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
