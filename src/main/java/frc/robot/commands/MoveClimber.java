// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveClimber extends Command {
    private final Climber climber;
    private final double speed;

    public MoveClimber(Climber climber, double speed) {
        this.climber = climber;
        this.speed = speed;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        // 🚨 Prevent forward movement if locked
        if (climber.getIsLocked() && speed > 0) {
            climber.stopClimber();
            return;
        }

        climber.setClimberSpeed(speed);
        
    }

    @Override
    public void end(boolean interrupted) {
        climber.stopClimber();
    }
}