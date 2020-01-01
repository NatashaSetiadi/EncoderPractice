package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorPositionCommand extends Command {
	
	public ElevatorPositionCommand() {
		requires(Robot.encoderSubsystem); 
	}

	protected void initialize() {

	}

	public void execute() {
        Robot.encoderSubsystem.motorSet();

	}

	protected boolean isFinished() {
		return false; // We don't want the command to stop, we want it to be interrupted.
	}

	protected void end() {
		Robot.encoderSubsystem.stop();
	}

}