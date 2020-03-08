/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Operates the climber actuators using the game controller.
 */
public class RunClimberWithGameController extends CommandBase
{
	@SuppressWarnings(
	{ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Climber climber;
	private final XboxController controller;

	public RunClimberWithGameController(Climber climber, XboxController controller)
	{
		this.climber = climber;
		this.controller = controller;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(climber);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize()
	{
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute()
	{
		climber.setSpeeds(-controller.getTriggerAxis(Hand.kLeft), -controller.getTriggerAxis(Hand.kRight));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted)
	{
		climber.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished()
	{
		return false;
	}
}
