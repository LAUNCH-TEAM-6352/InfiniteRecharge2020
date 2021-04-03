/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;

/**
 * Moves the hood to the down position.
 */
public class MoveHoodToDownPosition extends CommandBase
{
	@SuppressWarnings(
	{ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Hood hood;

	public MoveHoodToDownPosition(Hood hood)
	{
		this.hood = hood;

		// Use addRequirements() here to declare subsystem dependencies.
		if (hood != null)
		{
			addRequirements(hood);
		}
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
		hood.setHood(HoodConstants.moveDownAutoPercentage);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted)
	{
		hood.setHood(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished()
	{
		return hood.isHoodAtDownPosition();
	}
}
