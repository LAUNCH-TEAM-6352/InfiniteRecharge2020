/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;

/**
 * Moves the hood to the up position.
 */
public class MoveHoodToPosition extends CommandBase
{
	@SuppressWarnings(
	{ "PMD.UnusedPrivateField", "PMD.SingularField" })

	private Hood hood;
	private String key = null;
	private double position = 0;

	private MoveHoodToPosition(Hood hood)
	{
		this.hood = hood;
		addRequirements(hood);

		// This is a backup in case we don't zero in on the target position:
		//withTimeout(HoodConstants.moveToPositionTimeoutInSeconds);
	}

	public MoveHoodToPosition(Hood hood, String key)
	{
		this(hood);
		this.key = key;
	}

	public MoveHoodToPosition(Hood hood, double position)
	{
		this(hood);
		this.position = position;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize()
	{
		if (key != null)
		{
			position = SmartDashboard.getNumber(key, 0);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute()
	{
		hood.setHoodPosition(position);
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
		return Math.abs(hood.getCurrentHoodPosition() - position) <= HoodConstants.moveToPositionTolerance;
	}
}
