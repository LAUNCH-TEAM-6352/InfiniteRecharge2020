/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/**
 * An example command that uses an example subsystem.
 */
public class RunIndexer extends CommandBase
{
	@SuppressWarnings(
	{ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Indexer indexer;
	private String key = null;
	private double percentage;

	private RunIndexer(Indexer indexer)
	{
		this.indexer = indexer;
		addRequirements(indexer);
	}

	public RunIndexer(Indexer indexer, String key)
	{
		this(indexer);
		this.key = key;
	}

	public RunIndexer(Indexer indexer, double percentage)
	{
		this(indexer);
		this.percentage = percentage;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize()
	{
		if (key != null)
		{
			percentage = SmartDashboard.getNumber(key, 0.0);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute()
	{
		indexer.set(percentage);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted)
	{
		indexer.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished()
	{
		return false;
	}
}
