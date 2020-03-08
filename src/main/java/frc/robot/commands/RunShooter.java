/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * Runs the shooter motors at a particular velocity.
 */
public class RunShooter extends CommandBase
{
	@SuppressWarnings(
	{ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Shooter shooter;
	private String key = null;
	private double velocity;

	private RunShooter(Shooter shooter)
	{
		this.shooter = shooter;
		addRequirements(shooter);
	}

	public RunShooter(Shooter shooter, String key)
	{
		this(shooter);
		this.key = key;
	}

	public RunShooter(Shooter shooter, double velocity)
	{
		this(shooter);
		this.velocity = velocity;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize()
	{
		if (key != null)
		{
			velocity = SmartDashboard.getNumber(key, 0.0);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute()
	{
		shooter.setVelocity(velocity);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted)
	{
		shooter.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished()
	{
		return false;
	}
}
