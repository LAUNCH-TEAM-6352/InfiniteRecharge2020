/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

/**
 * Add your docs here.
 */
public class Indexer extends SubsystemBase
{
	private VictorSPX rearMotor = new VictorSPX(IndexerConstants.rearMotorChannel);

	public Indexer()
	{
		rearMotor.setInverted(IndexerConstants.isRearMotorInverted);
	}

	public void set(double percentage)
	{
		rearMotor.set(ControlMode.PercentOutput, percentage);
	}

	public void stop()
	{
		set(0);
	}
}
