/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Add your docs here.
 */
public class Intake extends SubsystemBase
{
	private DoubleSolenoid solenoid = new DoubleSolenoid(IntakeConstants.outSolenoidChannel, IntakeConstants.inSolenoidChannel);

	private TalonSRX motor = new TalonSRX(IntakeConstants.motorChannel);

	public Intake()
	{
		motor.setInverted(IntakeConstants.isIntakeMotorInverted);
	}

	public void set(double percentage)
	{
		motor.set(ControlMode.PercentOutput, percentage);
	}

	public void stop()
	{
		set(0);
	}

	public void moveIntakeIn()
	{
		solenoid.set(DoubleSolenoid.Value.kReverse);
	}

	public void moveIntakeOut()
	{
		solenoid.set(DoubleSolenoid.Value.kForward);
	}
}
