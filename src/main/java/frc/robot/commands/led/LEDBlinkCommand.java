package frc.robot.commands.led;

import static edu.wpi.first.wpilibj.util.Color.kBlack;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/** Command to blink the LEDs at an interval. */
public class LEDBlinkCommand extends Command {

	private final LEDSubsystem ledSubsystem;
	private final Color color;
	private final double interval;

	private final Timer timer = new Timer();

	private boolean ledsOn = true;

	/**
	 * Constructor
	 *
	 * @param ledSubsystem led subsystem
	 * @param color color to make the LEDs when they're on
	 * @param interval interval to blink the LEDs
	 */
	public LEDBlinkCommand(LEDSubsystem ledSubsystem, Color color, double interval) {
		this.ledSubsystem = ledSubsystem;
		this.color = color;
		this.interval = interval;

		addRequirements(ledSubsystem);
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
		ledSubsystem.setUpdater(this::animate);
	}

	public void animate(LEDStrips ledStrips) {
		if (timer.advanceIfElapsed(interval)) {
			if (ledsOn) {
				ledsOn = false;
				ledStrips.setAll(kBlack);
			} else {
				ledsOn = true;
				ledStrips.setAll(color);
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		ledSubsystem.setUpdater(null);
	}
}
