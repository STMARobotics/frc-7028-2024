package frc.robot.commands.led;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command to create an animation on the LEDs where ever other LED lights up with a given color, and
 * they swap on an interval.
 */
public class LEDAlternateCommand extends Command {

	private final LEDSubsystem ledSubsystem;
	private final Color color1;
	private final Color color2;
	private final double interval;

	private final Timer timer = new Timer();
	private boolean refresh = false;

	/**
	 * Constructor
	 *
	 * @param ledSubsystem led subsystem
	 * @param color1 first color to light up the LEDs
	 * @param color2 second color to light up the LEDs
	 * @param interval interval to swap the colors
	 */
	public LEDAlternateCommand(LEDSubsystem ledSubsystem, Color color1, Color color2, Time interval) {
		this.ledSubsystem = ledSubsystem;
		this.color1 = color1;
		this.color2 = color2;
		this.interval = interval.in(Seconds);

		addRequirements(ledSubsystem);
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
		ledSubsystem.setUpdater(this::animate);
		refresh = true;
	}

	public void animate(LEDStrips ledStrips) {
		if (timer.advanceIfElapsed(interval) || refresh) {
			long currentTime = System.currentTimeMillis();
			for (int strip = 0; strip < ledStrips.getStripCount(); strip++) {
				for (int index = 0; index < ledStrips.getStripSize(strip); index++) {
					if (index % 2 == (currentTime / (int) (interval * 1000) % 2)) {
						ledStrips.setLED(strip, index, color1);
					} else {
						ledStrips.setLED(strip, index, color2);
					}
				}
			}
			ledStrips.refresh();
			refresh = false;
		}
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		ledSubsystem.setUpdater(null);
	}
}
