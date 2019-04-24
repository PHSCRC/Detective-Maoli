/* Tyler Chen 1/30/18
 * A class that simulates the data received from rotary encoders
 */

import java.util.TimerTask;
import java.util.Timer;

public class Motors {

    private class UpdateDistances extends TimerTask {
	@Override
	public void run() {
	    leftDistance += leftSpeed*(1/frequency); // multiplying by period
	    rightDistance += rightSpeed*(1/frequency);
	}
    }
    
    private double leftSpeed; // in units per second
    private double rightSpeed; // in units per second
    private double leftDistance; // linear distance traveled of left
    private double rightDistance; // linear distance traveled of right
    private double frequency; // how many times per second speed is updated

    public void setLeftSpeed(double speed) {
	leftSpeed = speed;
    }

    public void setRightSpeed(double speed) {
	rightSpeed = speed;
    }

    public double getLeftDistance() {
	return leftDistance;
    }

    public double getRightDistance() {
	return rightDistance;
    }
    
    public Motors() {
	System.out.print("Initializing motor simulation... ");
	leftSpeed = 0;
	rightSpeed = 0;
	frequency = 144; // updates speed at 144 Hz
	Timer timer = new Timer(true); // initiates a daemon thread timer
	TimerTask encoder = new UpdateDistances();;
	timer.schedule(encoder, 0, (long) ((1/frequency)*1000)); // period is converted to millis
	System.out.println("Done!");
    }
}
