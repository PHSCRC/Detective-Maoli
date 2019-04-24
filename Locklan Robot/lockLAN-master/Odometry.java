/* Tyler Chen 11/7/17 Odometry Class
 * Tracks robot position using encoders.
 */

import java.util.Scanner;
import edu.princeton.cs.algs4.*;

public class Odometry {

    private double changeR; // Measured change in R
    private double changeL; // Measured change in L
    private double totalR; // Total distance traveled of right wheel
    private double totalL; // Total distance traveled of left wheel
    private double x; // Total x displacement
    private double y; // Total y displacement
    private double diameter; // Distance between wheels
    private double angle; // The angle between the wheels in radians
    private double field; // Radius of the robot's AT Field
    
    // Adds to the right wheel distance measurement. Should only be used for testing. 
    public void incrementRight(double changeR) {
	this.changeR = changeR;
	totalR += changeR;
    }

    // Adds to the left wheel distance measurement. Should only be used for testing.
    public void incrementLeft(double changeL) {
	this.changeL = changeL;
	totalL += changeL;
    }

    // Updates the total right wheel distance measurement
    public void setRight(double totalR) {
	changeR = totalR - this.totalR;
	this.totalR = totalR;
    }

    // Updates the total left wheel distance measurement
    public void setLeft(double totalL) {
	changeL = totalL - this.totalL;
	this.totalL = totalL;
    }

    // Updates odometry values
    public void updateOdometry() {
	double averageChange = (changeR + changeL)/2;
	angle += (changeR - changeL)/diameter;
	x += averageChange*Math.cos(angle);
	y += averageChange*Math.sin(angle);
    }

    // Prints odometry values
    public void printOdometry() {
	System.out.println("x = " + x + " y = " + y + " theta = " + angle);
    }

    // Returns x value
    public double getX() {
	return x;
    }

    // Returns y value
    public double getY() {
	return y;
    }

    // Returns angle theta
    public double getTheta() {
	return angle;
    }

    // Returns AT Field value
    public double getATField() {
	return field;
    }

    // Returns right wheel location
    public double[] getRightWheel() {
	return new double[] {x + (diameter/2)*Math.cos(Math.PI/2 - angle), y - (diameter/2)*Math.sin(Math.PI/2 - angle)};
    }

    // Returns left wheel location
    public double[] getLeftWheel() {
	return new double[] {x - (diameter/2)*Math.cos(Math.PI/2 - angle), y + (diameter/2)*Math.sin(Math.PI/2 - angle)};
    }

    // Draws the robot from a fixed reference point
    public void drawMap() {
	StdDraw.setPenColor(StdDraw.RED);
	StdDraw.filledCircle(x, y, 0.2);
	StdDraw.circle(x, y, field);
	StdDraw.setPenColor(StdDraw.BLACK);
	double[] rightWheel = getRightWheel();
	double[] leftWheel = getLeftWheel();
	StdDraw.filledCircle(rightWheel[0], rightWheel[1], 0.2);
	StdDraw.filledCircle(leftWheel[0], leftWheel[1], 0.2);
	StdDraw.setPenColor(StdDraw.BLUE);
	StdDraw.line(x, y, x + (diameter*2)*Math.cos(angle), y + (diameter*2)*Math.sin(angle));
    }

    public Odometry(double diameter, double field, double totalR, double totalL) {
	System.out.print("Initializing odometry... ");
	//StdDraw.setScale(-10, 10);
	//StdDraw.setCanvasSize(512, 512);
	this.totalR = totalR;
	this.totalL = totalL;
	angle = 0;
	this.diameter = diameter;
	this.field = field;
	System.out.println("Done!");
    }

    // Example showing estimated position based on wheel movement
    public static void main(String[] args) {
	Odometry  marvin = new Odometry(1, 0.8, 0, 0); // Initiate a robot with 1 unit distance between wheels
	Scanner sc = new Scanner(System.in);
	boolean end = false;
	while(!end) {
	    marvin.drawMap();
	    marvin.printOdometry();
	    System.out.print("Right wheel movement? ");
	    marvin.incrementRight(sc.nextDouble());
	    System.out.println();
	    System.out.print("Left wheel movement? ");
	    marvin.incrementLeft(sc.nextDouble());
	    System.out.println();
	    marvin.updateOdometry();
	}
    }
}
