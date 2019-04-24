/*
 * Kees' New Robot Class
 */
// control f '***' to find stuff not implemented (for the most part)
//import edu.princeton.cs.algs4.*;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.Arrays;


public class Robot {
    //constants:
    private final int WALL_THRESHOLD = 20; //threshhold for wall to be considered there
    public final int SAMPLE_SIZE = 10;
    private final double ANGLE_THRESHOLD = 0.01;
    private final double FORWARD_THRESHOLD = 0.01;
    private final double RIGHTSPEED = 50;
    private final double LEFTSPEED = 94.34;
    private final double CONVERT = 0.08;
    private final int FLAME_IN_ROOM = 800;
    private final int FLAME_INFRONT = 40;
    private final double SENSORDISTANCE = 12.5;
    private final double TWOPI = 2*Math.PI;
    //doing private classes first:



    //instance fields:
    private Odometry odometer;
    private SerialCommunicator motorCommunicator;
    private SerialCommunicator sensorCommunicator;
    private SerialCommunicator etcCommunicator;
    private SerialCommunicator colorCommunicator;
    private boolean detectedRoom;
    private boolean inRoom;
    private double[] sensors; 
    /* really only the front and right ones are used, but they are as follows:
       0 = front
       1 = frontLeft
       2 = midLeft
       3 = backLeft
       4 = behind (this one is useless)
       5 = backRight
       6 = midRight
       7 = frontRight
    */
    //methods:
    public Robot(){ //tyler
	sensors = new double[8];
	Odometry odometer = new Odometry(30*0.08, 2, 0, 0);

	// INITIALIZE SERIAL COMMUNICATORS

	System.out.print("Initializing motor serial communicator... ");
	motorCommunicator = new SerialCommunicator("/dev/ttyUSB0");
	motorCommunicator.initialize();
	Thread t = new Thread() {
		public void run() {
		    try {Thread.sleep(Long.MAX_VALUE);} catch (InterruptedException ie) {

		    }
		}
	    };
	t.start();
	System.out.println("Done!");
	System.out.print("Initializing sensor serial communicator... ");
	sensorCommunicator = new SerialCommunicator("/dev/ttyUSB1");
	sensorCommunicator.initialize();
	Thread t2 = new Thread() {
		public void run() {
		    try {Thread.sleep(Long.MAX_VALUE);} catch (InterruptedException ie) {

		    }
		}
	    };			t2.start();
	System.out.println("Done!");

	System.out.print("Initializing etc serial communicator... ");
	etcCommunicator = new SerialCommunicator("/dev/ttyUSB2");
	etcCommunicator.initialize();
	Thread t3 = new Thread() {
		public void run() {
		    try {Thread.sleep(Long.MAX_VALUE);} catch (InterruptedException ie) {

		    }
		}
	    };
	t3.start();
	System.out.println("Done!");

	System.out.print("Initializing color sensor serial communicator... ");
	colorCommunicator = new SerialCommunicator("/dev/ttyACM0");
	colorCommunicator.initialize();
	Thread t4 = new Thread() {
		public void run() {
		    try {Thread.sleep(Long.MAX_VALUE);} catch (InterruptedException ie) {

		    }
		}
	    };
	t4.start();
	System.out.println("Done!");

	// END INITIALIZATION OF SERIAL COMMUNICATORS

	System.out.println("Starting Odometry thread");
	Thread odometryThread = new Thread() {
		public void run() {
		    try {
			while(true) {
			    String[] raw;
			    if(!motorCommunicator.getCurrentLine().contains(",")) {
				continue;
			    }
			    raw = motorCommunicator.getCurrentLine().split(",");
			    odometer.setRight(CONVERT * Double.parseDouble(raw[0]));
			    odometer.setLeft(CONVERT * Double.parseDouble(raw[1]));
			    odometer.updateOdometry();
			}
		    }
		    catch(Exception e) {
			e.printStackTrace();
		    }
		}
	    };
	odometryThread.start();
	System.out.println("Done!");

	System.out.println("Starting color sensor thread");
	Thread colorThread = new Thread() {
		public void run() {
		    if(colorCommunicator.getCurrentLine().equals("WHITE"))
			detectedRoom = true;
		}
	    };
	colorThread.start();
	System.out.println("Done!");
    }
    public boolean soundStart(){ //returns true if the sound has been detected, false otherwise
	//tyler
	return etcCommunicator.getCurrentLine().equals("sound");
    }
    public void readSensors(int sampleSize){
	//tyler
	int i = 0;
	double[][] raw = new double[8][sampleSize];
	String[] previousLine = new String[8];
	while(i < sampleSize) {
	    String[] currentLine = sensorCommunicator.getCurrentLine().split(",");
	    if(currentLine.equals(previousLine))
		continue;
	    for(int j = 0; j <= raw.length - 1; j++) {
		raw[j][i] = Double.parseDouble(currentLine[j]);
	    }
	    previousLine = currentLine;
	    i++;
	}

	for(int k = 0; k <= sensors.length - 1; k++) {
	    Arrays.sort(raw[k]);
	    sensors[k] = raw[k][(int) Math.round(sampleSize/2)];
	}
    }
    public boolean wallRight(){ //done
	return (sensors[5] <= WALL_THRESHOLD ||
		sensors[6] <= WALL_THRESHOLD ||
		sensors[7] <= WALL_THRESHOLD); //returns true if any sensor on the right detects a wall
    }

    // Aditya
    public boolean roomDetected() {
	detectedRoom = false;
	if(colorCommunicator.getCurrentLine().equals("WHITE"))
	    detectedRoom = true;
	return detectedRoom;
    }



    public boolean wallFront(){ //done
	return (sensors[0] <= WALL_THRESHOLD); //returns true if front sensor detects wall
    }

    public void turnRight(){ //turns right tbh
	turn(-1 * Math.PI * 0.5);
    }

    public void turnLeft(){ //turns left tbh
	turn(Math.PI * 0.5);
    }

    public void alignRight(){
	readSensors(SAMPLE_SIZE);
	double front = sensors[7];
	double rear = sensors[5];
	double goodAngle = (TWOPI + Math.acos((rear - front)/SENSORDISTANCE)) % TWOPI;
	turn(goodAngle);
    }

    public void alignLeft(){
	readSensors(SAMPLE_SIZE);
	double front = sensors[1];
	double rear = sensors[3];
	double goodAngle = (TWOPI + Math.acos((rear - front)/SENSORDISTANCE)) % TWOPI;
	turn(goodAngle);
    }


    public void turn(double theta) {//arbitrary angle
	double angle = odometer.getTheta() + (theta + TWOPI) % (TWOPI);
	if (theta > 0) { //left
	    setSpeeds(RIGHTSPEED, LEFTSPEED, 'F', 'B');
	}
	while(Math.abs((odometer.getTheta() % TWOPI) -  angle) < ANGLE_THRESHOLD){

	} //go until it matches, doing nothing
	setSpeeds(0);
    }

    public boolean shouldStop(){
	String[] currentLine = sensorCommunicator.getCurrentLine().split(",");
	double[] sensors = new double[8];
	for(int i = 0; i < 8; i++){
	    sensors[i] = Double.parseDouble(currentLine[i]);
	}
	return (sensors[0] <= 25 || (
				     sensors[5] >= 30 &&
				     sensors[6] >= 30 &&
				     sensors[7] >= 30));
    }


    public void forward(double d){//forward a distance
	setSpeeds(RIGHTSPEED, LEFTSPEED, 'F', 'F');
	double initX = odometer.getX();
	double initY = odometer.getY();
	while (Math.abs(Math.hypot(odometer.getX() - initX, odometer.getY() - initY) - d) < FORWARD_THRESHOLD ) {

	} //goes until distance moved = d;
	setSpeeds(0);
    }


    public void forward() { // indefinite forward
	setSpeeds(RIGHTSPEED, LEFTSPEED, 'F', 'F');
    }

    public void setSpeeds(int i){
	if (i == 0){
	    motorCommunicator.sendString("0~");
	}
    }

    public void setSpeeds(double rightSpeed, double leftSpeed, char rightDirection, char leftDirection) {
	motorCommunicator.sendString("A" + rightDirection + rightSpeed + "," + "B" + leftDirection + leftSpeed + "~");
    }

    public boolean flameScan(){
	boolean detected = false;
	for(int i = 0; i < 3; i++){
	    detected = Integer.parseInt(etcCommunicator.getCurrentLine()) <= FLAME_IN_ROOM;
	    turn(Math.PI * (1.0/6));
	}
	turn(-Math.PI*0.5);
	for(int i = 0; i < 3; i++){
	    detected = Integer.parseInt(etcCommunicator.getCurrentLine()) <= FLAME_IN_ROOM;
	    turn(Math.PI * (-1.0/6));
	}
	turn(Math.PI*0.5);
	return detected;
    }
    //main with moving logic
    public static void main(String [] args) {
	Robot rj = new Robot();
	try {
	    Thread.sleep(3000); //give other stuff time to start up
	}
	catch(Exception e) {e.printStackTrace();}
	System.out.println("Waiting for start tone...");
	while(!rj.soundStart());
	rj.readSensors(rj.SAMPLE_SIZE);

	while (true){ //maze navigation logic
	    rj.readSensors(rj.SAMPLE_SIZE);
	    //any other stuff to update goes here
	    if(rj.roomDetected()){ //if there's a room,
		if(rj.flameScan()){ //if it found one
		    flameDetectLoop(rj); //do our loop made for when it does
		    return;
		}
	    }
	    //after roomDetected() return to spot before roomDetected()
	    if(!rj.wallRight()){
		rj.turnRight();
		rj.forward(20); //d=20 chosen by Kees at random basically
	    }

	    //if there's a fat L wall in front
	    else if (rj.wallFront()){
		rj.turnLeft();
	    }

	    //if no wall in front and a wall to the right, move forward
	    else{
		rj.forward();
		while(!rj.shouldStop()){
		}
		rj.setSpeeds(0);
	    }
	}
    }


    //other static methods

    //will do some stuff when a flame has been found\
    //can only start when in a room already
    public static void flameDetectLoop(Robot rj){
	while (true){
	    /***SHIT WILL BE HERE AT SOME POINT***/
	}
    }
}
