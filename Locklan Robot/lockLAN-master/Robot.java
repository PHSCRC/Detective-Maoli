/* Tyler Chen 11/7/17 Robot Class
 * 
 */

import edu.princeton.cs.algs4.*;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.Arrays;

public class Robot {

    private Odometry odometer;
    private Pathfinding pathfinder;
    private Motors motors;
    private LinkedList<Move> moveQueue; // queue of moves to be executed
    private int scale;
    private double previousRightSpeed = 0;
    private double previousLeftSpeed = 0;
    private char previousRightDirection = 0;
    private char previousLeftDirection = 0;
    private SerialCommunicator motorCommunicator;
    private SerialCommunicator sensorCommunicator;
    private SerialCommunicator etcCommunicator;
    private SerialCommunicator colorCommunicator;
    private double optimalAngle = 0;
    private boolean enteredRoom = false; // has the robot entered a room?
    
    private final double CONVERT = 0.08;
    private final double RIGHTSPEED = 50/2;
    private final double LEFTSPEED = 94.34/2;
    private final double SENSORDISTANCE = 12.5;
    
    private class MapThread extends Thread {
	public void run() {
	    try {
		while(true) {
		    drawMap();
		    Thread.sleep(20);
		}
	    }

	    catch(Exception e) {
		e.printStackTrace();
	    }
	}
    }
    
    private class Align implements Move {
	private double initialTheta = odometer.getTheta();
	private double optimalTheta = initialTheta;
	private double rightTheta = initialTheta;
	private double rearTheta = initialTheta;
	private double minRight = Double.MAX_VALUE;
	private double minRear = Double.MAX_VALUE;
	
	public void execute() {
	    setSpeeds(RIGHTSPEED, LEFTSPEED, 'F', 'B');
	    if(!sensorCommunicator.getCurrentLine().contains(",")) {
		return;
	    }
	    //System.out.println(sensorCommunicator.getCurrentLine());
	    String[] raw = sensorCommunicator.getCurrentLine().split(",");
	    double frontDistance = Double.parseDouble(raw[0]);
	    double leftDistance = Double.parseDouble(raw[2]);
	    double rightDistance = Double.parseDouble(raw[6]);
	    double rearDistance = Double.parseDouble(raw[4]);
	    if(rightDistance < minRight && (frontDistance > 50 && leftDistance > 50)) {
		minRight = rightDistance;
		rightTheta = odometer.getTheta();
	    }
	    if(rearDistance < minRear && (frontDistance > 50 && leftDistance > 50)) {
		minRear = rearDistance;
		rearTheta = odometer.getTheta();
	    }
	    //System.out.println(minRight + ", " + minRear);
	}

	public boolean stop
	    () {
	    double currentAngle = odometer.getTheta();
	    if(currentAngle >= (initialTheta + (2*Math.PI))) {
		optimalTheta = (rightTheta + rearTheta)/2;
		optimalAngle = optimalTheta;
		return true;
	    }
	    return false;
	}
    }

    /*
    private class AlignWall implements Move {
	private boolean wall; // true = right, false = left
	private boolean stop;
	private double[] sensors;
	//contributed by Minecraft user trevdd



	// end contribution
	private double[] parseLine() {
	    String[] currentLine = sensorCommunicator.getCurrentLine().split(",");
	    System.out.println(sensorCommunicator.getCurrentLine());
	    double[] parsed = new double[8];
	    for(int i = 0; i <= parsed.length - 1; i++) {
		parsed[i] = Double.parseDouble(currentLine[i]);
	    }
	    return parsed;
	}

	
	public boolean stop() {
	    return stop;
	}

	public void execute() {
	    //contributed by Kess the Magnificant on deviantart
	    double[] data = parseLine();
	    double rightAv = (data[5] + data[6]+ data[7]) / 3.0;
	    double leftAv = (data[1] + data[2] + data[3]) / 3.0;
	    double rightDev = ((data[5] - rightAv) * (data[5] > rightAv ? 1 : -1) +
			       (data[6] - rightAv) * (data[6] > rightAv ? 1 : -1) +
			       (data[7] - rightAv) * (data[7] > rightAv ? 1 : -1)) / 3;
	    
	    double leftDev =  ((data[1] - leftAv) * (data[1] > leftAv ? 1 : -1) +                                                                                                           
                               (data[2] - leftAv) * (data[2] > leftAv ? 1 : -1) +                                                                                                           
                               (data[3] - leftAv) * (data[3] > leftAv ? 1 : -1)) / 3;  
	    wall = rightDev  < leftDev;
	    sensors = new double[3];
	    if(wall) {
		sensors[0] = data[5];
		sensors[1] = data[6];
		sensors[2] = data[7];
	    }
	    else {
		sensors[0] = data[3];
		sensors[1] = data[2];
		sensors[2] = data[1];
	    }
	    if((sensors[2] > sensors[0] && wall) || (sensors[2] < sensors[0] && !wall)) {
		setSpeeds(RIGHTSPEED, LEFTSPEED, 'B', 'F');
	    }
	    else {
		setSpeeds(RIGHTSPEED, LEFTSPEED, 'F', 'B');
	    }
	    //System.out.println(sensors[0] + ", " + sensors[2]); 
	    if(Math.abs(sensors[0] - sensors[2]) <= 0.1) {
		stop = true;
	    }
	}
	
	public AlignWall() {
	    stop = false;
	}
    }
    */

    private class AlignWall implements Move {
	private boolean foundOptimum = false;
	private double[] lastLine = new double[8];
	private double[] rearSensor = new double[10];
	private double[] frontSensor = new double[10];
	private int index = 0;
	
	public void execute() {
	    double[] currentLine = parseSensorLine();
	    if(currentLine != lastLine && index < 10) {
		rearSensor[index] = currentLine[5];
		frontSensor[index] = currentLine[7];
		index++;
	    }
	    if(index == 10) {
		Arrays.sort(rearSensor);
		Arrays.sort(frontSensor);
		double rearMedian  = rearSensor[5];
		double frontMedian = frontSensor[5];
		optimalAngle = ((2*Math.PI) + Math.atan((rearMedian - frontMedian)/SENSORDISTANCE)) % (2*Math.PI);
		foundOptimum = true;
	    }
	    lastLine = currentLine;
	}

	public boolean stop() {
	    return foundOptimum;
	}
    }

    private class TurnOptimal implements Move {
	private double theta; // the angle to be turned to
	private boolean direction; // true = turning left, false = turning right
	private boolean init;
	private double distance;
	
	public void execute() {
	    double adjustedAngle = odometer.getTheta() % (2*Math.PI);
	    if(direction) {
		/*
		setRightSpeed(RIGHTSPEED/2, 'F');
		setLeftSpeed(LEFTSPEED/2, 'B');
		*/
		setSpeeds(RIGHTSPEED, LEFTSPEED, 'F', 'B');
	    }
	    else if(!direction) {
		/*
		setRightSpeed(RIGHTSPEED/2, 'B');
		setLeftSpeed(LEFTSPEED/2, 'F');
		*/
		setSpeeds(RIGHTSPEED, LEFTSPEED, 'B', 'F');
	    }
	}

	public boolean stop() {
	    double adjustedAngle = odometer.getTheta() % (2*Math.PI);
	    if(adjustedAngle < 0) {
		adjustedAngle = (2*Math.PI) + adjustedAngle;
	    }
	    double currentDistance = Math.abs(adjustedAngle - theta);
	    if(init) {
		theta = (getOptimalAngle() + getTheta()) % (2*Math.PI);
		// Left
		if(adjustedAngle < theta) {
		    direction = true;
		}
		// Right
		else if(adjustedAngle > theta) {
		    direction = false;
		}
		
		if(Math.abs(adjustedAngle - theta) > Math.PI) {
		    direction = !direction;
		}
		init = false;
	    }
	    
	    if(/*currentDistance > (distance + 0.05) ||*/ Math.abs(adjustedAngle - theta) < 0.01)  {
		distance = currentDistance;
		return true;
	    }
	    /*
	    else if(!direction && (adjustedAngle < theta || Math.abs(adjustedAngle - theta) < 0.01)) {
		return true;
	    }
	    */
	    else {
		distance = currentDistance;
		return false;
	    }
	}
	
	public TurnOptimal() {
	    this.theta = (getOptimalAngle() + getTheta()) % (2*Math.PI);
	    init = true;
	    direction = true;
	    distance = Double.MAX_VALUE;
	}
    }

    private class Turn implements Move {
	private double theta; // the angle to be turned to
	private boolean direction; // true = turning left, false = turning right
	private boolean init;
	private double distance;
	
	public void execute() {
	    double adjustedAngle = odometer.getTheta() % (2*Math.PI);
	    if(direction) {
		/*
		setRightSpeed(RIGHTSPEED/2, 'F');
		setLeftSpeed(LEFTSPEED/2, 'B');
		*/
		setSpeeds(RIGHTSPEED, LEFTSPEED, 'F', 'B');
	    }
	    else if(!direction) {
		/*
		setRightSpeed(RIGHTSPEED/2, 'B');
		setLeftSpeed(LEFTSPEED/2, 'F');
		*/
		setSpeeds(RIGHTSPEED, LEFTSPEED, 'B', 'F');
	    }
	}

	public boolean stop() {
	    double adjustedAngle = odometer.getTheta() % (2*Math.PI);
	    if(adjustedAngle < 0) {
		adjustedAngle = (2*Math.PI) + adjustedAngle;
	    }
	    double currentDistance = Math.abs(adjustedAngle - theta);
	    if(init) {
		// Left
		if(adjustedAngle < theta) {
		    direction = true;
		}
		// Right
		else if(adjustedAngle > theta) {
		    direction = false;
		}
		
		if(Math.abs(adjustedAngle - theta) > Math.PI) {
		    direction = !direction;
		}
		init = false;
	    }
	    
	    if(/*currentDistance > (distance + 0.05) ||*/ Math.abs(adjustedAngle - theta) < 0.01)  {
		distance = currentDistance;
		return true;
	    }
	    /*
	    else if(!direction && (adjustedAngle < theta || Math.abs(adjustedAngle - theta) < 0.01)) {
		return true;
	    }
	    */
	    else {
		distance = currentDistance;
		return false;
	    }
	}
	// "my wife's going to jamaica tomorrow?", "to jamaica?", "no she's going on her own accord" 
	public Turn(double theta) {
	    this.theta = theta;
	    init = true;
	    direction = true;
	    distance = Double.MAX_VALUE;
	}
    }

    private class ForwardUntilRight implements Move {

	private double[] parseLine() {
	    String[] currentLine = sensorCommunicator.getCurrentLine().split(",");
	    //System.out.println(sensorCommunicator.getCurrentLine());
	    double[] parsed = new double[8];
	    for(int i = 0; i <= parsed.length - 1; i++) {
		parsed[i] = Double.parseDouble(currentLine[i]);
	    }
	    return parsed;
	}
	
	public void execute() {
	    setSpeeds(RIGHTSPEED, LEFTSPEED, 'F', 'F');
	}
	public boolean stop() {
	    double[] sensors = parseLine();
	    if(sensors[5] > 40 && sensors[6] > 40 && sensors[7] > 40) {
		return true;
	    }
	    else if(sensors[0] < 25) {
		return true;
	    }
	    return false;
	}
    }

    /*
    private class InverseForwardUntilRight implements Move {

	private double[] parseLine() {
	    String[] currentLine = sensorCommunicator.getCurrentLine().split(",");
	    System.out.println(sensorCommunicator.getCurrentLine());
	    double[] parsed = new double[8];
	    for(int i = 0; i <= parsed.length - 1; i++) {
		parsed[i] = Double.parseDouble(currentLine[i]);
	    }
	    return parsed;
	}
	
	public void execute() {
	    setSpeeds(RIGHTSPEED, LEFTSPEED, 'F', 'F');
	}
	public boolean stop() {
	    double[] sensors = parseLine();
	    if(sensors[5] < 40 && sensors[6] < 40 && sensors[7] < 40) {
		return true;
	    }
	    else if(sensors[0] < 30) {
		return true;
	    }
	    return false;
	}
    }
    */

    private class Forward implements Move {
	private double distance; // the distance to be moved
	private double initX;
	private double initY;
	private boolean init;
	
	public void execute() {
	    /*
	    setLeftSpeed(LEFTSPEED, 'F');
	    setRightSpeed(RIGHTSPEED, 'F');
	    */
	    System.out.println("Executing forward");
	    setSpeeds(RIGHTSPEED, LEFTSPEED, 'F', 'F');
	}

	public boolean stop() {
	    if(init) {
		initX = odometer.getX();
		initY = odometer.getY();
		init = false;
	    }
	    double currentDistance = Math.hypot(odometer.getX() - initX, odometer.getY() - initY);
	    System.out.println(currentDistance);
	    return currentDistance  > distance || Math.abs(currentDistance - distance) < 0.01;
	}
	
	public Forward(double distance) {
	    this.distance = distance;
	    init = true;
	    initX = 0;
	    initY = 0;
	}
    }

    public void turnOnFan() {
	etcCommunicator.sendString("1~");
    }

    public void checkColor() {
	if(colorCommunicator.getCurrentLine().equals("WHITE"))
	    enteredRoom = true;
    }

    public boolean hasEnteredRoom() {
	return enteredRoom;
    }

    public boolean soundStart() {
	if(etcCommunicator.getCurrentLine().equals("sound"))
	    return true;
	return false;
    }

    public boolean fire() {
	if(Integer.parseInt(etcCommunicator.getCurrentLine()) < 40) {
	    return true;
	}
	return false;
    }

    public void leaveRoom() {
	enteredRoom = false;
    }
    
    public void drawMap() {
	StdDraw.clear();
	pathfinder.drawMap();
	odometer.drawMap();
	StdDraw.show();
    }

    public double[] parseSensorLine() {
	    String[] currentLine = sensorCommunicator.getCurrentLine().split(",");
	    //System.out.println(sensorCommunicator.getCurrentLine());
	    double[] parsed = new double[8];
	    for(int i = 0; i <= parsed.length - 1; i++) {
		parsed[i] = Double.parseDouble(currentLine[i]);
	    }
	    return parsed;
    }

    public Robot(int scale, Odometry odometer, Pathfinding pathfinder, Motors motors) {
	this.odometer = odometer;
	this.pathfinder = pathfinder;
	this.scale = scale;
	this.motors = motors;
	moveQueue = new LinkedList<Move>();
	//moveQueue.add(new Align());
	//moveQueue.add(new Turn(Math.PI/2));
	//moveQueue.add(new Forward(4));
	//moveQueue.add(new Turn(Math.PI));
	//moveQueue.add(new Forward(4));

	System.out.print("Initializing motor serial communicator... ");
	motorCommunicator = new SerialCommunicator("/dev/ttyUSB0");
	motorCommunicator.initialize();
	Thread t = new Thread() {
		public void run() {
		    try {Thread.sleep(Long.MAX_VALUE);} catch (InterruptedException ie) {}
		}
	    };
	t.start();
	System.out.println("Done!");

	System.out.print("Initializing sensor serial communicator... ");
	sensorCommunicator = new SerialCommunicator("/dev/ttyUSB1");
	sensorCommunicator.initialize();
	Thread t2 = new Thread() {
		public void run() {
		    try {Thread.sleep(Long.MAX_VALUE);} catch (InterruptedException ie) {}
		}
	    };
	t2.start();
	System.out.println("Done!");

	System.out.print("Initializing etc serial communicator... ");
	etcCommunicator = new SerialCommunicator("/dev/ttyUSB2");
	etcCommunicator.initialize();
	Thread t3 = new Thread() {
		public void run() {
		    try {Thread.sleep(Long.MAX_VALUE);} catch (InterruptedException ie) {}
		}
	    };
	t3.start();
	System.out.println("Done!");

	System.out.print("Initializing color sensor serial communicator... ");
	colorCommunicator = new SerialCommunicator("/dev/ttyACM0");
	colorCommunicator.initialize();
	Thread t4 = new Thread() {
		public void run() {
		    try {Thread.sleep(Long.MAX_VALUE);} catch (InterruptedException ie) {}
		}
	    };
	t4.start();
	System.out.println("Done!");
	
	
	StdDraw.setScale(scale*(-1), scale);
	StdDraw.enableDoubleBuffering();
    }

    public double getTheta() {
	return odometer.getTheta();
    }

    public void addMove(Move aMove) {
	moveQueue.add(aMove);
    }

    public boolean isEmpty() {
	return moveQueue.isEmpty();
    }

    public void markObstacles() {
	if(!sensorCommunicator.getCurrentLine().contains(",")) {
	    return;
	}
	//System.out.println(sensorCommunicator.getCurrentLine());
	String[] raw = sensorCommunicator.getCurrentLine().split(",");
	double angle = 0;
	for(int i = 0; i <= raw.length - 1; i++) {
	    double robotAngle = odometer.getTheta();
	    double distance = Double.parseDouble(raw[i])*CONVERT;
	    double robotX = odometer.getX();
	    double robotY = odometer.getY();
	    pathfinder.mark(robotX, robotY, robotX + distance*Math.cos(robotAngle + angle), robotY + distance*Math.sin(robotAngle + angle));
	    angle += (2*Math.PI)/(raw.length);
	}
    }

    public void updateDistances() {
	String[] raw;
	//System.out.println(motorCommunicator.getCurrentLine());
	if(!motorCommunicator.getCurrentLine().contains(",")) {
	    return;
	}
	else {
	    raw = motorCommunicator.getCurrentLine().split(",");
	}
	//System.out.println(raw[0] + "," + raw[1]);
	odometer.setRight(CONVERT * Double.parseDouble(raw[0]));
	odometer.setLeft(CONVERT * Double.parseDouble(raw[1]));
    }

    public void startDrawing() {
	MapThread mapDrawer = new MapThread();
	mapDrawer.start();
    }

    public void checkQueue() {
	if(moveQueue.isEmpty()/* || motorCommunicator.getCurrentLine() == ""*/) {
	    return;
	}
	if(fire()) {
	    stopMotors();
	    moveQueue.clear();
	    System.out.println("holy shit a fire");
	    return;
	}
	
	//System.out.println(moveQueue);
	Move currentMove = moveQueue.peek();
	if(!currentMove.stop()) currentMove.execute();
	else {
	    stopMotors();
	    moveQueue.remove();
	    //drawMap();
	    System.out.println("Finished move, current position: ");
	    System.out.println("X: " + odometer.getX() + " Y: " + odometer.getY() + " Angle: " + odometer.getTheta());
	}
    }

    public void addPath(double initX, double initY, double finalX, double finalY) {
	ArrayList<Double[]> path = pathfinder.path(initX, initY, finalX, finalY);
	double previousTheta = odometer.getTheta();
	double cumulativeDistance = 0;
	for(int i = path.size() - 2; i >= 0; i--) {
	    //System.out.println(path.get(i + 1)[0] + " " + path.get(i + 1)[1] + "||" + path.get(i)[0] + " " + path.get(i)[1]);
	    double relX = path.get(i)[0] - path.get(i + 1)[0];
	    double relY = path.get(i)[1] - path.get(i + 1)[1];
	    double distance = Math.hypot(relX, relY);
	    double theta = 0;
	    
	    // Handling atan2 special cases
	    if(relX == 0 && relY > 0) {
		theta = Math.PI/2;
	    }
	    else if(relX == 0 && relY < 0) {
		theta = (3/2) * Math.PI;
	    }
	    else if(relY == 0 && relX > 0) {
		theta = 0;
	    }
	    else if(relY == 0 && relX < 0) {
		theta = Math.PI;
	    }
	    else {
		theta = Math.atan2(relY, relX);
		if(theta < 0) {
		    theta = (2*Math.PI) + theta;
		}
	    }

	    /*
	    // If in Quadrant IV
	    if(relX > 0 && relY < 0) {
		theta += 2*Math.PI;
	    }
	    // If in Quadrant II or III
	    else if(relX < 0) {
		theta += Math.PI;
	    }
	    */
	    
	    //System.out.println("Moving " + distance + " at angle " + theta);
	    //System.out.println(relX + " " + relY);
	    
	    if(theta != previousTheta) {
		moveQueue.add(new Forward(cumulativeDistance));
		moveQueue.add(new Turn(theta));
		cumulativeDistance = 0;
		previousTheta = theta;
	    }

	    cumulativeDistance += distance;
	}
	
	if(cumulativeDistance != 0) {
	    moveQueue.add(new Forward(cumulativeDistance));
	}
    }

    // Use setSpeeds instead
    @Deprecated
    public void setRightSpeed(double speed, char direction) {
	motorCommunicator.sendString("A" + direction + speed + "~");
    }

    // Use setSpeeds instead
    @Deprecated
    public void setLeftSpeed(double speed, char direction) {
	motorCommunicator.sendString("B" + direction + speed + "~");
    }

    public void setSpeeds(double rightSpeed, double leftSpeed, char rightDirection, char leftDirection) {
	if(!(rightSpeed == previousRightSpeed && leftSpeed == previousLeftSpeed && rightDirection == previousRightDirection && leftDirection == previousLeftDirection)) {
	    motorCommunicator.sendString("A" + rightDirection + rightSpeed + "," + "B" + leftDirection + leftSpeed + "~");
	    previousRightSpeed = rightSpeed;
	    previousLeftSpeed = leftSpeed;
	    previousRightDirection = rightDirection;
	    previousLeftDirection = leftDirection;
	}
    }

    public double getOptimalAngle() {
	return optimalAngle;
    }

    public void stopMotors() {
	motorCommunicator.sendString("0~");
    }

    public void updateOdometry() {
	odometer.updateOdometry();
    }

    public static void main(String args[]) throws Exception {
	Robot yui = new Robot(20, new Odometry(30*0.08, 2, 0, 0), new Pathfinding(20, 2), new Motors());	
	/*
	double rightSpeed = 0.4;
	double leftSpeed = 0.7;

        yui.setLeftSpeed(leftSpeed);
	yui.setRightSpeed(rightSpeed);
	*/
	/*
	yui.addPath(0, 0, 0, -6);
	yui.addPath(0, -6, 0, 0);
	*/
	Thread.sleep(3000);
        //yui.startDrawing();
	System.out.println("Waiting for tone...");
	while(!yui.soundStart());

	double[] sensors = yui.parseSensorLine();
	if(sensors[5] > 40 && sensors[6] > 40 && sensors[7] > 40) {
	    yui.addMove(yui.new Turn(((yui.getTheta() + 3.0*(Math.PI/2)) % (2*Math.PI))));
	}
	
	while(true) {
	    try {
		while(!yui.isEmpty()) {
		    yui.updateDistances();
		    yui.updateOdometry();
		    yui.checkQueue();
		    yui.checkColor();
		    //yui.markObstacles();
		    Thread.sleep(0);
		}
	    }
	    catch(InterruptedException e) {
		e.printStackTrace();
	    }

	    sensors = yui.parseSensorLine();
	    if(yui.hasEnteredRoom()) {
		yui.addMove(yui.new Turn((yui.getTheta() + Math.PI) % (2*Math.PI)));
		yui.addMove(yui.new Turn((yui.getTheta() + Math.PI) % (2*Math.PI)));
	        yui.leaveRoom();
	    }
	    else if(sensors[5] > 40 && sensors[6] > 40 && sensors[7] > 40) {
		System.out.println("Turning right... ");
		yui.addMove(yui.new Forward(0.4));
	        yui.addMove(yui.new Turn(((yui.getTheta() + 3.0*(Math.PI/2)) % (2*Math.PI))));
		yui.addMove(yui.new Forward(1.6));
	    }
	    else if(sensors[0] < 25) {
		System.out.println("Turning left... ");
		//System.out.println((yui.getTheta() + (Math.PI/2)));
		yui.addMove(yui.new Turn(((yui.getTheta() + (Math.PI/2)) % (2*Math.PI))));
	    }
	    else {
		System.out.println("Going straight... ");
		if(sensors[5] < 40 && sensors[6] < 40 && sensors[7] < 40) {
		    yui.addMove(yui.new AlignWall());
		    //System.out.println(yui.getOptimalAngle());
		    yui.addMove(yui.new TurnOptimal());
		}
		yui.addMove(yui.new ForwardUntilRight());
	    }
	}

	
	/*
	Turn aTurn = yui.new Turn(yui.getOptimalAngle());
	yui.addMove(aTurn);
	while(!yui.isEmpty()) {
	    yui.updateDistances();
	    yui.updateOdometry();
	    yui.checkQueue();
	    //yui.markObstacles();
		
	}
	*/
    }
}
