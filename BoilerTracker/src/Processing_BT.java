import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Processing_BT {

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
	// Process for GRIP
	static BoilerTracker tracker;
	public static VideoCapture videoCapture;
	// Constants for known variables
	static Mat matOriginal;
	public static final double OFFSET_TO_FRONT = 0;
	public static final double CAMERA_WIDTH = 640;
	public static final double DISTANCE_CONSTANT = 5738;
	public static final double WIDTH_BETWEEN_TARGET = 8.5;
	public static boolean shouldRun = true;

	public static final boolean enableNetworkTables = true;
	public static final boolean showDebuggingStatements = true;
	static NetworkTable table;

	static double lengthBetweenContours;
	static double distanceFromTarget;
	static double lengthError;
	static double[] centerX;

	static double lastDistance;

	public static void main(String[] args) {
		if (enableNetworkTables) {
			NetworkTable.setClientMode();
			NetworkTable.setTeam(1806);
			NetworkTable.setIPAddress("roborio-3216-FRC.local");
			NetworkTable.initialize();
			table = NetworkTable.getTable("BoilerTracker");

			if (showDebuggingStatements) {
				System.out.println(NetworkTable.connections());
			} // end if
		} // end if

		while (shouldRun) {
			try {
				// opens up the camera stream and tries to load it
				videoCapture = new VideoCapture();
				tracker = new BoilerTracker();

				// Open the default video device
				videoCapture.open(0);

				// If the stream couldn't be opened, finish looping
				if (!videoCapture.isOpened()) {
					System.out.println("Didn't open Camera, restart jar");
					break;
				} // end if

				// time to actually process the acquired images
				while (videoCapture.isOpened()) {
					processImage();
				} // end while()

			} catch (Exception e) {
				e.printStackTrace();
				break;
			} // end try
		} // end while

		// make sure the java process quits when the loop finishes
		videoCapture.release();
		System.exit(0);
	} // end main

	public static void processImage() {
		System.out.println("Processing Started");
		matOriginal = new Mat();

		// Loop forever????
		while (true) {
			videoCapture.read(matOriginal);
			tracker.process(matOriginal);
			returnCenterX();
			if (enableNetworkTables) {
				table.putNumber("distanceFromTarget", distanceFromTarget());
				table.putNumber("angleFromGoal", getAngle());
				// table.putNumberArray("centerX", centerX);
			} // end if
			else {
				// Just call to test
				distanceFromTarget();
				getAngle();
			} // end else
			videoCapture.read(matOriginal);
		} // end while
	} // end processImage()

	public static double returnCenterX() {
		// This is the center value returned by GRIP thank WPI
		if (!tracker.filterContoursOutput.isEmpty() && tracker.filterContoursOutput.size() >= 2) {
			Rect r = Imgproc.boundingRect(tracker.filterContoursOutput.get(1));
			Rect r1 = Imgproc.boundingRect(tracker.filterContoursOutput.get(0));
			centerX = new double[] { r1.x + (r1.width / 2), r.x + (r.width / 2) };
			Imgcodecs.imwrite("output.png", matOriginal);

			if (showDebuggingStatements) {
				System.out.println("Center: " + centerX.length); // testing
			}

			// this again checks for the 2 shapes on the target
			if (centerX.length == 2) {
				// subtracts one another to get length in pixels
				lengthBetweenContours = Math.abs(centerX[0] - centerX[1]);
			} // end if
		} // end if
		return lengthBetweenContours;
	} // end returnCenterX

	public static double distanceFromTarget() {
		// distance constant divided by length between centers of contours
		distanceFromTarget = DISTANCE_CONSTANT / lengthBetweenContours;


	public static double getAngle() {
		// 8.5in is for the distance from center to center from goal, then
		// divide by lengthBetweenCenters in pixels to get proportion
		double constant = WIDTH_BETWEEN_TARGET / lengthBetweenContours;
		double angleToGoal = 0;
		// Looking for the 2 blocks to actually start trig
		if (!tracker.filterContoursOutput.isEmpty() && tracker.filterContoursOutput.size() >= 2) {
			if (centerX.length == 2) {
				// this calculates the distance from the center of goal to
				// center of webcam
				double distanceFromCenterPixels = ((centerX[0] + centerX[1]) / 2) - (CAMERA_WIDTH / 2);

				// Converts pixels to inches using the constant from above.
				double distanceFromCenterInch = distanceFromCenterPixels * constant;

				// math brought to you buy Chris and Jones
				angleToGoal = Math.atan(distanceFromCenterInch / distanceFromTarget());
				angleToGoal = Math.toDegrees(angleToGoal);

				// prints angle
				if (showDebuggingStatements) {
					System.out.println("Angle: " + angleToGoal); // testing
				} // end if
			} // end if
		} // end if
		return angleToGoal;
	} // end getAngle
} // end Processing_LT
