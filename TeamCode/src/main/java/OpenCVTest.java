import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static java.lang.Math.abs;

@Disabled
@TeleOp (name = "OpenCVTest")
public class OpenCVTest extends LinearOpMode
{
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;

    Servo shooterServo1;
    Servo shooterServo2;

    public static int x_anchor1 = 150;
    public static int x_anchor2 = 242;
    public static int x_anchor3 = 233;

    public static int y_anchor1 = 3;
    public static int y_anchor2 = 39;
    public static int y_anchor3 = 49;

    public static int height1 = 55;
    public static int height2 = 8;
    public static int height3 = 12;

    public static int width1 = 70;
    public static int width2 = 43;
    public static int width3 = 50;

    @Override
    public void runOpMode()

    {

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "externalWebcam"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        pipeline = new SkystoneDeterminationPipeline();
        pipeline.setHeightAndWidth(height1, height2, width1, width2, height3, width3);
        pipeline.setAnchorPoints2(x_anchor1, y_anchor1, x_anchor2, y_anchor2, x_anchor3, y_anchor3);
        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        FtcDashboard.getInstance().startCameraStream(webcam,100);
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Analysis1", pipeline.getAnalysis());
            telemetry.addData("Analysis2", pipeline.getAnalysis2());
            telemetry.addData("Analysis3", pipeline.getAnalysis3());

            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    /*
     * NOTE: stopping the stream from the camera early (before the end of the OpMode
     * when it will be automatically stopped for you) *IS* supported. The "if" statement
     * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
     */
            /*if(gamepad1.a)
            /*{
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                /*webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            /*sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    @Config
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        public static int x_anchor1 = 25;
        public static int y_anchor1 = 180;
        public static int x_anchor2 = 25;
        public static int y_anchor2 = 75;
        public static int x_anchor3 = 25;
        public static int y_anchor3 = 75;

        static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(x_anchor1,y_anchor1);
        static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(x_anchor2,y_anchor2);
        static Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(x_anchor3,y_anchor3);


        public static  int REGION1_WIDTH = 70;
        public static  int REGION1_HEIGHT = 80;
        public static  int REGION2_WIDTH = 70;
        public static  int REGION2_HEIGHT = 100;
        public static  int REGION3_WIDTH = 70;
        public static  int REGION3_HEIGHT = 125;


        public static int FOUR_RING_THRESHOLD = 132;
        public static int ONE_RING_THRESHOLD = 129;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);
        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat region2_Cb;
        Mat region3_Cb;

        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        int avg2;
        int avg3;
        int threshold = 7;
        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        public void setHeightAndWidth(int height1, int height2, int width1, int width2, int height3, int width3) {
            REGION1_WIDTH = width1;
            REGION2_WIDTH = width2;
            REGION3_WIDTH = width3;
            REGION1_HEIGHT = height1;
            REGION2_HEIGHT = height2;
            REGION3_HEIGHT = height3;
        }
        public void changeThreshold(int changeby){
            threshold = threshold+changeby;
        }



        public void setAnchorPoints(int x, int y) {
            x_anchor1 = x;
            y_anchor1 = y;
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x_anchor1, y_anchor1);
            region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);
            region2_pointA = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x,
                    REGION2_TOPLEFT_ANCHOR_POINT.y);
            region2_pointB = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH,
                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);
            region3_pointA = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x,
                    REGION3_TOPLEFT_ANCHOR_POINT.y);
            region3_pointB = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH,
                    REGION3_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);
        }

        public void setAnchorPoints2(int x1, int y1, int x2, int y2, int x3, int y3) {
            x_anchor1 = x1;
            y_anchor1 = y1;
            x_anchor2 = x2;
            y_anchor2 = y2;
            x_anchor3 = x3;
            y_anchor3 = y3;

            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x_anchor1, y_anchor1);
            REGION2_TOPLEFT_ANCHOR_POINT = new Point(x_anchor2, y_anchor2);
            REGION3_TOPLEFT_ANCHOR_POINT = new Point(x_anchor3, y_anchor3);

            region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

            region2_pointA = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x,
                    REGION2_TOPLEFT_ANCHOR_POINT.y);
            region2_pointB = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH,
                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);
            region3_pointA = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x,
                    REGION3_TOPLEFT_ANCHOR_POINT.y);
            region3_pointB = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH,
                    REGION3_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);
        }

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }
        public boolean isSimilarTo(int value1, int value2){
            if(abs(value1-value2) <= threshold){
                return true;
            }else{
                return false;
            }
        }
        @Override
        public Mat processFrame(Mat input)
        {
//            Mat gray = new Mat(input.rows(), input.cols(), input.type());
//            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
//            Mat binary = new Mat(input.rows(), input.cols(), input.type(), new Scalar(0));
//            Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY_INV);
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//            Scalar color = new Scalar(0, 0, 255);
//            Imgproc.drawContours(input, contours, -1, color, 2, Imgproc.LINE_8, hierarchy, 2, new Point());


            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            position = RingPosition.FOUR; // Record our analysis
//            if(avg1 > FOUR_RING_THRESHOLD){
//                position = RingPosition.FOUR;
//            }else if (avg1 > ONE_RING_THRESHOLD){
//                position = RingPosition.ONE;
//            }else{
//                position = RingPosition.NONE;
//            }

            if (!(isSimilarTo(avg1, avg3))) {
                position = RingPosition.FOUR;
            }
            else if((!isSimilarTo(avg1, avg2) && isSimilarTo(avg1, avg3))) {
                position = RingPosition.ONE;
            }
            else if(isSimilarTo(avg1, avg2)){
                position = RingPosition.NONE;
            }
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill


            return input;
        }

        public RingPosition getPosition(){return position;}
        public int getAnalysis()
        {
            return avg1;
        }
        public int getAnalysis2()
        {
            return avg2;
        }
        public int getAnalysis3()
        {
            return avg3;
        }
        public int getThreshold(){return threshold;}
    }

}