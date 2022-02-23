package Archive;/*

package org.firstinspires.ftc.teamcode.Archive;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import static java.lang.Math.abs;

@Disabled
@Autonomous (name = "OpenCV")
public class reliableOpenCV extends LinearOpMode
{
    //create webcam and pipeline
    OpenCvCamera webcam;
    UltimateGoalPipeline pipeline;

    //editable variables
    private int zone = 0;
    private boolean intakeRings = false;
    private int initialDelay = 0;
    public static int startingLine = 0;

    //camera anchors
    private static final int[] xAnchor1 = {108, 10, 108, 10}; //blueLeft, blueRight, redLeft, redRight
    private static final int[] yAnchor1 = {250, 250, 250, 250};
    //
    private  static final int[] xAnchor2 = {108, 10, 108, 10}; //raising values moves it to the right
    private static final int[] yAnchor2 = {210, 200, 200, 200}; //raising value moves it down
    //
    private static final int[] xAnchor3 = {108, 10, 108, 10};
    private static final int[] yAnchor3 = {180, 170, 165, 170};
    //width and height for boxes
    private static final int[] region1Width = {90, 90, 90, 90};
    private static final int[] region1Height = {50, 50, 50, 50};
    //
    private static final int[] region2Width = {90, 90, 90, 90};
    private static final int[] region2Height = {20, 20, 20, 20};
    //
    private static final int[] region3Width = {90, 90, 90, 90};
    private static final int[] region3Height = {25, 25, 25, 25};







    @Override
    public void runOpMode()

    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        pipeline = new UltimateGoalPipeline();
        webcam.setPipeline(pipeline);



        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //webcam.startStreaming(320, 240);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

            }

            @Override
            public void onError(int errorCode) {

            }
        });


        while (!opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Analysis1", pipeline.getAnalysis());
            telemetry.addData("Analysis2", pipeline.getAnalysis2());
            telemetry.addData("Analysis3", pipeline.getAnalysis3());
            telemetry.addData("Position", pipeline.position);

            telemetry.addLine("Waiting for start");
            telemetry.update();
        }



        waitForStart();





        if(pipeline.position == UltimateGoalPipeline.RingPosition.FOUR)
            zone = 2;
        else if (pipeline.position == UltimateGoalPipeline.RingPosition.ONE)
            zone = 1;
        else if (pipeline.position == UltimateGoalPipeline.RingPosition.NONE)
            zone = 0;


    }





    //ring detector pipeline
    public static class UltimateGoalPipeline extends OpenCvPipeline
    {

        //An enum to define the ring position
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }


        //Some color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        */
/*
         * The core values which define the location and size of the sample regions
         *//*



        //grey tile
        public static int x_anchor1 = xAnchor1[startingLine];
        public static int y_anchor1 = yAnchor1[startingLine];
        //bottom ring
        public static int x_anchor2 = xAnchor2[startingLine];
        public static int y_anchor2 = yAnchor2[startingLine];
        //top rings
        public static int x_anchor3 = xAnchor3[startingLine];;
        public static int y_anchor3 = yAnchor3[startingLine];


        static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(x_anchor1,y_anchor1);
        static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(x_anchor2,y_anchor2);
        static Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(x_anchor3,y_anchor3);


        public static  int REGION1_WIDTH = region1Width[startingLine];
        public static  int REGION1_HEIGHT = region1Height[startingLine];

        public static  int REGION2_WIDTH = region2Width[startingLine];
        public static  int REGION2_HEIGHT = region2Height[startingLine];

        public static  int REGION3_WIDTH = region3Width[startingLine];
        public static  int REGION3_HEIGHT = region3Height[startingLine];



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
        */
/*
         * Working variables
         *//*

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

        */
/*public void setHeightAndWidth(int height1, int height2, int width1, int width2, int height3, int width3) {
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
        }*//*


        */
/*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         *//*

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

            inputToCb(input);

            //grey tile
            avg1 = (int) Core.mean(region1_Cb).val[0];
            //bottom ring
            avg2 = (int) Core.mean(region2_Cb).val[0];
            //top 3 rings
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
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            position = RingPosition.FOUR; // Record our analysis
//

            if (!(isSimilarTo(avg1, avg3))) { // abs(tileAvg - topAvg) >= 7
                position = RingPosition.FOUR;
            }
            else if((!isSimilarTo(avg1, avg2) && isSimilarTo(avg1, avg3))) { // abs(tileAvg - bottomAvg) >= 7
                position = RingPosition.ONE;
            }
            else if(isSimilarTo(avg1, avg2)){ // abs(tileAvg - bottomAvg) <= 7
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
    }

}*/
