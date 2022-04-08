import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Competition Auto", group = "Competition")

public class BlueCompAuto extends LinearOpMode {

    BarcodePipeline.FreightFrenzyPipeline pipeline;
    OpenCvCamera webcam;

    //dynamic variables
    private int autoStart = 0;
    private int autoMode = 0;
    private int initialDelay = 0;



    //
    private double  currentHeading = 0,
            targetHeading = 0;


    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double wheelDiameter = 3.77953;
    private final double botRotationSpeed = 1.1; //to match rotation with driving
    private final double wheelMaxVelocity = 1;
    private final double maxSlideTicks = 384.5 * 2.8;
    private final double maxSlideVelocity = wheelMotorRPM * wheelMotorTicks / 60;
    private final double spinnyTicks = 537.7;
    private final double maxSpinnyVelocity = 312 * spinnyTicks / 60;

    private final double intakeDown = .5;
    private final double intakeUp = .3;
    private final double intakeDrop = .65;

    private final double dropdownDown = .55;
    private final double dropdownUp = .1;
    private final double dropdownHalf = .3;
    private final double dropdownDownLeft = .3;
    private final double dropdownUpLeft = .055;




    private boolean bumpersPressed = false;
    private int activeGear = 2;//values 1 - 3

    //inverse drive variables
    private boolean xPressed = false;
    private int inverse = 1;

    //create the gear array
    private final double[] gearValues = {.18, .3, .5, 1.0};

    private int slidePosition = 0;
    private final double[] slideValues = {
            maxSlideTicks * 1, //top level
            maxSlideTicks * .55, //shared
            maxSlideTicks * .3, //low level
            maxSlideTicks * .6, //middle level

    };
    private int barcodeValue = 0;
    private int firstLevel = (int) (maxSlideTicks * .05);
    private int secondLevel = (int) (maxSlideTicks * .5);

    private final double[] cameraslideValuesred = {
            firstLevel,
            secondLevel,
            maxSlideTicks
    };

    private final double[] cameraslideValuesblue = {
            secondLevel,
            firstLevel,
            maxSlideTicks
    };

    //create general variables
    private double gamepad1LY;
    private double gamepad1LX;
    private double gamepad1RX;

    //create motor/servo objects
//    private CRServo leftCarousel;
//    private CRServo rightCarousel;

    private DcMotor carousel;

    private Servo stick;

    private DcMotorEx rightSlide;
    private DcMotorEx leftSlide;

    private DcMotor tubeys;

    private Servo rightDropdown;
    private Servo leftDropdown;


    private ColorSensor color;

    public enum CarouselRedState {
        STARTING_POSITION,
        SHIPPING_COORDINATE,
        SHIPPING_LOCALIZATION,

    }


    public enum AutoIntakeState {
        DEFAULT_POSITION,
        INTAKE_ACTIVE,
        ACTIVE_INTAKE_NO_SENSOR,
        REVERSE_INTAKE,
        TOP_LEVEL,
        HORIZONTAL_ROTATION_RIGHT,
        HORIZONTAL_ROTATION_LEFT,
        HORIZONTAL_ROTATION_FORWARD,
        DROP_FREIGHT,
        SLIDE_DOWN,
        MIDDLE_LEVEL,
        SHARED
    }

    //AutoIntakeState autoState = AutoIntakeState.DEFAULT_POSITION;

    private ElapsedTime autoIntakeTime = new ElapsedTime();

    private final double down = 0;
    private final double forward = .8;




    private final double stickUp = .4;
    private final double stickDown = 0;
    private final double stickMiddle = .1;



    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);



        pipeline = new BarcodePipeline.FreightFrenzyPipeline();

        webcam.setPipeline(pipeline);

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
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        //initialize imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        //imu hardware map




        //create variable to store visual information
        String[] display = {"Carousel-Blue", "Warehouse-Blue", "Carousel-Red",
                "Warehouse-Red", "Deliver Freight", "Park"};
        String[] Barcode = {"Outside Detection", "Left Side", "Right Side"};
        //for loop to allow selection of autonomous situation
        for (boolean buttonPressed = false; !gamepad2.a && !opModeIsActive() && !isStopRequested();) {

            //change starting line
            if (gamepad2.dpad_left && autoStart > 0 && !buttonPressed) {
                autoStart--;
                buttonPressed = true;
            } else if (gamepad2.dpad_right && autoStart < 3 && !buttonPressed) {
                autoStart++;
                buttonPressed = true;
                //change automode
            }
            else if (gamepad2.dpad_down && initialDelay > 0 && !buttonPressed) {
                initialDelay--;
                buttonPressed = true;
            } else if (gamepad2.dpad_up && initialDelay < 25 && !buttonPressed) {
                initialDelay++;
                buttonPressed = true;
            }
            //choose which autonomous mode
            else if (gamepad2.left_bumper && autoMode > 0 && !buttonPressed) {
                autoMode--;
                buttonPressed = true;
            } else if (gamepad2.right_bumper && autoMode < 1 && !buttonPressed) {
                autoMode++;
                buttonPressed = true;
            }
            //choose barcode position
           /* else if (gamepad2.square && barcodeValue > 0 && !buttonPressed) {
                barcodeValue--;
                buttonPressed = true;
            } else if (gamepad2.circle && barcodeValue < 2 && !buttonPressed) {
                barcodeValue++;
                buttonPressed = true;
            }*/

            //wait until buttons are not pressed
            else if (buttonPressed && !gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.dpad_up &&
                    !gamepad2.dpad_down && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.square && !gamepad2.circle)
                buttonPressed = false;

            //output telemetry
            telemetry.addData("Auto Start (Right/Left Dpad)", display[autoStart]);
            telemetry.addData("Auto Mode (Bumpers)", display[autoMode + 4]);
            telemetry.addData("initialDelay (Up/Down Dpad)", initialDelay);
            telemetry.addData("Barcode Position", Barcode[barcodeValue]);

//            telemetry.addData("Analysis1", pipeline.avg1);
//            telemetry.addData("Analysis2", pipeline.avg2);

            if (BarcodePipeline.position == BarcodePipeline.BarcodePosition.LEFT)
                barcodeValue = 0;
            if (BarcodePipeline.position == BarcodePipeline.BarcodePosition.MIDDLE)
                barcodeValue = 1;
            if (BarcodePipeline.position == BarcodePipeline.BarcodePosition.RIGHT)
                barcodeValue = 2;


//            telemetry.addData("Position", BarcodePipeline.position);
            telemetry.addData("BarcodeValue", barcodeValue);






            telemetry.update();

        }

        //setup driving motors

        //setup other objects
//        leftCarousel = hardwareMap.get(CRServo.class, "leftCarousel");
//        rightCarousel = hardwareMap.get(CRServo.class, "rightCarousel");
        carousel = hardwareMap.get(DcMotor.class, "carousel");



        stick = hardwareMap.get(Servo.class, "stick");


        rightDropdown = hardwareMap.get(Servo.class, "rightDropdown");
        leftDropdown = hardwareMap.get(Servo.class, "leftDropdown");

        //servo direction
        rightDropdown.setDirection(Servo.Direction.REVERSE);
        leftDropdown.setDirection(Servo.Direction.FORWARD);




        //armMotor encoders
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setTargetPosition(0);
        rightSlide.setVelocity(0);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(0);
        leftSlide.setVelocity(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        color = hardwareMap.colorSensor.get("color");

        tubeys = hardwareMap.get(DcMotor.class, "tubeys");



        if (isStopRequested()) return;




        waitForStart();




        //intake Servo
        rightSlide.setVelocity(maxSlideVelocity);
        leftSlide.setVelocity(maxSlideVelocity);

  //TODO Warehouse Blue
         if (autoStart == 1 && autoMode == 0) { //Warehouse Blue, deliver freight
            sleep(initialDelay * 1000);
             drive.setPoseEstimate(new Pose2d(12, 62, Math.toRadians(90)));

             TrajectorySequence trajRWtest = drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(90)))

                     .lineToSplineHeading(new Pose2d(-8, 38, Math.toRadians(-125)))

                     .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                            rightSlide.setPower(.5);
                            leftSlide.setPower(.5);
                            rightSlide.setTargetPosition((int) cameraslideValuesblue[barcodeValue]);
                            leftSlide.setTargetPosition((int) cameraslideValuesblue[barcodeValue]);
                            rightDropdown.setPosition(forward);
                            leftDropdown.setPosition(forward);
                            stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                         stick.setPosition(stickUp);})
                     .waitSeconds(.3)

                     .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                          stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                            stick.setPosition(stickDown);
                            rightSlide.setVelocity(maxSlideVelocity * .5);
                            leftSlide.setVelocity(maxSlideVelocity * .5);
                            leftDropdown.setPosition(down + .05);
                            rightDropdown.setPosition(down + .05);
                            rightSlide.setTargetPosition(0);
                            leftSlide.setTargetPosition(0);})
                     .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            tubeys.setPower(.82);
                            stick.setPosition(stickUp);
                            leftDropdown.setPosition(down);
                            rightDropdown.setPosition(down);})
                     .lineToSplineHeading(new Pose2d(5, 62, Math.toRadians(180)))
                     .UNSTABLE_addTemporalMarkerOffset(1.75, () -> {
                          stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                          tubeys.setPower(-.5);;})
                     .UNSTABLE_addTemporalMarkerOffset(2.25, () -> {
                          tubeys.setPower(0);;})
                     .splineToConstantHeading(new Vector2d(48, 68), Math.toRadians(0))
                     .lineToSplineHeading(new Pose2d(22, 72, Math.toRadians(-180)))
                     .UNSTABLE_addTemporalMarkerOffset(-.8, () -> {
                            rightSlide.setPower(.5);
                            leftSlide.setPower(.5);
                            rightSlide.setTargetPosition((int) maxSlideTicks);
                            leftSlide.setTargetPosition((int) maxSlideTicks);
                            rightDropdown.setPosition(forward);
                            leftDropdown.setPosition(forward);
                            stick.setPosition(stickDown);})
                     .splineToSplineHeading(new Pose2d(-5, 46, Math.toRadians(-110)), Math.toRadians(-90))
                     .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {stick.setPosition(stickUp);})
                     .waitSeconds(.1)

                     .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                         stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                            stick.setPosition(stickDown);
                            rightSlide.setVelocity(maxSlideVelocity * .5);
                            leftSlide.setVelocity(maxSlideVelocity * .5);
                            leftDropdown.setPosition(down + .05);
                            rightDropdown.setPosition(down + .05);
                            rightSlide.setTargetPosition(0);
                            leftSlide.setTargetPosition(0);})
                     .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                            tubeys.setPower(.82);
                            stick.setPosition(stickUp);
                            leftDropdown.setPosition(down);
                            rightDropdown.setPosition(down);})
                     .lineToSplineHeading(new Pose2d(0, 62, Math.toRadians(-180)))
                     .UNSTABLE_addTemporalMarkerOffset(1.75, () -> {
                         stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                         tubeys.setPower(-.5);;})
                     .UNSTABLE_addTemporalMarkerOffset(2.25, () -> {
                         tubeys.setPower(0);;})
                     .splineToConstantHeading(new Vector2d(50, 68), Math.toRadians(0))
                     .lineToSplineHeading(new Pose2d(12, 72, Math.toRadians(-180)))
                     .UNSTABLE_addTemporalMarkerOffset(-.8, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);})
                     .splineToSplineHeading(new Pose2d(-5, 46, Math.toRadians(-110)), Math.toRadians(-90))
                     .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {stick.setPosition(stickUp);})
                     .waitSeconds(.1)

                     .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                         stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                            stick.setPosition(stickDown);
                            rightSlide.setVelocity(maxSlideVelocity * .5);
                            leftSlide.setVelocity(maxSlideVelocity * .5);
                            leftDropdown.setPosition(down + .05);
                            rightDropdown.setPosition(down + .05);
                            rightSlide.setTargetPosition(0);
                            leftSlide.setTargetPosition(0);})
                     .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                            tubeys.setPower(.82);
                            stick.setPosition(stickUp);
                            leftDropdown.setPosition(down);
                            rightDropdown.setPosition(down);})
                     .lineToSplineHeading(new Pose2d(0, 62, Math.toRadians(-180)))
                     .UNSTABLE_addTemporalMarkerOffset(1.75, () -> {
                         stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                         tubeys.setPower(-.5);;})
                     .UNSTABLE_addTemporalMarkerOffset(2.25, () -> {
                         tubeys.setPower(0);;})
                     .splineToConstantHeading(new Vector2d(58, 68), Math.toRadians(0))
                     .lineToSplineHeading(new Pose2d(12, 76, Math.toRadians(-180)))
                     .UNSTABLE_addTemporalMarkerOffset(-.8, () -> {
                            rightSlide.setPower(.5);
                            leftSlide.setPower(.5);
                            rightSlide.setTargetPosition((int) maxSlideTicks);
                            leftSlide.setTargetPosition((int) maxSlideTicks);
                            rightDropdown.setPosition(forward);
                            leftDropdown.setPosition(forward);
                            stick.setPosition(stickDown);})
                     .splineToSplineHeading(new Pose2d(0, 50, Math.toRadians(-110)), Math.toRadians(-90))
                     .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {stick.setPosition(stickUp);})
                     .waitSeconds(.1)

                     .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                          stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                            stick.setPosition(stickDown);
                            rightSlide.setVelocity(maxSlideVelocity * .5);
                            leftSlide.setVelocity(maxSlideVelocity * .5);
                            leftDropdown.setPosition(down + .05);
                            rightDropdown.setPosition(down + .05);
                            rightSlide.setTargetPosition(0);
                            leftSlide.setTargetPosition(0);})
                     .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                            tubeys.setPower(.82);
                            stick.setPosition(stickUp);
                            leftDropdown.setPosition(down);
                            rightDropdown.setPosition(down);})


                     .lineToSplineHeading(new Pose2d(0, 62, Math.toRadians(-180)))
                     .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                         stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                         tubeys.setPower(0);;})
                     .splineToConstantHeading(new Vector2d(91, 68), Math.toRadians(0))

                     .build();

             drive.followTrajectorySequence(trajRWtest);


        } else if (autoStart == 1 && autoMode == 1) { //Warehouse Blue park

            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(12, 62, Math.toRadians(270)));
            //drive.followTrajectorySequence(warehouseBluePark);
        }

      //TODO Carousel Red
        else if (autoStart == 2 && autoMode == 0) { //Carousel Red, deliver freight top
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-35, -62, Math.toRadians(270)));
            // drive.followTrajectorySequence(carouselRedTop);
            TrajectorySequence CRtop = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(270)))

                    .lineToSplineHeading(new Pose2d(-55, -40, Math.toRadians(360)))
                    .lineToConstantHeading(new Vector2d(-55, -20))

                    //drop freight
                    .lineToConstantHeading(new Vector2d(-33, -25))
                    .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) cameraslideValuesred[barcodeValue]);
                        leftSlide.setTargetPosition((int) cameraslideValuesred[barcodeValue]);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);})
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        stick.setPosition(stickUp);})
                    .waitSeconds(.3)

                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        stick.setPosition(stickDown);})
                    .UNSTABLE_addTemporalMarkerOffset(.4, () -> {
                        stick.setPosition(stickDown);
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);
                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);
                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0)
                        ;})
                    .lineToSplineHeading(new Pose2d(-62, -30, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(-60, -60, Math.toRadians(90)))
                    .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
                        carousel.setPower(-.35);})
                    .waitSeconds(3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        carousel.setPower(0);})

                    .lineToConstantHeading(new Vector2d(-59, -66))
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                        tubeys.setPower(.82);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);})

                    .lineToConstantHeading(new Vector2d(-40, -62))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        stick.setPosition(stickDown);})
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        tubeys.setPower(0);})
                    .lineToSplineHeading(new Pose2d(-55, -40, Math.toRadians(360)))
                    .lineToConstantHeading(new Vector2d(-55, -20))
                    //drop duck
                    .lineToConstantHeading(new Vector2d(-33, -25))
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);})
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        stick.setPosition(stickUp);})
                    .waitSeconds(.3)

                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        stick.setPosition(stickDown);})
                    .UNSTABLE_addTemporalMarkerOffset(.4, () -> {
                        stick.setPosition(stickDown);
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);
                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);
                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);})
                    .lineToSplineHeading(new Pose2d(-62, -30, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(-64, -40, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(CRtop);

        }  //TODO Carousel Blue
        else if (autoStart == 0 && autoMode == 0) { //Carousel Red, deliver freight top, noncompleted
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-35, 62, Math.toRadians(-270)));
            // drive.followTrajectorySequence(carouselRedTop);
            TrajectorySequence CRtop = drive.trajectorySequenceBuilder(new Pose2d(-35, 62, Math.toRadians(-270)))

                    .lineToSplineHeading(new Pose2d(-55, 40, Math.toRadians(-360)))
                    .lineToConstantHeading(new Vector2d(-55, 20))

                    //drop freight
                    .lineToConstantHeading(new Vector2d(-33, 25))
                    .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) cameraslideValuesblue[barcodeValue]);
                        leftSlide.setTargetPosition((int) cameraslideValuesblue[barcodeValue]);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);})
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        stick.setPosition(stickUp);})
                    .waitSeconds(.3)

                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        stick.setPosition(stickDown);})
                    .UNSTABLE_addTemporalMarkerOffset(.4, () -> {
                        stick.setPosition(stickDown);
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);
                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);
                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0)
                        ;})
                    .lineToSplineHeading(new Pose2d(-62, 30, Math.toRadians(-90)))
                    .lineToSplineHeading(new Pose2d(-62, 60, Math.toRadians(-90)))
                    .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
                        carousel.setPower(.35);})
                    .waitSeconds(3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        carousel.setPower(0);})

                    .lineToConstantHeading(new Vector2d(-59, 66))
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                        tubeys.setPower(.82);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);})

                    .lineToConstantHeading(new Vector2d(-40, 62))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        stick.setPosition(stickDown);})
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        tubeys.setPower(0);})
                    .lineToSplineHeading(new Pose2d(-55, 40, Math.toRadians(-360)))
                    .lineToConstantHeading(new Vector2d(-55, 20))
                    //drop duck
                    .lineToConstantHeading(new Vector2d(-33, 25))
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);})
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        stick.setPosition(stickUp);})
                    .waitSeconds(.3)

                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        stick.setPosition(stickDown);})
                    .UNSTABLE_addTemporalMarkerOffset(.4, () -> {
                        stick.setPosition(stickDown);
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);
                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);
                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);})
                    .lineToSplineHeading(new Pose2d(-62, 30, Math.toRadians(-90)))
                    .lineToSplineHeading(new Pose2d(-64, 40, Math.toRadians(-90)))
                    .build();
            drive.followTrajectorySequence(CRtop);

        } else if (autoStart == 2 && autoMode == 1) { //Carousel Red park

        }

        //TODO Warehouse Red
        else if (autoStart == 3 && autoMode == 0) { //Warehouse Red, deliver freight
            //drive.followTrajectorySequence(WarehouseRedTop);
            drive.setPoseEstimate(new Pose2d(12, -62, Math.toRadians(270)));

             TrajectorySequence trajRWtest = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(270)))

                     .lineToSplineHeading(new Pose2d(-8, -38, Math.toRadians(125)))

                     .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                            rightSlide.setPower(.5);
                            leftSlide.setPower(.5);
                            rightSlide.setTargetPosition((int) cameraslideValuesred[barcodeValue]);
                            leftSlide.setTargetPosition((int) cameraslideValuesred[barcodeValue]);
                            rightDropdown.setPosition(forward);
                            leftDropdown.setPosition(forward);
                            stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                         stick.setPosition(stickUp);})
                     .waitSeconds(.3)

                     .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                         stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                         stick.setPosition(stickDown);
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);
                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);
                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);})
                     .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        tubeys.setPower(.82);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);})
                     .lineToSplineHeading(new Pose2d(5, -62, Math.toRadians(180)))
                     .UNSTABLE_addTemporalMarkerOffset(1.75, () -> {
                         stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                          tubeys.setPower(-.5);;})
                     .UNSTABLE_addTemporalMarkerOffset(2.25, () -> {
                         tubeys.setPower(0);;})
                     .splineToConstantHeading(new Vector2d(48, -68), Math.toRadians(0))
                     .lineToSplineHeading(new Pose2d(22, -72, Math.toRadians(180)))
                     .UNSTABLE_addTemporalMarkerOffset(-.8, () -> {
                            rightSlide.setPower(.5);
                            leftSlide.setPower(.5);
                            rightSlide.setTargetPosition((int) maxSlideTicks);
                            leftSlide.setTargetPosition((int) maxSlideTicks);
                            rightDropdown.setPosition(forward);
                            leftDropdown.setPosition(forward);
                            stick.setPosition(stickDown);})
                     .splineToSplineHeading(new Pose2d(-5, -46, Math.toRadians(110)), Math.toRadians(90))
                     .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {stick.setPosition(stickUp);})
                     .waitSeconds(.1)

                     .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                         stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                            stick.setPosition(stickDown);
                            rightSlide.setVelocity(maxSlideVelocity * .5);
                            leftSlide.setVelocity(maxSlideVelocity * .5);
                            leftDropdown.setPosition(down + .05);
                            rightDropdown.setPosition(down + .05);
                            rightSlide.setTargetPosition(0);
                            leftSlide.setTargetPosition(0);})
                     .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                            tubeys.setPower(.82);
                            stick.setPosition(stickUp);
                            leftDropdown.setPosition(down);
                            rightDropdown.setPosition(down);})
                     .lineToSplineHeading(new Pose2d(0, -62, Math.toRadians(180)))
                     .UNSTABLE_addTemporalMarkerOffset(1.75, () -> {
                         stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                         tubeys.setPower(-.5);;})
                     .UNSTABLE_addTemporalMarkerOffset(2.25, () -> {
                         tubeys.setPower(0);;})
                     .splineToConstantHeading(new Vector2d(50, -68), Math.toRadians(0))
                     .lineToSplineHeading(new Pose2d(12, -72, Math.toRadians(180)))
                     .UNSTABLE_addTemporalMarkerOffset(-.8, () -> {
                            rightSlide.setPower(.5);
                            leftSlide.setPower(.5);
                            rightSlide.setTargetPosition((int) maxSlideTicks);
                            leftSlide.setTargetPosition((int) maxSlideTicks);
                            rightDropdown.setPosition(forward);
                            leftDropdown.setPosition(forward);
                            stick.setPosition(stickDown);})
                     .splineToSplineHeading(new Pose2d(-5, -46, Math.toRadians(110)), Math.toRadians(90))
                     .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {stick.setPosition(stickUp);})
                     .waitSeconds(.1)

                     .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                         stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                            stick.setPosition(stickDown);
                            rightSlide.setVelocity(maxSlideVelocity * .5);
                            leftSlide.setVelocity(maxSlideVelocity * .5);
                            leftDropdown.setPosition(down + .05);
                            rightDropdown.setPosition(down + .05);
                            rightSlide.setTargetPosition(0);
                            leftSlide.setTargetPosition(0);})
                     .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                            tubeys.setPower(.82);
                            stick.setPosition(stickUp);
                            leftDropdown.setPosition(down);
                            rightDropdown.setPosition(down);})
                     .lineToSplineHeading(new Pose2d(0, -62, Math.toRadians(180)))
                     .UNSTABLE_addTemporalMarkerOffset(1.75, () -> {
                             stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                         tubeys.setPower(-.5);;})
                     .UNSTABLE_addTemporalMarkerOffset(2.25, () -> {
                         tubeys.setPower(0);;})
                     .splineToConstantHeading(new Vector2d(58, -68), Math.toRadians(0))
                     .lineToSplineHeading(new Pose2d(12, -76, Math.toRadians(180)))
                     .UNSTABLE_addTemporalMarkerOffset(-.8, () -> {
                            rightSlide.setPower(.5);
                            leftSlide.setPower(.5);
                            rightSlide.setTargetPosition((int) maxSlideTicks);
                            leftSlide.setTargetPosition((int) maxSlideTicks);
                            rightDropdown.setPosition(forward);
                            leftDropdown.setPosition(forward);
                            stick.setPosition(stickDown);})
                     .splineToSplineHeading(new Pose2d(0, -50, Math.toRadians(110)), Math.toRadians(90))
                     .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {stick.setPosition(stickUp);})
                     .waitSeconds(.1)

                     .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                         stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                         stick.setPosition(stickDown);
                         rightSlide.setVelocity(maxSlideVelocity * .5);
                         leftSlide.setVelocity(maxSlideVelocity * .5);
                         leftDropdown.setPosition(down + .05);
                         rightDropdown.setPosition(down + .05);
                         rightSlide.setTargetPosition(0);
                         leftSlide.setTargetPosition(0);})
                     .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                         tubeys.setPower(.82);
                         stick.setPosition(stickUp);
                         leftDropdown.setPosition(down);
                         rightDropdown.setPosition(down);})


                     .lineToSplineHeading(new Pose2d(0, -62, Math.toRadians(180)))
                     .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                         stick.setPosition(stickDown);})
                     .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                         tubeys.setPower(0);;})
                     .splineToConstantHeading(new Vector2d(91, -68), Math.toRadians(0))

                     .build();

             drive.followTrajectorySequence(trajRWtest);


        }

    else if (autoStart == 3 && autoMode == 1) { //Warehouse Red park

        }



    }



}

