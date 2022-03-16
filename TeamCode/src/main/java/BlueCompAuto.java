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
    private int barcodeValue = 0;




    //
    private double  currentHeading = 0,
            targetHeading = 0;


    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double wheelDiameter = 3.77953;
    private final double botRotationSpeed = 1.1; //to match rotation with driving
    private final double wheelMaxVelocity = 1;
    private final double maxrightSlideTicks = 384.5 * 2;
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
            maxrightSlideTicks * 1, //top level
            maxrightSlideTicks * .55, //shared
            maxrightSlideTicks * .3, //low level
            maxrightSlideTicks * .6, //middle level

    };




    //create general variables
    private double gamepad1LY;
    private double gamepad1LX;
    private double gamepad1RX;

    //create motor/servo objects



    private CRServo leftCarousel;
    private CRServo rightCarousel;

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

    private final double maxSlideTicks = 384.5 * 2.8;


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
        String[] Barcode = {"Right", "Left", "Outside"};
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
            }  /*else if (gamepad1.dpad_down && autoMode > 0 && !buttonPressed) {
                autoMode--;
                buttonPressed = true;
            } else if (gamepad1.dpad_up && autoMode < 1 && !buttonPressed) {
                autoMode++;
                buttonPressed = true;
            }*/
            //change initial delay
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
            else if (gamepad2.square && barcodeValue > 0 && !buttonPressed) {
                barcodeValue--;
                buttonPressed = true;
            } else if (gamepad2.circle && barcodeValue < 2 && !buttonPressed) {
                barcodeValue++;
                buttonPressed = true;
            }

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
            telemetry.addData("Position", BarcodePipeline.position);
            telemetry.addData("BarcodeValue", barcodeValue);

            //robot position notes
//            if (autoStart == 0 && autoMode == 0) //CAROUSEL BLUE SPIN AND PARK
//                telemetry.addLine("Spinny - Bottom Right");
//            else if (autoStart == 1 && autoMode == 0) //WAREHOUSE BLUE PARK
//                telemetry.addLine("Spinny - Bottom Left");
//            else if (autoStart == 2 && autoMode == 0) //CAROUSEL RED SPIN AND PARK
//                telemetry.addLine("Spinny - Bottom Left");
//            else if (autoStart == 3 && autoMode == 0) //WAREHOUSE RED PARK
//                telemetry.addLine("Spinny - Top Right");






            telemetry.update();

        }

        //setup driving motors

        //setup other objects
        leftCarousel = hardwareMap.get(CRServo.class, "leftCarousel");
        rightCarousel = hardwareMap.get(CRServo.class, "rightCarousel");

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

        if (BarcodePipeline.position == BarcodePipeline.BarcodePosition.LEFT)
            barcodeValue = 0;
        if (BarcodePipeline.position == BarcodePipeline.BarcodePosition.MIDDLE)
            barcodeValue = 1;
        if (BarcodePipeline.position == BarcodePipeline.BarcodePosition.RIGHT)
            barcodeValue = 2;



        //intake Servo
        rightSlide.setVelocity(maxSlideVelocity);
        leftSlide.setVelocity(maxSlideVelocity);
/*
        switch (autoState) {
            case DEFAULT_POSITION:

                rightSlide.setVelocity(maxSlideVelocity);
                leftSlide.setVelocity(maxSlideVelocity);

                //TODO
                leftDropdown.setPosition(down + .05);
                rightDropdown.setPosition(down + .05);

                rightSlide.setTargetPosition(0);
                leftSlide.setTargetPosition(0);
                tubeys.setPower(0);





                break;




            case TOP_LEVEL:

                rightDropdown.setPosition(down + .2);
                leftDropdown.setPosition(down + .2);


                if (autoIntakeTime.seconds() > 0) {
                    rightSlide.setPower(.5);
                    leftSlide.setPower(.5);
                    rightSlide.setTargetPosition((int) maxSlideTicks);
                    leftSlide.setTargetPosition((int) maxSlideTicks);
                    rightDropdown.setPosition(forward);
                    leftDropdown.setPosition(forward);

                } if (autoIntakeTime.seconds() > .1) {


            }

                break;

            case MIDDLE_LEVEL:

                rightDropdown.setPosition(down + .2);
                leftDropdown.setPosition(down + .2);

                if (autoIntakeTime.seconds() > 0) {
                    rightSlide.setPower(.5);
                    leftSlide.setPower(.5);
                    rightSlide.setTargetPosition((int) (maxSlideTicks * .55));
                    leftSlide.setTargetPosition((int) (maxSlideTicks * .55));
                    rightDropdown.setPosition(forward);
                    leftDropdown.setPosition(forward);

                }
                break;

            case SHARED:

                rightDropdown.setPosition(down + .2);
                leftDropdown.setPosition(down + .2);


                if (autoIntakeTime.seconds() > 0) {
                    rightSlide.setPower(.5);
                    leftSlide.setPower(.5);
                    rightSlide.setTargetPosition((int) 0);
                    leftSlide.setTargetPosition((int) 0);
                    rightDropdown.setPosition(forward);
                    leftDropdown.setPosition(forward);

                }

                break;


            case DROP_FREIGHT:


                if (autoIntakeTime.seconds() > 0 && autoIntakeTime.seconds() < .3) {
                    stick.setPosition(stickUp);

                }
                else if (autoIntakeTime.seconds() > .3) {
                    autoIntakeTime.reset();
                    autoState = AutoIntakeState.SLIDE_DOWN;




                }
                break;

            case SLIDE_DOWN:
                rightSlide.setVelocity(maxSlideVelocity * .5);
                leftSlide.setVelocity(maxSlideVelocity * .5);

                leftDropdown.setPosition(down + .05);
                rightDropdown.setPosition(down + .05);

//                    leftDropdown.setPosition(dropdownDown);
//                    rightDropdown.setPosition(dropdownDown);
                rightSlide.setTargetPosition(0);
                leftSlide.setTargetPosition(0);


                *//*slideRotation.setPosition(middle);*//*
                if (autoIntakeTime.seconds() > .5)
                    autoState = AutoIntakeState.DEFAULT_POSITION;
                break;
            case INTAKE_ACTIVE:

                tubeys.setPower(.75);
                stick.setPosition(stickUp);
                leftDropdown.setPosition(down);
                rightDropdown.setPosition(down);

                *//*    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);*//*

                autoIntakeTime.reset();

                if (!gamepad2.right_bumper && autoIntakeTime.seconds() < .75) {


//                        leftDropdown.setPosition(dropdownDown);
//                        rightDropdown.setPosition(dropdownDown);
                    stick.setPosition(stickDown);

                    autoState = AutoIntakeState.DEFAULT_POSITION;
                } else if (((DistanceSensor) color).getDistance(DistanceUnit.CM) < 3 && autoIntakeTime.seconds() < .5) {
                    gamepad1.rumbleBlips(1);
                    gamepad2.rumbleBlips(1);

                    stick.setPosition(stickDown);

//                        leftDropdown.setPosition(dropdownUp);
//                        rightDropdown.setPosition(dropdownUp);

                    autoState = AutoIntakeState.DEFAULT_POSITION;
                }
                break;
            case ACTIVE_INTAKE_NO_SENSOR:

//                    leftDropdown.setPosition(dropdownDown);
//                    rightDropdown.setPosition(dropdownDown);
                autoIntakeTime.reset();
                if (gamepad2.right_trigger == 0 && autoIntakeTime.seconds() < .75) {
                    tubeys.setPower(.75);
                    stick.setPosition(stickUp);
                    leftDropdown.setPosition(down);
                    rightDropdown.setPosition(down);

//                        leftDropdown.setPosition(dropdownDown);
//                        rightDropdown.setPosition(dropdownDown);

                    autoState = AutoIntakeState.DEFAULT_POSITION;
                }
                break;
            case REVERSE_INTAKE:
                tubeys.setPower(-.5);
                stick.setPosition(stickUp);

//                    leftDropdown.setPosition(dropdownDown);
//                    rightDropdown.setPosition(dropdownDown);
                autoIntakeTime.reset();
                if (!gamepad2.touchpad && autoIntakeTime.seconds() < .5) {
                    tubeys.setPower(-.5);

//                        leftDropdown.setPosition(dropdownUp);
//                        rightDropdown.setPosition(dropdownUp);

                    autoState = AutoIntakeState.DEFAULT_POSITION;
                }
                break;

        }*/


       /* TrajectorySequence warehouseRedPark = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))

                .strafeRight(30)

                .build();

        TrajectorySequence warehouseBluePark = drive.trajectorySequenceBuilder(new Pose2d(-35, 62, Math.toRadians(270)))

                .strafeLeft(30)

                .build();

        TrajectorySequence carouselRedTop = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))

                .lineToSplineHeading(new Pose2d(-53, -50, Math.toRadians(250)))
                .forward(5)
                .waitSeconds(3)
                .lineToSplineHeading(new Pose2d(-60, -34, Math.toRadians(180)))



                .build();

        TrajectorySequence carouselBlueTop = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-25, -34, Math.toRadians(45)), Math.toRadians(90))
                .waitSeconds(3) //wait for freight dropoffaaa
                .lineToSplineHeading(new Pose2d(-53, -55, Math.toRadians(250)))
                .waitSeconds(3)
                .lineToSplineHeading(new Pose2d(-60, -34, Math.toRadians(180)))


                .build();

        TrajectorySequence WarehouseRedTop = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))

                .lineToSplineHeading(new Pose2d(0, -32, Math.toRadians(120)))


                .build();

        TrajectorySequence WarehouseBlueTop = drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(10, 34, Math.toRadians(210)))
                .forward(8)
                .waitSeconds(.5) //wait for freight dropoffaaa
                .lineToSplineHeading(new Pose2d(12, 65, Math.toRadians(180)))

                .addDisplacementMarker(() -> {
                    drive.setPoseEstimate( new Pose2d(12, 62, Math.toRadians(180)));

                })

                .back(40)


                .addTemporalMarker(.5, () -> {
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                })





                .build();
*/










        //TODO Warehouse Blue Middle
       if (autoStart == 1 && autoMode == 0 && barcodeValue == 1) { //Warehouse Blue, deliver freight
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(12, 62, Math.toRadians(90)));

            TrajectorySequence traj2BWM = drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(90)))


                    .lineToSplineHeading(new Pose2d(-5, 42, Math.toRadians(250)))
                    .waitSeconds(1)
//                                .splineToSplineHeading(new Pose2d(10, 70, Math.toRadians(180)), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(10, 65, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(50, 65), Math.toRadians(0))

                    .addTemporalMarker(.1, () -> {

                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })

                    .addTemporalMarker(3.8, () -> {
                        stick.setPosition(stickUp);
                    })
                    .addTemporalMarker(4.5, () -> {
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);

                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);
                    })

                    .build();




            drive.followTrajectorySequence(traj2BWM);

        }  //TODO Warehouse Blue Top
        else if (autoStart == 1 && autoMode == 0 && barcodeValue == 2) { //Warehouse Blue, deliver freight
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(12, 62, Math.toRadians(-270)));

            Trajectory traj1RW = drive.trajectoryBuilder(new Pose2d(12, 62, Math.toRadians(-270)))
                    //sleep(initialDelay * 1000);
                    .lineToSplineHeading(new Pose2d(-5, 45, Math.toRadians(-110)))
                    /* .splineToSplineHeading(new Pose2d(10, -70, Math.toRadians(180)), Math.toRadians(-90))
                     .splineToConstantHeading(new Vector2d(50, -70), Math.toRadians(0))*/

                    .addTemporalMarker(.1, () -> {

                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })

                    .build();

            TrajectorySequence traj2RW = drive.trajectorySequenceBuilder(traj1RW.end())

                    .splineToSplineHeading(new Pose2d(10, 70, Math.toRadians(-180)), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(50, 70), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(15, 70, Math.toRadians(-180)))

//                                .splineToConstantHeading(new Vector2d(0, -70), Math.toRadians(0))
//                                .lineToSplineHeading(new Pose2d(5, -32, Math.toRadians(150)))
//                                .lineToSplineHeading(new Pose2d(5, -32, Math.toRadians(150)))
                    .splineToSplineHeading(new Pose2d(-5, 46, Math.toRadians(-110)), Math.toRadians(-90))
                    .waitSeconds(.5)
//                                .splineToSplineHeading(new Pose2d(15, -70, Math.toRadians(180)), Math.toRadians(-90))
                    .lineToSplineHeading(new Pose2d(15, 72, Math.toRadians(-180)))

                    .splineToConstantHeading(new Vector2d(50, 72), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(18, 72, Math.toRadians(-180)))

                    .splineToSplineHeading(new Pose2d(-5, 47, Math.toRadians(-110)), Math.toRadians(-90))
                    .waitSeconds(.5)
//                                .splineToSplineHeading(new Pose2d(15, -70, Math.toRadians(180)), Math.toRadians(-90))
                    .lineToSplineHeading(new Pose2d(15, 74, Math.toRadians(-180)))

                    .splineToConstantHeading(new Vector2d(50, 74), Math.toRadians(0))





                    .addTemporalMarker(0, () -> {

                        stick.setPosition(stickUp);



                    })


                    .addTemporalMarker(.5, () -> {

                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);



                    })

                    .addTemporalMarker(3, () -> {

                        tubeys.setPower(.75);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);




                    })


                    .addTemporalMarker(5.36, () -> {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

                    })
                    .addTemporalMarker(6, () -> {
                        tubeys.setPower(0);

                    })

                    .addTemporalMarker(6.5, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })
                    .addTemporalMarker(8, () -> {

                        stick.setPosition(stickUp);



                    })
//
//
                    .addTemporalMarker(8.3, () -> {

                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);



                    })

                    .addTemporalMarker(10.8, () -> {

                        tubeys.setPower(.75);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);




                    })


                    .addTemporalMarker(12.66, () -> {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

                    })
                    .addTemporalMarker(13.8, () -> {
                        tubeys.setPower(0);

                    })

                    .addTemporalMarker(14.3, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })
                    .addTemporalMarker(15.8, () -> {

                        stick.setPosition(stickUp);



                    })
//
//
                    .addTemporalMarker(16.1, () -> {

                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);


                    })

                    .addTemporalMarker(18.6, () -> {

                        tubeys.setPower(.75);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);




                    })


                    .addTemporalMarker(21.46, () -> {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

                    })

                    .build();

            drive.followTrajectory(traj1RW);
            drive.followTrajectorySequence(traj2RW);
            /*drive.setPoseEstimate(new Pose2d(12, 62, Math.toRadians(90)));
            TrajectorySequence traj2BWT = drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(90)))


                    .lineToSplineHeading(new Pose2d(-5, 42, Math.toRadians(250)))
                    .waitSeconds(1)
//                                .splineToSplineHeading(new Pose2d(10, 70, Math.toRadians(180)), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(10, 65, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(50, 65), Math.toRadians(0))

                    .addTemporalMarker(.1, () -> {

                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })

                    .addTemporalMarker(3.8, () -> {
                        stick.setPosition(stickUp);
                    })
                    .addTemporalMarker(4.5, () -> {
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);

                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);
                    })

                    .build();*/

            //drive.followTrajectorySequence(traj2BWT);

        } //TODO Warehouse Blue Bottom
        else if (autoStart == 1 && autoMode == 0 && barcodeValue == 0) { //Warehouse Blue, deliver freight
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(12, 62, Math.toRadians(90)));
            TrajectorySequence traj2BWB = drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(90)))


                    .lineToSplineHeading(new Pose2d(-5, 42, Math.toRadians(250)))
                    .waitSeconds(1)
//                                .splineToSplineHeading(new Pose2d(10, 70, Math.toRadians(180)), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(10, 65, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(50, 65), Math.toRadians(0))

                    .addTemporalMarker(.1, () -> {

                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) (maxSlideTicks * .2));
                        leftSlide.setTargetPosition((int) (maxSlideTicks * .2));
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })

                    .addTemporalMarker(3.8, () -> {
                        stick.setPosition(stickUp);
                    })
                    .addTemporalMarker(4.5, () -> {
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);

                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);
                    })

                    .build();




            drive.followTrajectorySequence(traj2BWB);

        }else if (autoStart == 1 && autoMode == 1 && barcodeValue == 2) { //Warehouse Blue park

            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(12, 62, Math.toRadians(270)));
            //drive.followTrajectorySequence(warehouseBluePark);
        }

        //TODO Carousel Red Bottom
        else if (autoStart == 2 && autoMode == 0 && barcodeValue == 0) { //Carousel Red, deliver freight top, noncompleted
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-32, -62, Math.toRadians(270)));
           // drive.followTrajectorySequence(carouselRedTop);
            TrajectorySequence CRtop = drive.trajectorySequenceBuilder(new Pose2d(-32, -62, Math.toRadians(270)))

                    .lineToSplineHeading(new Pose2d(-55, -55, Math.toRadians(360)))
                    .waitSeconds(5)
                    .lineToSplineHeading(new Pose2d(-33, -20, Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(-62, -34, Math.toRadians(0)))

                    .addTemporalMarker(2.5, () -> {
                        leftCarousel.setPower(-1);
                        rightCarousel.setPower(-1);

                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);
                    })
                    .addTemporalMarker(7.7, () -> {
                        leftCarousel.setPower(0);
                        rightCarousel.setPower(0);
                    })
                    .addTemporalMarker(8, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) (maxSlideTicks * .2));
                        leftSlide.setTargetPosition((int) (maxSlideTicks * .2));
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);
                    })
                    .addTemporalMarker(9.6, () -> {

                        stick.setPosition(stickUp);
                    })
                    .addTemporalMarker(10.2, () -> {
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);
                    })


                    .build();
            drive.followTrajectorySequence(CRtop);

        } //TODO Carousel Red Middle
        else if (autoStart == 2 && autoMode == 0 && barcodeValue == 1) { //Carousel Red, deliver freight top, noncompleted
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-32, -62, Math.toRadians(270)));
            // drive.followTrajectorySequence(carouselRedTop);
            TrajectorySequence CRtop = drive.trajectorySequenceBuilder(new Pose2d(-32, -62, Math.toRadians(270)))

                    .lineToSplineHeading(new Pose2d(-55, -55, Math.toRadians(360)))
                    .waitSeconds(5)
                    .lineToSplineHeading(new Pose2d(-33, -20, Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(-62, -34, Math.toRadians(0)))

                    .addTemporalMarker(2.5, () -> {
                        leftCarousel.setPower(-1);
                        rightCarousel.setPower(-1);

                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);
                    })
                    .addTemporalMarker(7.7, () -> {
                        leftCarousel.setPower(0);
                        rightCarousel.setPower(0);
                    })
                    .addTemporalMarker(8, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) (maxSlideTicks * .55));
                        leftSlide.setTargetPosition((int) (maxSlideTicks * .55));
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);
                    })
                    .addTemporalMarker(9.6, () -> {

                        stick.setPosition(stickUp);
                    })
                    .addTemporalMarker(10.2, () -> {
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);
                    })


                    .build();
            drive.followTrajectorySequence(CRtop);

        } //TODO Carousel Red Top
        else if (autoStart == 2 && autoMode == 0 && barcodeValue == 2) { //Carousel Red, deliver freight top, noncompleted
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-32, -62, Math.toRadians(270)));
            // drive.followTrajectorySequence(carouselRedTop);
            TrajectorySequence CRtop = drive.trajectorySequenceBuilder(new Pose2d(-32, -62, Math.toRadians(270)))

                    .lineToSplineHeading(new Pose2d(-55, -55, Math.toRadians(360)))
                    .waitSeconds(5)
                    .lineToSplineHeading(new Pose2d(-33, -20, Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(-62, -34, Math.toRadians(0)))

                    .addTemporalMarker(2.5, () -> {
                        leftCarousel.setPower(-1);
                        rightCarousel.setPower(-1);

                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);
                    })
                    .addTemporalMarker(7.7, () -> {
                        leftCarousel.setPower(0);
                        rightCarousel.setPower(0);
                    })
                    .addTemporalMarker(8, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);
                    })
                    .addTemporalMarker(9.6, () -> {

                        stick.setPosition(stickUp);
                    })
                    .addTemporalMarker(10.2, () -> {
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);
                    })


                    .build();
            drive.followTrajectorySequence(CRtop);

        } //TODO Carousel Blue Bottom
        else if (autoStart == 0 && autoMode == 0 && barcodeValue == 0) { //Carousel Red, deliver freight top, noncompleted
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-32, 62, Math.toRadians(-270)));
            // drive.followTrajectorySequence(carouselRedTop);
            TrajectorySequence CRtop = drive.trajectorySequenceBuilder(new Pose2d(-32, -62, Math.toRadians(270)))

                    .lineToSplineHeading(new Pose2d(-55, 55, Math.toRadians(-360)))
                    .waitSeconds(5)
                    .lineToSplineHeading(new Pose2d(-33, 20, Math.toRadians(-0)))
                    .lineToSplineHeading(new Pose2d(-62, 34, Math.toRadians(-0)))

                    .addTemporalMarker(2.5, () -> {
                        leftCarousel.setPower(-1);
                        rightCarousel.setPower(-1);

                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);
                    })
                    .addTemporalMarker(7.7, () -> {
                        leftCarousel.setPower(0);
                        rightCarousel.setPower(0);
                    })
                    .addTemporalMarker(8, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) (maxSlideTicks * .2));
                        leftSlide.setTargetPosition((int) (maxSlideTicks * .2));
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);
                    })
                    .addTemporalMarker(9.6, () -> {

                        stick.setPosition(stickUp);
                    })
                    .addTemporalMarker(10.2, () -> {
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);
                    })


                    .build();
            drive.followTrajectorySequence(CRtop);

        } //TODO Carousel Blue Middle
        else if (autoStart == 0 && autoMode == 0 && barcodeValue == 1) { //Carousel Red, deliver freight top, noncompleted
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-32, -62, Math.toRadians(270)));
            // drive.followTrajectorySequence(carouselRedTop);
            TrajectorySequence CRtop = drive.trajectorySequenceBuilder(new Pose2d(-32, 62, Math.toRadians(-270)))

                    .lineToSplineHeading(new Pose2d(-55, 55, Math.toRadians(-360)))
                    .waitSeconds(5)
                    .lineToSplineHeading(new Pose2d(-33, 20, Math.toRadians(-0)))
                    .lineToSplineHeading(new Pose2d(-62, 34, Math.toRadians(-0)))

                    .addTemporalMarker(2.5, () -> {
                        leftCarousel.setPower(-1);
                        rightCarousel.setPower(-1);

                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);
                    })
                    .addTemporalMarker(7.7, () -> {
                        leftCarousel.setPower(0);
                        rightCarousel.setPower(0);
                    })
                    .addTemporalMarker(8, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) (maxSlideTicks * .55));
                        leftSlide.setTargetPosition((int) (maxSlideTicks * .55));
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);
                    })
                    .addTemporalMarker(9.6, () -> {

                        stick.setPosition(stickUp);
                    })
                    .addTemporalMarker(10.2, () -> {
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);
                    })


                    .build();
            drive.followTrajectorySequence(CRtop);

        } //TODO Carousel Blue Top
        else if (autoStart == 0 && autoMode == 0 && barcodeValue == 2) { //Carousel Red, deliver freight top, noncompleted
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-32, 62, Math.toRadians(-270)));
            // drive.followTrajectorySequence(carouselRedTop);
            TrajectorySequence CRtop = drive.trajectorySequenceBuilder(new Pose2d(-32, 62, Math.toRadians(-270)))

                    .lineToSplineHeading(new Pose2d(-55, 55, Math.toRadians(-360)))
                    .waitSeconds(5)
                    .lineToSplineHeading(new Pose2d(-33, 20, Math.toRadians(-0)))
                    .lineToSplineHeading(new Pose2d(-62, 34, Math.toRadians(-0)))

                    .addTemporalMarker(2.5, () -> {
                        leftCarousel.setPower(-1);
                        rightCarousel.setPower(-1);

                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);
                    })
                    .addTemporalMarker(7.7, () -> {
                        leftCarousel.setPower(0);
                        rightCarousel.setPower(0);
                    })
                    .addTemporalMarker(8, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);
                    })
                    .addTemporalMarker(9.6, () -> {

                        stick.setPosition(stickUp);
                    })
                    .addTemporalMarker(10.2, () -> {
                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);
                    })


                    .build();
            drive.followTrajectorySequence(CRtop);

        } else if (autoStart == 2 && autoMode == 1) { //Carousel Red park

        }

        //TODO Warehouse Red Top
        else if (autoStart == 3 && autoMode == 0 && barcodeValue == 2) { //Warehouse Red, deliver freight
            //drive.followTrajectorySequence(WarehouseRedTop);

            drive.setPoseEstimate(new Pose2d(12, -62, Math.toRadians(270)));

            Trajectory traj1RW = drive.trajectoryBuilder(new Pose2d(12, -62, Math.toRadians(270)))
                    //sleep(initialDelay * 1000);
                    .lineToSplineHeading(new Pose2d(-5, -45, Math.toRadians(110)))
                    /* .splineToSplineHeading(new Pose2d(10, -70, Math.toRadians(180)), Math.toRadians(-90))
                     .splineToConstantHeading(new Vector2d(50, -70), Math.toRadians(0))*/

                    .addTemporalMarker(.1, () -> {

                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })

                    .build();

            TrajectorySequence traj2RW = drive.trajectorySequenceBuilder(traj1RW.end())

                    .splineToSplineHeading(new Pose2d(10, -70, Math.toRadians(180)), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(50, -70), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(15, -70, Math.toRadians(180)))

//                                .splineToConstantHeading(new Vector2d(0, -70), Math.toRadians(0))
//                                .lineToSplineHeading(new Pose2d(5, -32, Math.toRadians(150)))
//                                .lineToSplineHeading(new Pose2d(5, -32, Math.toRadians(150)))
                    .splineToSplineHeading(new Pose2d(-5, -46, Math.toRadians(110)), Math.toRadians(90))
                    .waitSeconds(.5)
//                                .splineToSplineHeading(new Pose2d(15, -70, Math.toRadians(180)), Math.toRadians(-90))
                    .lineToSplineHeading(new Pose2d(15, -72, Math.toRadians(180)))

                    .splineToConstantHeading(new Vector2d(50, -72), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(18, -72, Math.toRadians(180)))

                    .splineToSplineHeading(new Pose2d(-5, -47, Math.toRadians(110)), Math.toRadians(90))
                    .waitSeconds(.5)
//                                .splineToSplineHeading(new Pose2d(15, -70, Math.toRadians(180)), Math.toRadians(-90))
                    .lineToSplineHeading(new Pose2d(15, -74, Math.toRadians(180)))

                    .splineToConstantHeading(new Vector2d(50, -74), Math.toRadians(0))





                    .addTemporalMarker(0, () -> {

                        stick.setPosition(stickUp);



                    })


                       .addTemporalMarker(.5, () -> {

                           rightSlide.setVelocity(maxSlideVelocity * .5);
                           leftSlide.setVelocity(maxSlideVelocity * .5);

                           leftDropdown.setPosition(down + .05);
                           rightDropdown.setPosition(down + .05);


                           rightSlide.setTargetPosition(0);
                           leftSlide.setTargetPosition(0);



                       })

                    .addTemporalMarker(3, () -> {

                        tubeys.setPower(.75);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);




                    })


                    .addTemporalMarker(5.36, () -> {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

                    })
                    .addTemporalMarker(6, () -> {
                        tubeys.setPower(0);

                    })

                    .addTemporalMarker(6.5, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })
                    .addTemporalMarker(8, () -> {

                        stick.setPosition(stickUp);



                    })
//
//
                    .addTemporalMarker(8.3, () -> {

                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);



                    })

                    .addTemporalMarker(10.8, () -> {

                        tubeys.setPower(.75);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);




                    })


                    .addTemporalMarker(12.66, () -> {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

                    })
                    .addTemporalMarker(13.8, () -> {
                        tubeys.setPower(0);

                    })

                    .addTemporalMarker(14.3, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })
                    .addTemporalMarker(15.8, () -> {

                        stick.setPosition(stickUp);



                    })
//
//
                    .addTemporalMarker(16.1, () -> {

                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);


                    })

                    .addTemporalMarker(18.6, () -> {

                        tubeys.setPower(.75);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);




                    })


                    .addTemporalMarker(21.46, () -> {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

                    })

                    .build();

            drive.followTrajectory(traj1RW);
            drive.followTrajectorySequence(traj2RW);








        }

        //TODO Warehouse Red Middle
        else if (autoStart == 3 && autoMode == 0 && barcodeValue == 1) { //Warehouse Red, deliver freight
            //drive.followTrajectorySequence(WarehouseRedTop);

            drive.setPoseEstimate(new Pose2d(12, -62, Math.toRadians(270)));

            Trajectory traj1RWM = drive.trajectoryBuilder(new Pose2d(12, -62, Math.toRadians(270)))
                    //sleep(initialDelay * 1000);
                    .lineToSplineHeading(new Pose2d(-5, -45, Math.toRadians(110)))
                    /* .splineToSplineHeading(new Pose2d(10, -70, Math.toRadians(180)), Math.toRadians(-90))
                     .splineToConstantHeading(new Vector2d(50, -70), Math.toRadians(0))*/

                    .addTemporalMarker(.1, () -> {

                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) (maxSlideTicks * .55));
                        leftSlide.setTargetPosition((int) (maxSlideTicks * .55));
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })

                    .build();

            TrajectorySequence traj2RWM = drive.trajectorySequenceBuilder(traj1RWM .end())

                    .splineToSplineHeading(new Pose2d(10, -70, Math.toRadians(180)), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(50, -70), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(15, -70, Math.toRadians(180)))

//                                .splineToConstantHeading(new Vector2d(0, -70), Math.toRadians(0))
//                                .lineToSplineHeading(new Pose2d(5, -32, Math.toRadians(150)))
//                                .lineToSplineHeading(new Pose2d(5, -32, Math.toRadians(150)))
                    .splineToSplineHeading(new Pose2d(-5, -46, Math.toRadians(110)), Math.toRadians(90))
                    .waitSeconds(.5)
//                                .splineToSplineHeading(new Pose2d(15, -70, Math.toRadians(180)), Math.toRadians(-90))
                    .lineToSplineHeading(new Pose2d(15, -72, Math.toRadians(180)))

                    .splineToConstantHeading(new Vector2d(50, -72), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(18, -72, Math.toRadians(180)))

                    .splineToSplineHeading(new Pose2d(-5, -47, Math.toRadians(110)), Math.toRadians(90))
                    .waitSeconds(.5)
//                                .splineToSplineHeading(new Pose2d(15, -70, Math.toRadians(180)), Math.toRadians(-90))
                    .lineToSplineHeading(new Pose2d(15, -74, Math.toRadians(180)))

                    .splineToConstantHeading(new Vector2d(50, -74), Math.toRadians(0))





                    .addTemporalMarker(0, () -> {

                        stick.setPosition(stickUp);



                    })


                    .addTemporalMarker(.5, () -> {

                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);



                    })

                    .addTemporalMarker(3, () -> {

                        tubeys.setPower(.75);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);




                    })


                    .addTemporalMarker(5.36, () -> {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

                    })
                    .addTemporalMarker(6, () -> {
                        tubeys.setPower(0);

                    })

                    .addTemporalMarker(6.5, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })
                    .addTemporalMarker(8, () -> {

                        stick.setPosition(stickUp);



                    })
//
//
                    .addTemporalMarker(8.3, () -> {

                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);



                    })

                    .addTemporalMarker(10.8, () -> {

                        tubeys.setPower(.75);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);




                    })


                    .addTemporalMarker(12.66, () -> {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

                    })
                    .addTemporalMarker(13.8, () -> {
                        tubeys.setPower(0);

                    })

                    .addTemporalMarker(14.3, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })
                    .addTemporalMarker(15.8, () -> {

                        stick.setPosition(stickUp);



                    })
//
//
                    .addTemporalMarker(16.1, () -> {

                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);


                    })

                    .addTemporalMarker(18.6, () -> {

                        tubeys.setPower(.75);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);




                    })


                    .addTemporalMarker(21.46, () -> {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

                    })

                    .build();

            drive.followTrajectory(traj1RWM);
            drive.followTrajectorySequence(traj2RWM);








        }
        //TODO Warehouse Red Bottom
        else if (autoStart == 3 && autoMode == 0 && barcodeValue == 0) { //Warehouse Red, deliver freight
            //drive.followTrajectorySequence(WarehouseRedTop);

            drive.setPoseEstimate(new Pose2d(12, -62, Math.toRadians(270)));

            Trajectory traj1RWL = drive.trajectoryBuilder(new Pose2d(12, -62, Math.toRadians(270)))
                    //sleep(initialDelay * 1000);
                    .lineToSplineHeading(new Pose2d(-5, -45, Math.toRadians(110)))
                    /* .splineToSplineHeading(new Pose2d(10, -70, Math.toRadians(180)), Math.toRadians(-90))
                     .splineToConstantHeading(new Vector2d(50, -70), Math.toRadians(0))*/

                    .addTemporalMarker(.1, () -> {

                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) (maxSlideTicks * .2));
                        leftSlide.setTargetPosition((int) (maxSlideTicks * .2));
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })

                    .build();

            TrajectorySequence traj2RWL = drive.trajectorySequenceBuilder(traj1RWL .end())

                    .splineToSplineHeading(new Pose2d(10, -70, Math.toRadians(180)), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(50, -70), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(15, -70, Math.toRadians(180)))

//                                .splineToConstantHeading(new Vector2d(0, -70), Math.toRadians(0))
//                                .lineToSplineHeading(new Pose2d(5, -32, Math.toRadians(150)))
//                                .lineToSplineHeading(new Pose2d(5, -32, Math.toRadians(150)))
                    .splineToSplineHeading(new Pose2d(-5, -46, Math.toRadians(110)), Math.toRadians(90))
                    .waitSeconds(.5)
//                                .splineToSplineHeading(new Pose2d(15, -70, Math.toRadians(180)), Math.toRadians(-90))
                    .lineToSplineHeading(new Pose2d(15, -72, Math.toRadians(180)))

                    .splineToConstantHeading(new Vector2d(50, -72), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(18, -72, Math.toRadians(180)))

                    .splineToSplineHeading(new Pose2d(-5, -47, Math.toRadians(110)), Math.toRadians(90))
                    .waitSeconds(.5)
//                                .splineToSplineHeading(new Pose2d(15, -70, Math.toRadians(180)), Math.toRadians(-90))
                    .lineToSplineHeading(new Pose2d(15, -74, Math.toRadians(180)))

                    .splineToConstantHeading(new Vector2d(50, -74), Math.toRadians(0))





                    .addTemporalMarker(0, () -> {

                        stick.setPosition(stickUp);



                    })


                    .addTemporalMarker(.5, () -> {

                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);



                    })

                    .addTemporalMarker(3, () -> {

                        tubeys.setPower(.75);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);




                    })


                    .addTemporalMarker(5.36, () -> {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

                    })
                    .addTemporalMarker(6, () -> {
                        tubeys.setPower(0);

                    })

                    .addTemporalMarker(6.5, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })
                    .addTemporalMarker(8, () -> {

                        stick.setPosition(stickUp);



                    })
//
//
                    .addTemporalMarker(8.3, () -> {

                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);



                    })

                    .addTemporalMarker(10.8, () -> {

                        tubeys.setPower(.75);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);




                    })


                    .addTemporalMarker(12.66, () -> {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

                    })
                    .addTemporalMarker(13.8, () -> {
                        tubeys.setPower(0);

                    })

                    .addTemporalMarker(14.3, () -> {
                        rightSlide.setPower(.5);
                        leftSlide.setPower(.5);
                        rightSlide.setTargetPosition((int) maxSlideTicks);
                        leftSlide.setTargetPosition((int) maxSlideTicks);
                        rightDropdown.setPosition(forward);
                        leftDropdown.setPosition(forward);
                        stick.setPosition(stickDown);


                    })
                    .addTemporalMarker(15.8, () -> {

                        stick.setPosition(stickUp);



                    })
//
//
                    .addTemporalMarker(16.1, () -> {

                        rightSlide.setVelocity(maxSlideVelocity * .5);
                        leftSlide.setVelocity(maxSlideVelocity * .5);

                        leftDropdown.setPosition(down + .05);
                        rightDropdown.setPosition(down + .05);


                        rightSlide.setTargetPosition(0);
                        leftSlide.setTargetPosition(0);


                    })

                    .addTemporalMarker(18.6, () -> {

                        tubeys.setPower(.75);
                        stick.setPosition(stickUp);
                        leftDropdown.setPosition(down);
                        rightDropdown.setPosition(down);




                    })


                    .addTemporalMarker(21.46, () -> {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

                    })

                    .build();

            drive.followTrajectory(traj1RWL);
            drive.followTrajectorySequence(traj2RWL);








        }else if (autoStart == 3 && autoMode == 1) { //Warehouse Red park

        }



        //TODO Deliver Freight and Park


//        drive.setPoseEstimate( new Pose2d(12, 62, Math.toRadians(270)));
//        drive.followTrajectorySequence(WarehouseBlueTop);






    }



}

