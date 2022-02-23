package Archive;/*
package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Control with Arm Encoders", group = "Testing")

@Disabled

public class ShowCaseBot extends LinearOpMode {




    //editable values
    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double wheelDiameter = 3.77953;
    private final double botRotationSpeed = .8; //always <= 1
    private final double wheelMaxVelocity = wheelMotorRPM * wheelMotorTicks / 60;

    private final double servoLeftStart= .7;
    private final double servoLeftEnd= .1;
    private final double servoRightStart = .2;
    private final double servoRightEnd= .75;

    private boolean bumpersPressed = false;
    private int activeGear = 2;//values 1 - 3

    //create the gear array
    private final double[] gearValues = {.18, .3, .5, 1.0};

    //create general variables
    private double gamepad1LY;
    private double gamepad1LX;
    private double gamepad1RX;

    //create motor objects
    private DcMotorEx flMotor;
    private DcMotorEx frMotor;
    private DcMotorEx blMotor;
    private DcMotorEx brMotor;



    //create arm objects
    private DcMotor intakeMotor; //port 0
    private Servo servoLeft; //port 0
    private Servo servoRight; //port 1

    //create arm variables
    private final double armTicks = 360;

    //arm position values
    private int armPosition = 0;
    private final double[] armValues = {
            0,
            70 / 360.0  * armTicks,
            150 / 360.0 * armTicks,
            175 / 360.0 * armTicks
    };
    //arm toggle booleans
    private boolean armMoving = false;
    private boolean clawOpen = false;
    private boolean clawActive = false;

    //method to change gears
    private void changeGears() {

        //if statement to change active gear count
        if (gamepad1.right_trigger > 0 && !bumpersPressed && activeGear < gearValues.length - 1) {

            //set boolean variable to true to avoid multiple inputs
            bumpersPressed = true;
            //add 1 to active gear
            activeGear++;

        } else if (gamepad1.left_trigger > 0 && !bumpersPressed && activeGear > 0) {

            //set boolean variable to true to avoid multiple inputs
            bumpersPressed = true;
            //subtract 1 from active gear
            activeGear--;

        } else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
            bumpersPressed = false;

    }
*/
/*
*//*
*/
/*    public void armControl() {

        if (gamepad2.left_stick_y > 0)
            armMotor.setTargetPosition(armMotor.getCurrentPosition() + 50);
        else if (gamepad2.left_stick_y < 0)
            armMotor.setTargetPosition(armMotor.getCurrentPosition() - 50);
        else if (gamepad2.left_stick_y == 0)
            armMotor.setTargetPosition(armMotor.getCurrentPosition());*//*
*/
/*

*//*
*/
/*
        armMotor.setTargetPosition((int) armValues[armPosition]);

        //increase and decrease current position
        if (gamepad1.dpad_up && armPosition > 0 && !armMoving) {
            //increase position
            armPosition--;
            armMoving = true;
        } else if (gamepad1.dpad_down && armPosition < 3 && !armMoving) {
            //decrease position
            armPosition++;
            armMoving = true;
        } else if (!gamepad1.dpad_up && !gamepad2.dpad_down && armMoving)
            armMoving = false;
*//*
*/
/*

    *//*
*/
/*    armMotor.setVelocity(armTicks);*//*
*/
/*


    }*//*


    public void clawControl() {

        if (gamepad2.y && !clawOpen && !clawActive) {
            //open claw
            servoLeft.setPosition(servoLeftEnd);
            servoRight.setPosition(servoRightEnd);
            clawActive = true;
            clawOpen = true;
        } else if (gamepad2.y && clawOpen && !clawActive) {
            //close claw
            servoLeft.setPosition(servoLeftStart);
            servoRight.setPosition(servoRightStart);
            clawActive = true;
            clawOpen = false;
        } else if (!gamepad2.y && clawActive)
            clawActive = false;


    }





    @Override
    public void runOpMode() throws InterruptedException {




        //TODO organize
        //setup driving motors
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        //flip right motors
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        brMotor.setDirection(DcMotor.Direction.REVERSE);
        //setup arm motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        */
/*armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*//*

        //armMotor.setDirection(DcMotor.Direction.REVERSE);
        //setup servos
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");



        waitForStart();

        servoLeft.setPosition(servoLeftStart);
        servoRight.setPosition(servoRightStart);


        //while loop
        while (opModeIsActive()) {

            //call shooter method to get rid of

            //update controller variables
            gamepad1LY = gamepad1.left_stick_y;
            gamepad1LX = gamepad1.left_stick_x;
            gamepad1RX = gamepad1.right_stick_x;

            //event methods
            changeGears();
            */
/*armControl();*//*

            clawControl();









            //---------------------------------------------------------------------
            //Mecanum Drivetrain Control
            //if statement to ensure robot won't move unless control sticks are moved
            if (Math.abs(gamepad1LY) > .05 || Math.abs(gamepad1LX) > .05 || Math.abs(gamepad1RX) > .05) {
                //control logic: multiply by gear array for different speeds,
                //multiply right stick movement by rotation fraction to reduce uncontrolability
                flMotor.setVelocity((gearValues[activeGear] * (gamepad1LY - gamepad1LX) -  gamepad1RX) * wheelMaxVelocity);
                frMotor.setVelocity((gearValues[activeGear] * (gamepad1LY + gamepad1LX) +  gamepad1RX) * wheelMaxVelocity);
                blMotor.setVelocity((gearValues[activeGear] * (gamepad1LY + gamepad1LX) -  gamepad1RX) * wheelMaxVelocity);
                brMotor.setVelocity((gearValues[activeGear] * (gamepad1LY - gamepad1LX) +  gamepad1RX) * wheelMaxVelocity);
            } else if (gamepad1.left_bumper && !gamepad1.right_bumper && Math.abs(gamepad1LY) < .05 && Math.abs(gamepad1LX) < .05 && Math.abs(gamepad1RX) < .05) {
                flMotor.setVelocity( .2 * wheelMaxVelocity);
                frMotor.setVelocity(-.2 * wheelMaxVelocity);
                blMotor.setVelocity( .2 * wheelMaxVelocity);
                brMotor.setVelocity(-.2 * wheelMaxVelocity);
            } else if (gamepad1.right_bumper && !gamepad1.left_bumper && Math.abs(gamepad1LY) < .05 && Math.abs(gamepad1LX) < .05 && Math.abs(gamepad1RX) < .05) {
                flMotor.setVelocity(-.2 * wheelMaxVelocity);
                frMotor.setVelocity( .2 * wheelMaxVelocity);
                blMotor.setVelocity(-.2 * wheelMaxVelocity);
                brMotor.setVelocity( .2 * wheelMaxVelocity);
            } else {
                flMotor.setVelocity(0);
                frMotor.setVelocity(0);
                blMotor.setVelocity(0);
                brMotor.setVelocity(0);
            }

            intakeMotor.setPower(gamepad2.right_stick_y);


            //send what gear is active
            telemetry.addData("Gear", activeGear + 1);
            telemetry.addData("Arm Position", armPosition + 1);

            telemetry.update();



        }

    }

}*/
