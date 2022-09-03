package org.firstinspires.ftc.teamcode;

public class Main{


    private double setpointRange;

    public static void main(String[] args) {
         Pid PID  ;



       // double p= 0.25 ;
        //double i = 0.01;
       // double d = 0.4;


       /// PID = new Pid( 0.25,  0.01 , 0.4);
       // Pid.setOutputLimits(10);
        //miniPID.setMaxIOutput(2);
        //miniPID.setOutputRampRate(3);
        //miniPID.setOutputFilter(.3);
     //   Pid.setSetpointRange(40);

        double target=100;

        double actual=0;
        double output=0;

      // Pid.setSetpoint(0);
    ///    Pid.setSetpoint(target);

        System.err.printf("Target\tActual\tOutput\tError\n");
        //System.err.printf("Output\tP\tI\tD\n");

        // Position based test code
        for (int i = 0; i < 100; i++){

            //if(i==50)miniPID.setI(.05);

            if (i == 60)
                target = 50;

            //if(i==75)target=(100);
            //if(i>50 && i%4==0)target=target+(Math.random()-.5)*50;

            //qoutput = Pid.getOutput(actual, target);
            actual = actual + output;

            //System.out.println("==========================");
            //System.out.printf("Current: %3.2f , Actual: %3.2f, Error: %3.2f\n",actual, output, (target-actual));
            System.err.printf("%3.2f\t%3.2f\t%3.2f\t%3.2f\n", target, actual, output, (target-actual));

            //if(i>80 && i%5==0)actual+=(Math.random()-.5)*20;
        }
    }
   //public void setSetpointRange(double range){
      //  setpointRange=range;
    }

   // private static class pid {
       // public void setSetpoint(double target) {
       // }

     //

       // }






