package org.firstinspires.ftc.teamcode.scoring;

/** Monotone measured physical-unit to command (or voltage to physical-unit) knots. No extrapolation. */
public final class CalibratedAxis {
    private final double[] input,output;
    public CalibratedAxis(double[] input,double[] output) {
        if(input==null||output==null||input.length<2||input.length!=output.length)
            throw new IllegalArgumentException("At least two measured axis knots required");
        this.input=input.clone();this.output=output.clone();
        for(int i=0;i<input.length;i++) {
            if(!Double.isFinite(input[i])||!Double.isFinite(output[i])||(i>0&&input[i]<=input[i-1]))
                throw new IllegalArgumentException("Finite increasing input knots required");
            if(i>0&&(output[i]-output[i-1])*(output[1]-output[0])<=0)
                throw new IllegalArgumentException("Monotone output knots required");
        }
    }
    public double map(double value) {
        if(!Double.isFinite(value)||value<input[0]||value>input[input.length-1])return Double.NaN;
        for(int i=1;i<input.length;i++)if(value<=input[i])
            return output[i-1]+(output[i]-output[i-1])*(value-input[i-1])/(input[i]-input[i-1]);
        return Double.NaN;
    }
    public double minInput() {return input[0];}
    public double maxInput() {return input[input.length-1];}
}
