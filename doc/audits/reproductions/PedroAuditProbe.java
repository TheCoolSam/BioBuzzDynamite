import java.lang.reflect.Proxy;
import java.util.*;
import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.*;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.*;
import com.pedropathing.math.*;
import org.firstinspires.ftc.teamcode.pedro.PedroManualDrive;

public final class PedroAuditProbe {
    static final class L implements Localizer {
        MotionState value=MotionState.zero();
        public void setPose(Pose p) { value=value.withPose(p); }
        public MotionState state(){ return value; }
        public void update(){}
        public void reset(){value=MotionState.zero();}
    }
    static final class D implements Drivetrain {
        DrivePowers last=DrivePowers.zero();
        public void drive(DrivePowers p,boolean manual){last=p;}
        public double maxScaling(DrivePowers p,DrivePowers q){return 1;}
        public void stop(){last=DrivePowers.zero();}
        public void stop(boolean b){stop();}
        public Map<String,Object> debug(){return Collections.emptyMap();}
        public double interpolateVelocity(double a,double b,double c){return 1;}
    }
    public static void main(String[] args) {
        L l=new L(); D d=new D();
        Algorithm algorithm=(Algorithm)Proxy.newProxyInstance(Algorithm.class.getClassLoader(),new Class<?>[]{Algorithm.class},(proxy,method,values)->null);
        Follower f=new Follower(l,d,algorithm);
        PedroManualDrive.request(f,.5,0,0); f.update(.02);
        l.setPose(new Pose(0,0,Double.NaN));
        boolean accepted=PedroManualDrive.request(f,0,0,0); f.update(.02);
        System.out.println("Drive command accepted="+accepted+"; actual output after invalid heading="+d.last);
    }
}
