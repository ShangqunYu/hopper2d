/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class hopper2dOpti_lcmt implements lcm.lcm.LCMEncodable
{
    public double body_pos[];
    public double theta;
    public double under_contact;
    public double contact_pos;
 
    public hopper2dOpti_lcmt()
    {
        body_pos = new double[2];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x49474f0536e92fb3L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.hopper2dOpti_lcmt.class))
            return 0L;
 
        classes.add(lcmtypes.hopper2dOpti_lcmt.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        for (int a = 0; a < 2; a++) {
            outs.writeDouble(this.body_pos[a]); 
        }
 
        outs.writeDouble(this.theta); 
 
        outs.writeDouble(this.under_contact); 
 
        outs.writeDouble(this.contact_pos); 
 
    }
 
    public hopper2dOpti_lcmt(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public hopper2dOpti_lcmt(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.hopper2dOpti_lcmt _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.hopper2dOpti_lcmt o = new lcmtypes.hopper2dOpti_lcmt();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.body_pos = new double[(int) 2];
        for (int a = 0; a < 2; a++) {
            this.body_pos[a] = ins.readDouble();
        }
 
        this.theta = ins.readDouble();
 
        this.under_contact = ins.readDouble();
 
        this.contact_pos = ins.readDouble();
 
    }
 
    public lcmtypes.hopper2dOpti_lcmt copy()
    {
        lcmtypes.hopper2dOpti_lcmt outobj = new lcmtypes.hopper2dOpti_lcmt();
        outobj.body_pos = new double[(int) 2];
        System.arraycopy(this.body_pos, 0, outobj.body_pos, 0, 2); 
        outobj.theta = this.theta;
 
        outobj.under_contact = this.under_contact;
 
        outobj.contact_pos = this.contact_pos;
 
        return outobj;
    }
 
}
