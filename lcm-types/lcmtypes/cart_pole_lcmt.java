/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class cart_pole_lcmt implements lcm.lcm.LCMEncodable
{
    public float link1_pos[];
    public float link2_pos[];
 
    public cart_pole_lcmt()
    {
        link1_pos = new float[3];
        link2_pos = new float[3];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xd4aaa707e5c69bc8L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.cart_pole_lcmt.class))
            return 0L;
 
        classes.add(lcmtypes.cart_pole_lcmt.class);
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
        for (int a = 0; a < 3; a++) {
            outs.writeFloat(this.link1_pos[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeFloat(this.link2_pos[a]); 
        }
 
    }
 
    public cart_pole_lcmt(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public cart_pole_lcmt(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.cart_pole_lcmt _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.cart_pole_lcmt o = new lcmtypes.cart_pole_lcmt();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.link1_pos = new float[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.link1_pos[a] = ins.readFloat();
        }
 
        this.link2_pos = new float[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.link2_pos[a] = ins.readFloat();
        }
 
    }
 
    public lcmtypes.cart_pole_lcmt copy()
    {
        lcmtypes.cart_pole_lcmt outobj = new lcmtypes.cart_pole_lcmt();
        outobj.link1_pos = new float[(int) 3];
        System.arraycopy(this.link1_pos, 0, outobj.link1_pos, 0, 3); 
        outobj.link2_pos = new float[(int) 3];
        System.arraycopy(this.link2_pos, 0, outobj.link2_pos, 0, 3); 
        return outobj;
    }
 
}

