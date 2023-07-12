/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

using System;
using System.Collections.Generic;
using System.IO;
using LCM.LCM;
 
namespace LCMTypes
{
    public sealed class loco2dOpti_lcmt : LCM.LCM.LCMEncodable
    {
        public double[] body_pos;
        public double theta;
        public double r_under_contact;
        public double r_contact_pos;
        public double l_under_contact;
        public double l_contact_pos;
 
        public loco2dOpti_lcmt()
        {
            body_pos = new double[2];
        }
 
        public static readonly ulong LCM_FINGERPRINT;
        public static readonly ulong LCM_FINGERPRINT_BASE = 0xef81033a5293f20eL;
 
        static loco2dOpti_lcmt()
        {
            LCM_FINGERPRINT = _hashRecursive(new List<String>());
        }
 
        public static ulong _hashRecursive(List<String> classes)
        {
            if (classes.Contains("LCMTypes.loco2dOpti_lcmt"))
                return 0L;
 
            classes.Add("LCMTypes.loco2dOpti_lcmt");
            ulong hash = LCM_FINGERPRINT_BASE
                ;
            classes.RemoveAt(classes.Count - 1);
            return (hash<<1) + ((hash>>63)&1);
        }
 
        public void Encode(LCMDataOutputStream outs)
        {
            outs.Write((long) LCM_FINGERPRINT);
            _encodeRecursive(outs);
        }
 
        public void _encodeRecursive(LCMDataOutputStream outs)
        {
            for (int a = 0; a < 2; a++) {
                outs.Write(this.body_pos[a]); 
            }
 
            outs.Write(this.theta); 
 
            outs.Write(this.r_under_contact); 
 
            outs.Write(this.r_contact_pos); 
 
            outs.Write(this.l_under_contact); 
 
            outs.Write(this.l_contact_pos); 
 
        }
 
        public loco2dOpti_lcmt(byte[] data) : this(new LCMDataInputStream(data))
        {
        }
 
        public loco2dOpti_lcmt(LCMDataInputStream ins)
        {
            if ((ulong) ins.ReadInt64() != LCM_FINGERPRINT)
                throw new System.IO.IOException("LCM Decode error: bad fingerprint");
 
            _decodeRecursive(ins);
        }
 
        public static LCMTypes.loco2dOpti_lcmt _decodeRecursiveFactory(LCMDataInputStream ins)
        {
            LCMTypes.loco2dOpti_lcmt o = new LCMTypes.loco2dOpti_lcmt();
            o._decodeRecursive(ins);
            return o;
        }
 
        public void _decodeRecursive(LCMDataInputStream ins)
        {
            this.body_pos = new double[(int) 2];
            for (int a = 0; a < 2; a++) {
                this.body_pos[a] = ins.ReadDouble();
            }
 
            this.theta = ins.ReadDouble();
 
            this.r_under_contact = ins.ReadDouble();
 
            this.r_contact_pos = ins.ReadDouble();
 
            this.l_under_contact = ins.ReadDouble();
 
            this.l_contact_pos = ins.ReadDouble();
 
        }
 
        public LCMTypes.loco2dOpti_lcmt Copy()
        {
            LCMTypes.loco2dOpti_lcmt outobj = new LCMTypes.loco2dOpti_lcmt();
            outobj.body_pos = new double[(int) 2];
            for (int a = 0; a < 2; a++) {
                outobj.body_pos[a] = this.body_pos[a];
            }
 
            outobj.theta = this.theta;
 
            outobj.r_under_contact = this.r_under_contact;
 
            outobj.r_contact_pos = this.r_contact_pos;
 
            outobj.l_under_contact = this.l_under_contact;
 
            outobj.l_contact_pos = this.l_contact_pos;
 
            return outobj;
        }
    }
}
