using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MageInterface.Kinematics
{
  class KinTrunnionBC : Kinematics
  {
    override public void InverseKinematics(double[] tar_xyz, double[] tar_ijk,
      double[] pre_gcd, out double[] next)
    {
      next = new double[6];
      double pre_tilt = pre_gcd[4];
      double pre_rot = pre_gcd[5];
      double tilt;
      switch(this.Type)
      {
        case "Normal":
          tilt = -Math.Acos(tar_ijk[2]);
          break;
        case "For5x":
          tilt = -Math.Acos(tar_ijk[2]);
          break;
        case "For4x":
          tilt = -Math.Atan2(tar_ijk[0], tar_ijk[2]);
          break;
        default:
          tilt = -Math.Acos(tar_ijk[2]);
          break;
      }
      // OLD
      //double tilt = -Math.Acos(tar_ijk[2]);
      // NEW
      //double tilt = -Math.Atan2(tar_ijk[0], tar_ijk[2]);
      double rot;
      if (tar_ijk[0] == 0.0)
      {
        if (Math.Sin(tilt) == 0.0)
        {
          rot = pre_rot;
        }
        else
        {
          double buf1 = Math.Abs(pre_rot - Math.PI / 2.0);
          double buf2 = Math.Abs(pre_rot + Math.PI / 2.0);
          if (buf1 < buf2)
          {
            rot = Math.PI / 2.0;
          }
          else
          {
            rot = -Math.PI / 2.0;
          }
        }
      }
      else
      {
        // OLD
        //rot = -Math.Atan2(tar_ijk[1], tar_ijk[0]);
        // NEW
        //rot = -Math.Asin(tar_ijk[1]);
        switch(this.Type)
        {
          case "Normal":
            rot = -Math.Atan2(tar_ijk[1], tar_ijk[0]);
            break;
          case "For5x":
            rot = -Math.Atan2(tar_ijk[1], tar_ijk[0]);
            break;
          case "For4x":
            rot = -Math.Asin(tar_ijk[1]);
            break;
          default:
            rot = -Math.Atan2(tar_ijk[1], tar_ijk[0]);
            break;
        }
      }
      // rot
      int buf_base = (int)Math.Ceiling(pre_rot / Math.PI / 2.0);
      double buf_1 = (buf_base - 1) * Math.PI * 2.0 + rot;
      double buf_2 = buf_base * Math.PI * 2.0 + rot;
      double buf_3 = (buf_base + 1) * Math.PI * 2.0 + rot;
      double dist_1 = Math.Abs(buf_1 - pre_rot);
      double dist_2 = Math.Abs(buf_2 - pre_rot);
      double dist_3 = Math.Abs(buf_3 - pre_rot);
      if (dist_1 <= dist_2 && dist_1 <= dist_3)
      {
        rot = buf_1;
      }
      else if (dist_2 <= dist_1 && dist_2 <= dist_3)
      {
        rot = buf_2;
      }
      else
      {
        rot = buf_3;
      }

      // pos
      double bx1 = Math.Cos(rot) * tar_xyz[0] - Math.Sin(rot) * tar_xyz[1];
      double by1 = Math.Sin(rot) * tar_xyz[0] + Math.Cos(rot) * tar_xyz[1];
      double bz1 = tar_xyz[2];
      double bx2 = Math.Cos(-tilt) * bx1 - Math.Sin(-tilt) * bz1;
      double by2 = by1;
      double bz2 = Math.Sin(-tilt) * bx1 + Math.Cos(-tilt) * bz1;
      next[0] = bx2 + this.OriginX;
      next[1] = by2 + this.OriginY;
      next[2] = bz2 + this.OriginZ;
      next[3] = 0.0;
      next[4] = tilt;
      next[5] = rot;
    }

    override public void InverseKinematicsWithABC(double[] tar_xyz, double tar_a, double tar_b, double tar_c,
      double[] pre_gcd, out double[] next)
    {
      next = new double[6];
      double pre_tilt = pre_gcd[4];
      double pre_rot = pre_gcd[5];
      double tilt = tar_b;
      double rot = tar_c;
      int buf_base = (int)Math.Ceiling(pre_rot / Math.PI / 2.0);
      double buf_1 = (buf_base - 1) * Math.PI * 2.0 + rot - buf_base * Math.PI * 2.0;
      double buf_2 = buf_base * Math.PI * 2.0 + rot - buf_base * Math.PI * 2.0;
      double buf_3 = (buf_base + 1) * Math.PI * 2.0 + rot - buf_base * Math.PI * 2.0;
      double dist_1 = Math.Abs(buf_1 - pre_rot);
      double dist_2 = Math.Abs(buf_2 - pre_rot);
      double dist_3 = Math.Abs(buf_3 - pre_rot);
      if (dist_1 <= dist_2 && dist_1 <= dist_3)
      {
        rot = buf_1;
      }
      else if (dist_2 <= dist_1 && dist_2 <= dist_3)
      {
        rot = buf_2;
      }
      else
      {
        rot = buf_3;
      }

      // pos
      double bx1 = Math.Cos(rot) * tar_xyz[0] - Math.Sin(rot) * tar_xyz[1];
      double by1 = Math.Sin(rot) * tar_xyz[0] + Math.Cos(rot) * tar_xyz[1];
      double bz1 = tar_xyz[2];
      double bx2 = Math.Cos(-tilt) * bx1 - Math.Sin(-tilt) * bz1;
      double by2 = by1;
      double bz2 = Math.Sin(-tilt) * bx1 + Math.Cos(-tilt) * bz1;
      next[0] = bx2 + this.OriginX;
      next[1] = by2 + this.OriginY;
      next[2] = bz2 + this.OriginZ;
      next[3] = 0.0;
      next[4] = tilt;
      next[5] = rot;
    }
    override public string[] OutputGcodeNormal(double[] next, double pe, double edelta, double printFval, out double ne)
    {
      List<string> ret = new List<string>();
      string buf = string.Format("G1 X{0:F3} Y{1:F3} Z{2:F3} B{3:F3} C{4:F3} E{5:F5} F{6:F2}", 
        next[0], next[1], next[2], next[4] / Math.PI * 180.0, next[5] / Math.PI * 180.0, 
        pe + edelta, printFval * 60.0);
      ret.Add(buf);
      ne = pe + edelta;
      return ret.ToArray();
    }
    override public string[] OutputGcodeHopping(double[] next, double pe,
      double retLength, double retPull, double retPush, double hopZ, double moveFval, double[] prev)
    {
      List<string> ret = new List<string>();
      double bufZ = Math.Max(next[2], prev[2]);
      string buf;
      buf = string.Format("G0 E{0:F5} F{1:F2}", pe -retLength, retPull * 60.0);
      ret.Add(buf);
      buf = string.Format("G0 Z{0:F3} F{1:F2}", bufZ + hopZ, moveFval * 60.0);
      ret.Add(buf);
      buf = string.Format("G0 X{0:F3} Y{1:F3} B{2:F3} C{3:F3} F{4:F2}", 
        next[0], next[1], next[4] / Math.PI * 180.0, next[5] / Math.PI * 180.0, moveFval * 60.0);
      ret.Add(buf);
      buf = string.Format("G0 Z{0:F3} F{1:F2}", next[2], moveFval * 60.0);
      ret.Add(buf);
      buf = string.Format("G0 E{0:F5} F{1:F2}", pe, retPush * 60.0);
      ret.Add(buf);
      return ret.ToArray();
    }
  }
}
