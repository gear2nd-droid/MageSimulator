using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MageInterface.Kinematics
{
  class KinCartesian : Kinematics
  {
    override public void InverseKinematics(double[] tar_xyz, double[] tar_ijk,
      double[] pre_gcd, out double[] next)
    {
      next = new double[6];
      next[0] = tar_xyz[0];
      next[1] = tar_xyz[1];
      next[2] = tar_xyz[2];
      next[3] = next[4] = next[5] = 0.0;
    }
    override public void InverseKinematicsWithABC(double[] tar_xyz, double tar_a, double tar_b, double tar_c,
      double[] pre_gcd, out double[] next)
    {
      next = new double[6];
      next[0] = tar_xyz[0];
      next[1] = tar_xyz[1];
      next[2] = tar_xyz[2];
      next[3] = next[4] = next[5] = 0.0;
    }
    override public string[] OutputGcodeNormal(double[] next, double pe, double edelta, double printFval, out double ne)
    {
      List<string> ret = new List<string>();
      string buf = string.Format("G1 X{0:F3} Y{1:F3} Z{2:F3} E{3:F5} F{4:F2}", 
        next[0], next[1], next[2], pe + edelta, printFval * 60.0);
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
      buf = string.Format("G0 X{0:F3} Y{1:F3} F{2:F2}", 
        next[0], next[1], moveFval * 60.0);
      ret.Add(buf);
      buf = string.Format("G0 Z{0:F3} F{1:F2}", next[2], moveFval * 60.0);
      ret.Add(buf);
      buf = string.Format("G0 E{0:F5} F{1:F2}", pe, retPush * 60.0);
      ret.Add(buf);
      return ret.ToArray();
    }
  }
}
