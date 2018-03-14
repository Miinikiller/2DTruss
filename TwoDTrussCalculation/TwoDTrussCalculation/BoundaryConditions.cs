using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using TwoDTrussCalculation.Properties;

namespace TwoDTrussCalculation
{
    public class BoundaryConditions : GH_Component
    {

        public BoundaryConditions()
          : base("BDC", "BDC",
              "Set boundary conditions at nodes",
              "Koala", "2D Truss")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "Points to apply Boundary Conditions", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Boundary Conditions", "BDC", "Boundary Conditions x,y,z where 0=clamped and 1=free", GH_ParamAccess.list, new List<int>(new int[] { 0, 0, 0 }));
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("B.Cond.", "BDC", "Boundary Conditions for 2D Truss Calculation", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //Expected inputs
            List<Point3d> pointList = new List<Point3d>();          //List of points where BDC is to be applied
            List<int> BDC = new List<int>();                        //is BDC free? (=clamped) (1 == true, 0 == false)
            List<string> pointInStringFormat = new List<string>();  //output in form of list of strings


            //Set expected inputs from Indata and aborts with error message if input is incorrect
            if (!DA.GetDataList(0, pointList)) return;
            if (!DA.GetDataList(1, BDC)) { AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "testing"); return; }


            //Preallocate temporary variables
            string BDCString;
            int bdcx = 0;
            int bdcy = 0;
            int bdcz = 0;


            if (BDC.Count == 3) //Boundary condition input for identical conditions in all points. Split into if/else for optimization
            {
                bdcx = BDC[0];
                bdcy = BDC[1];
                bdcz = BDC[2];

                BDCString = bdcx + "," + bdcy + "," + bdcz;

                for (int i = 0; i < pointList.Count; i++)   //Format stringline for all points (identical boundary conditions for all points)
                {
                    pointInStringFormat.Add(pointList[i].X + "," + pointList[i].Y + "," + pointList[i].Z + ":" + BDCString);
                }
            }
            else    //BDCs are not identical for all points
            {
                for (int i = 0; i < pointList.Count; i++)
                {
                    if (i > (BDC.Count / 3) - 1)  //Are there more points than BDCs given? (BDC always lists x,y,z per point)
                    {
                        BDCString = bdcx + "," + bdcy + "," + bdcz; //use values from last BDC in list of BDCs
                    }
                    else
                    {
                        //retrieve BDC for x,y,z-dir
                        bdcx = BDC[i * 3];
                        bdcy = BDC[i * 3 + 1];
                        bdcz = BDC[i * 3 + 2];
                        BDCString = bdcx + "," + bdcy + "," + bdcz;
                    }
                    pointInStringFormat.Add(pointList[i].X + "," + pointList[i].Y + "," + pointList[i].Z + ":" + BDCString);    //Add stringline to list of strings
                }
            }
            DA.SetDataList(0, pointInStringFormat);
        } //End of main program

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.BoundaryCondition; //Setting component icon
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("0efc7b95-936a-4c88-8005-485398c61a31"); }
        }
    }
}