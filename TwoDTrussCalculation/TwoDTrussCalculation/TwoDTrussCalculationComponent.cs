using System;
using System.Collections.Generic;
using System.Linq;

using Grasshopper.Kernel;
using Rhino.Geometry;
using TwoDTrussCalculation.Properties;

namespace TwoDTrussCalculation
{
    public class TwoDTrussCalculationComponent : GH_Component
    {
        public TwoDTrussCalculationComponent()
          : base("2D Truss Calc.", "2DTrussCalc",
              "Description",
              "Koala", "2D Truss")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddLineParameter("Lines", "LNS", "Geometry, in form of Lines)", GH_ParamAccess.list);
            pManager.AddTextParameter("Boundary Conditions", "BDC", "Boundary Conditions in form (x,z):1,1 where 1=free and 0=restrained", GH_ParamAccess.list);
            pManager.AddNumberParameter("Crossection area", "A", "Crossectional area, initial value 10e3 [mm*mm]", GH_ParamAccess.item, 10000);
            pManager.AddNumberParameter("Material E modulus", "E", "Material Property, initial value 210e3 [MPa]", GH_ParamAccess.item, 200000);
            pManager.AddTextParameter("Loads", "L", "Load given as Vector [N]", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {

            pManager.AddNumberParameter("Deformations", "Def", "Deformations", GH_ParamAccess.list);
            pManager.AddNumberParameter("Reactions", "R", "Reaction Forces", GH_ParamAccess.list);
            pManager.AddNumberParameter("Element stresses", "Strs", "The Stress in each element", GH_ParamAccess.list);
            pManager.AddNumberParameter("Element strains", "Strn", "The Strain in each element", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //Expected inputs
            List<Line> geometry = new List<Line>();         //initial Geometry of lines
            double E = 0;                                   //Material property, initial value 210000 [MPa]
            double A = 0;                                   //Area for each element in same order as geometry, initial value 10000 [mm^2]
            List<string> bdctxt = new List<string>();       //Boundary conditions in string format
            List<string> loadtxt = new List<string>();      //loads in string format


            //Set expected inputs from Indata
            if (!DA.GetDataList(0, geometry)) return;       //sets geometry
            if (!DA.GetDataList(1, bdctxt)) return;         //sets boundary conditions
            if (!DA.GetData(2, ref A)) return;              //sets Area
            if (!DA.GetData(3, ref E)) return;              //sets material
            if (!DA.GetDataList(4, loadtxt)) return;        //sets load


            //List all nodes (every node only once), numbering them according to list index
            List<Point3d> points = CreatePointList(geometry);


            //Interpret the BDC inputs (text) and create list of boundary condition (1/0 = free/clamped) for each dof.
            List<int> bdc_value = CreateBDCList(bdctxt, points);


            //Interpreting input load (text) and creating load list (double)
            List<double> load = CreateLoadList(loadtxt, points);


            //Create global stiffness matrix
            double[,] K_tot = CreateGlobalStiffnessMatrix(geometry, points, E, A);


            //Create the reduced global stiffness matrix and reduced load list
            int dofs_red = points.Count * 2 - (bdc_value.Count - bdc_value.Sum());                          //reduced number of dofs
            double[,] K_red = new double[dofs_red, dofs_red];                                               //preallocate reduced K matrix
            List<double> load_red = new List<double>();                                                     //preallocate reduced load list
            CreateReducedGlobalStiffnessMatrix(points, bdc_value, K_tot, load, out K_red, out load_red);    //outputs are reduced K-matrix and reduced load list (removed free dofs)


            //Run the cholesky method for solving the system of equations for the deformations
            List<double> deformations_red = Cholesky_Banachiewicz(K_red, load_red);


            //Add the clamped dofs (= 0) to the deformations list
            List<double> deformations = RestoreTotalDeformationVector(deformations_red, bdc_value);


            //Calculate the reaction forces from the deformations
            List<double> Reactions = CalculateReactionforces(deformations, K_tot, bdc_value);


            //Calculate the internal strains and stresses in each member
            List<double> internalStresses;
            List<double> internalStrains;
            CalculateInternalStrainsAndStresses(deformations, points, E, geometry, out internalStresses, out internalStrains);

            //Set output data
            string K_print = PrintStiffnessMatrix(K_red);
            string K_print1 = PrintStiffnessMatrix(K_tot);

            DA.SetDataList(0, deformations);
            DA.SetDataList(1, Reactions);
            DA.SetDataList(2, internalStresses);
            DA.SetDataList(3, internalStrains);
        } //End of main program

        private void CalculateInternalStrainsAndStresses(List<double> def, List<Point3d> points, double E, List<Line> geometry, out List<double> internalStresses, out List<double> internalStrains)
        {
            //preallocating lists
            internalStresses = new List<double>(geometry.Count);
            internalStrains = new List<double>(geometry.Count);

            foreach (Line line in geometry)
            {
                int index1 = points.IndexOf(line.From);
                int index2 = points.IndexOf(line.To);

                //fetching deformation of point in x and y direction
                double u2 = def[index2 * 2];
                double v2 = def[index2 * 2 + 1];
                double u1 = def[index1 * 2];
                double v1 = def[index1 * 2 + 1];

                //creating new point at deformed coordinates
                double nx1 = points[index1].X + u1;
                double nz1 = points[index1].Z + v1;
                double nx2 = points[index2].X + u2;
                double nz2 = points[index2].Z + v2;

                //calculating dL = (length of deformed line - original length of line)
                double dL = Math.Sqrt(Math.Pow((nx2 - nx1), 2) + Math.Pow((nz2 - nz1), 2)) - line.Length;

                //calculating strain and stress
                internalStrains.Add(dL / line.Length);
                internalStresses.Add(internalStrains[internalStrains.Count - 1] * E);
            }
        }

        private List<double> RestoreTotalDeformationVector(List<double> deformations_red, List<int> bdc_value)
        {
            List<double> def = new List<double>();
            int index = 0;

            for (int i = 0; i < bdc_value.Count; i++)
            {
                if (bdc_value[i] == 0)
                {
                    def.Add(0);
                }
                else
                {
                    def.Add(deformations_red[index]);
                    index += 1;
                }
            }

            return def;
        }

        private List<double> CalculateReactionforces(List<double> def, double[,] K_tot, List<int> bdc_value)
        {
            List<double> R = new List<double>();

            for (int i = 0; i < K_tot.GetLength(1); i++)
            {
                if (bdc_value[i] == 0)
                {
                    double R_temp = 0;
                    for (int j = 0; j < K_tot.GetLength(0); j++)
                    {
                        R_temp += K_tot[i, j] * def[j];
                    }
                    R.Add(Math.Round(R_temp, 2));
                }
                else
                {
                    R.Add(0);
                }
            }
            return R;
        }

        private List<double> Cholesky_Banachiewicz(double[,] m, List<double> load)
        {

            ///commented solution for swapping of rows (superfluous)
            ///List<int> rearrangement = Enumerable.Range(0, m.GetLength(0) - 1).ToList(); //create list of numbers from 0 to rowlength of m - 1
            ///List<double> sol = new List<double>();                                      //preallocate solution
            ///
            /// //swaps rows so that no diagonal element is 0
            ///Tuple<double[,], List<double>, List<int>> T2 = CheckDiagonalAndRearrange(m, load, rearrangement);
            ///double[,] A = T2.Item1;         //modified K-matrix (i.e. rows swapped if needed)
            ///List<double> load1 = T2.Item2;  //rearrangement order (e.g. for list [0,1,2], swap row 0 and 2 -> [2,1,0])
            ///
            /// //remove A and load1 below if uncommenting this section

            double[,] A = m;
            List<double> load1 = load;

            //Cholesky only works for square, symmetric and positive definite matrices. 
            //Square matrix is guaranteed because of how matrix is constructed, but symmetry is checked
            if (IsSymmetric(A))
            {
                //preallocating L and L_transposed matrices
                double[,] L = new double[m.GetLength(0), m.GetLength(1)];
                double[,] L_T = new double[m.GetLength(0), m.GetLength(1)];

                //creation of L and L_transposed matrices
                for (int i = 0; i < L.GetLength(0); i++)
                {
                    for (int j = 0; j <= i; j++)
                    {
                        double L_sum = 0;
                        if (i == j)
                        {
                            for (int k = 0; k < j; k++)
                            {
                                L_sum += L[i, k] * L[i, k];
                            }
                            L[i, i] = Math.Sqrt(A[i, j] - L_sum);
                            L_T[i, i] = L[i, i];
                        }
                        else
                        {
                            for (int k = 0; k < j; k++)
                            {
                                L_sum += L[i, k] * L[j, k];
                            }
                            L[i, j] = (1 / L[j, j]) * (A[i, j] - L_sum);
                            L_T[j, i] = L[i, j];
                        }
                    }
                }
                //Solving L*y=load1 for temporary variable y
                List<double> y = ForwardsSubstitution(load1, L);


                //Solving L^T*x = y for deformations x
                List<double> x = BackwardsSubstitution(load1, L_T, y);

                return x;
            }
            else    //K-matrix is not symmetric
            {
                //throw new RuntimeException("Matrix is not symmetric");
                System.Diagnostics.Debug.WriteLine("Matrix is not symmetric (ERROR!)");
                return null;
            }
        }

        private List<double> ForwardsSubstitution(List<double> load1, double[,] L)
        {
            List<double> y = new List<double>();
            for (int i = 0; i < L.GetLength(1); i++)
            {
                double L_prev = 0;

                for (int j = 0; j < i; j++)
                {
                    L_prev += L[i, j] * y[j];
                }
                y.Add((load1[i] - L_prev) / L[i, i]);
            }
            return y;
        }

        private List<double> BackwardsSubstitution(List<double> load1, double[,] L_T, List<double> y)
        {
            var x = new List<double>(new double[load1.Count]);
            for (int i = L_T.GetLength(1) - 1; i > -1; i--)
            {
                double L_prev = 0;

                for (int j = L_T.GetLength(1) - 1; j > i; j--)
                {
                    L_prev += L_T[i, j] * x[j];
                }

                x[i] = ((y[i] - L_prev) / L_T[i, i]);
            }
            return x;
        }

        private static void CreateReducedGlobalStiffnessMatrix(List<Point3d> points, List<int> bdc_value, double[,] K_tot, List<double> load, out double[,] K_red, out List<double> load_red)
        {
            int dofs_red = points.Count * 2 - (bdc_value.Count - bdc_value.Sum());
            double[,] K_redu = new double[dofs_red, dofs_red];
            List<double> load_redu = new List<double>();
            List<int> bdc_red = new List<int>();
            int m = 0;
            for (int i = 0; i < K_tot.GetLength(0); i++)
            {
                if (bdc_value[i] == 1)
                {
                    int n = 0;
                    for (int j = 0; j < K_tot.GetLength(1); j++)
                    {
                        if (bdc_value[j] == 1)
                        {
                            K_redu[m, n] = K_tot[i, j];
                            n++;
                        }
                    }

                    load_redu.Add(load[i]);

                    m++;
                }
            }
            load_red = load_redu;
            K_red = K_redu;
        }

        private double[,] CreateGlobalStiffnessMatrix(List<Line> geometry, List<Point3d> points, double E, double A)
        {
            int dofs = points.Count * 2;
            double[,] K_tot = new double[dofs, dofs];

            for (int i = 0; i < geometry.Count; i++)
            {
                Line currentLine = geometry[i];
                double mat = (E * A) / (currentLine.Length);
                Point3d p1 = currentLine.From;
                Point3d p2 = currentLine.To;

                double angle = Math.Atan2(p2.Z - p1.Z, p2.X - p1.X);
                double c = Math.Cos(angle);
                double s = Math.Sin(angle);

                double[,] K_elem = new double[,]{
                    { c* c* mat, s*c* mat, -c*c* mat, -s * c* mat},
                    { s* c* mat, s*s* mat, -s * c* mat, -s*s* mat},
                    { -c*c* mat, -s * c* mat, c* c* mat, s*c* mat},
                    { -s* c* mat, -s* s* mat, s* c* mat, s*s* mat} };

                int node1 = points.IndexOf(p1);
                int node2 = points.IndexOf(p2);

                //upper left corner of k-matrix
                K_tot[node1 * 2, node1 * 2] += K_elem[0, 0];
                K_tot[node1 * 2, node1 * 2 + 1] += K_elem[0, 1];
                K_tot[node1 * 2 + 1, node1 * 2] += K_elem[1, 0];
                K_tot[node1 * 2 + 1, node1 * 2 + 1] += K_elem[1, 1];

                //upper right corner of k-matrix
                K_tot[node1 * 2, node2 * 2] += K_elem[0, 2];
                K_tot[node1 * 2, node2 * 2 + 1] += K_elem[0, 3];
                K_tot[node1 * 2 + 1, node2 * 2] += K_elem[1, 2];
                K_tot[node1 * 2 + 1, node2 * 2 + 1] += K_elem[1, 3];

                //lower left corner of k-matrix
                K_tot[node2 * 2, node1 * 2] += K_elem[2, 0];
                K_tot[node2 * 2, node1 * 2 + 1] += K_elem[2, 1];
                K_tot[node2 * 2 + 1, node1 * 2] += K_elem[3, 0];
                K_tot[node2 * 2 + 1, node1 * 2 + 1] += K_elem[3, 1];

                //lower right corner of k-matrix
                K_tot[node2 * 2, node2 * 2] += K_elem[2, 2];
                K_tot[node2 * 2, node2 * 2 + 1] += K_elem[2, 3];
                K_tot[node2 * 2 + 1, node2 * 2] += K_elem[3, 2];
                K_tot[node2 * 2 + 1, node2 * 2 + 1] += K_elem[3, 3];
            }

            return K_tot;
        }

        private List<double> CreateLoadList(List<string> loadtxt, List<Point3d> points)
        {
            List<double> loads = new List<double>();
            List<double> inputLoads = new List<double>();
            List<double> coordlist = new List<double>();

            for (int i = 0; i < loadtxt.Count; i++)
            {
                string coordstr = (loadtxt[i].Split(':')[0]);
                string loadstr = (loadtxt[i].Split(':')[1]);

                string[] coordstr1 = (coordstr.Split(','));
                string[] loadstr1 = (loadstr.Split(','));

                inputLoads.Add(Math.Round(double.Parse(loadstr1[0])));
                inputLoads.Add(Math.Round(double.Parse(loadstr1[1])));
                inputLoads.Add(Math.Round(double.Parse(loadstr1[2])));

                coordlist.Add(Math.Round(double.Parse(coordstr1[0])));
                coordlist.Add(Math.Round(double.Parse(coordstr1[1])));
                coordlist.Add(Math.Round(double.Parse(coordstr1[2])));
            }

            int loadIndex = 0; //bdc_points index

            for (int i = 0; i < points.Count; i++)
            {

                double cptx = Math.Round(points[i].X);
                double cpty = Math.Round(points[i].Y);
                double cptz = Math.Round(points[i].Z);
                bool foundPoint = false;

                for (int j = 0; j < coordlist.Count / 3; j++) if (loadIndex < coordlist.Count)
                    {
                        if (coordlist[j * 3] == cptx && coordlist[j * 3 + 1] == cpty && coordlist[j * 3 + 2] == cptz)
                        {
                            loads.Add(inputLoads[loadIndex]);
                            loads.Add(inputLoads[loadIndex + 2]);
                            loadIndex += 3;
                            foundPoint = true;
                        }
                    }

                if (foundPoint == false)
                {
                    loads.Add(0);
                    loads.Add(0);
                }
            }


            return loads;
        }

        private List<int> CreateBDCList(List<string> bdctxt, List<Point3d> points)
        {
            List<int> bdc_value = new List<int>();
            List<int> bdcs = new List<int>();
            List<double> bdc_points = new List<double>(); //Coordinates relating til bdc_value in for (eg. x y z)
            int bdcIndex = 0; //bdc_points index

            for (int i = 0; i < bdctxt.Count; i++)
            {
                string coordstr = (bdctxt[i].Split(':')[0]);
                string bdcstr = (bdctxt[i].Split(':')[1]);

                string[] coordstr1 = (coordstr.Split(','));
                string[] bdcstr1 = (bdcstr.Split(','));

                bdc_points.Add(double.Parse(coordstr1[0]));
                bdc_points.Add(double.Parse(coordstr1[1]));
                bdc_points.Add(double.Parse(coordstr1[2]));

                bdcs.Add(int.Parse(bdcstr1[0]));
                bdcs.Add(int.Parse(bdcstr1[1]));
                bdcs.Add(int.Parse(bdcstr1[2]));
            }

            for (int i = 0; i < points.Count; i++)
            {

                double cptx = points[i].X;
                double cpty = points[i].Y;
                double cptz = points[i].Z;
                bool foundPoint = false;

                for (int j = 0; j < bdc_points.Count / 3; j++) if (bdcIndex < bdc_points.Count)
                    {
                        if (bdc_points[bdcIndex] == cptx && bdc_points[bdcIndex + 1] == cpty && bdc_points[bdcIndex + 2] == cptz)
                        {
                            bdc_value.Add(bdcs[bdcIndex]);
                            bdc_value.Add(bdcs[bdcIndex + 2]);
                            bdcIndex += 3;
                            foundPoint = true;
                        }
                    }

                if (foundPoint == false)
                {
                    bdc_value.Add(1);
                    bdc_value.Add(1);
                }
            }

            return bdc_value;
        }

        private List<Point3d> CreatePointList(List<Line> geometry)
        {
            List<Point3d> points = new List<Point3d>();

            for (int i = 0; i < geometry.Count; i++) //adds every point unless it already exists in list
            {
                Line l1 = geometry[i];
                if (!points.Contains(l1.From))
                {
                    points.Add(l1.From);
                }
                if (!points.Contains(l1.To))
                {
                    points.Add(l1.To);
                }
            }

            return points;
        }

        //private Tuple<double[,], List<double>, List<int>> CheckDiagonalAndRearrange(double[,] m, List<double> load, List<int> rearrangement)
        //{
        //    double[,] new_m = (double[,])m.Clone();

        //    for (int i = 0; i < m.GetLength(0); i++)
        //    {
        //        if (m[i, i] <= 0)
        //        {
        //            for (int j = i + 1; j < m.GetLength(0); j++)
        //            {
        //                if (m[j, i] > 0)
        //                {
        //                    m = SwapRows(m, i, j);
        //                    SwapList(load, i, j);
        //                    SwapListInt(rearrangement, i, j);
        //                }
        //            }

        //        }
        //    }
        //    var T2 = new Tuple<double[,], List<double>, List<int>>(new_m, load, rearrangement);

        //    return T2;
        //}
        //private double[,] SwapRows(double[,] m, int r1, int r2)
        //{
        //    double[,] new_m = new double[m.GetLength(0), m.GetLength(1)];

        //    for (int i = 0; i < m.GetLength(0); i++)
        //    {
        //        for (int j = 0; j < m.GetLength(1); j++)
        //        {
        //            if (i == r1)
        //            {
        //                new_m[r2, j] = m[i, j];
        //            }
        //            else if (i == r2)
        //            {
        //                new_m[r1, j] = m[i, j];
        //            }
        //            else
        //            {
        //                new_m[i, j] = m[i, j];
        //            }
        //        }
        //    }
        //    return new_m;
        //}

        //private List<double> SwapList(List<double> l, int r1, int r2)
        //{
        //    List<double> new_l = new List<double>(l);

        //    for (int i = 0; i < l.Count; i++)
        //    {
        //        if (i == r1)
        //        {
        //            new_l[r2] = l[i];
        //        }
        //        else if (i == r2)
        //        {
        //            new_l[r1] = l[i];
        //        }
        //        else
        //        {
        //            new_l[i] = l[i];
        //        }
        //    }
        //    return new_l;
        //}

        //private List<int> SwapListInt(List<int> l, int r1, int r2)
        //{
        //    List<int> new_l = new List<int>(l);

        //    for (int i = 0; i < l.Count; i++)
        //    {
        //        if (i == r1)
        //        {
        //            new_l[r2] = l[i];
        //        }
        //        else if (i == r2)
        //        {
        //            new_l[r1] = l[i];
        //        }
        //        else
        //        {
        //            new_l[i] = l[i];
        //        }
        //    }
        //    return new_l;
        //}

        private static bool IsSymmetric(double[,] A)
        {
            int rowCount = A.GetLength(0);
            for (int i = 0; i < rowCount; i++)
            {
                for (int j = 0; j < i; j++)
                {
                    if (A[i, j] != A[j, i])
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        private string PrintStiffnessMatrix(double[,] m)
        {
            string stringMatrix = "";
            for (int i = 0; i < m.GetLength(0); i++)
            {
                for (int j = 0; j < m.GetLength(1); j++)
                {
                    double temp = Math.Round(m[i, j], 2);
                    if (temp == 0)
                    {
                        string txt = String.Format("{0,10:0.0}", ("  0"));
                        stringMatrix += txt;
                    }
                    else
                    {
                        string txt = String.Format("{0,10:0.0}", temp);
                        stringMatrix += txt;
                    }
                }
                stringMatrix += Environment.NewLine + Environment.NewLine + Environment.NewLine;
            }
            return stringMatrix;
        }

        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.TwoDTrussCalculation;    //Setting component icon
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("beae0421-b363-41de-89a2-49cca8210736"); }
        }
    }
}
