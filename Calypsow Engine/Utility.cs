using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Calypsow_Engine.Rigid;
using Microsoft.Xna.Framework;


namespace Calypsow_Engine
{
    namespace Utility
    {
        [Serializable]
        public static class Tables
        {
            //Rubber = 0
            //Wood = 1
            //Glass = 2
            //Metal = 3
            //Concrete = 4
            //Ice = 5
            [Serializable]
            public static class Materials
            {
                public static int Rubber = 0;
                public static int Wood = 1;
                public static int Glass = 2;
                public static int Metal = 3;
                public static int Concrete = 4;
                public static int Ice = 5;
                //also wood
                public static int Explosive = 5;
            }

            public static float[][] FrictionTable = new float[10][];
            public static float[][] RestitutionTable = new float[10][];
            public static float[] DensityTable = new float[10];
            public static float[] EngineTable = new float[6];
            static Tables()
            {
                //...............................................friction table
                FrictionTable[0] = new float[6];
                FrictionTable[1] = new float[6];
                FrictionTable[2] = new float[6];
                FrictionTable[3] = new float[6];
                FrictionTable[4] = new float[6];
                FrictionTable[5] = new float[6];
                FrictionTable[0][0] = 0.9f;
                FrictionTable[0][1] = FrictionTable[1][0] = 0.6f;
                FrictionTable[0][2] = FrictionTable[2][0] = 0.8f;
                FrictionTable[0][3] = FrictionTable[3][0] = 0.4f;
                FrictionTable[0][4] = FrictionTable[4][0] = 0.85f;
                FrictionTable[0][5] = FrictionTable[5][0] = 0.1f;
                FrictionTable[1][1] = 0.75f;
                FrictionTable[1][2] = FrictionTable[2][1] = 0.56f;
                FrictionTable[1][3] = FrictionTable[3][1] = 0.35f;
                FrictionTable[1][4] = FrictionTable[4][1] = 0.62f;
                FrictionTable[1][5] = FrictionTable[5][1] = 0.1f;
                FrictionTable[2][2] = 0.3f;
                FrictionTable[2][3] = FrictionTable[3][2] = 0.4f;
                FrictionTable[2][4] = FrictionTable[4][2] = 0.7f;
                FrictionTable[2][5] = FrictionTable[5][2] = 0.1f;
                FrictionTable[3][3] = 0.6f;
                FrictionTable[3][4] = FrictionTable[4][3] = 0.7f;
                FrictionTable[3][5] = FrictionTable[5][3] = 0.1f;
                FrictionTable[4][4] = 0.8f;
                FrictionTable[4][5] = FrictionTable[5][4] = 0.1f;
                FrictionTable[5][5] = 0.1f;  //........ice is 0.1

                //...............................................restitution table
                RestitutionTable[0] = new float[6];
                RestitutionTable[1] = new float[6];
                RestitutionTable[2] = new float[6];
                RestitutionTable[3] = new float[6];
                RestitutionTable[4] = new float[6];
                RestitutionTable[5] = new float[6];
                RestitutionTable[0][0] = 0.9f;
                RestitutionTable[0][1] = RestitutionTable[1][0] = 0.7f;
                RestitutionTable[0][2] = RestitutionTable[2][0] = 0.7f;
                RestitutionTable[0][3] = RestitutionTable[3][0] = 0.8f;
                RestitutionTable[0][4] = RestitutionTable[4][0] = 0.8f;
                RestitutionTable[0][5] = RestitutionTable[5][0] = 0.2f;
                RestitutionTable[1][1] = 0.5f;
                RestitutionTable[1][2] = RestitutionTable[2][1] = 0.65f;
                RestitutionTable[1][3] = RestitutionTable[3][1] = 0.55f;
                RestitutionTable[1][4] = RestitutionTable[4][1] = 0.4f;
                RestitutionTable[1][5] = RestitutionTable[5][1] = 0.2f;
                RestitutionTable[2][2] = 0.8f;
                RestitutionTable[2][3] = RestitutionTable[3][2] = 0.7f;
                RestitutionTable[2][4] = RestitutionTable[4][2] = 0.6f;
                RestitutionTable[2][5] = RestitutionTable[5][2] = 0.2f;
                RestitutionTable[3][3] = 0.75f;
                RestitutionTable[3][4] = RestitutionTable[4][3] = 0.6f;
                RestitutionTable[3][5] = RestitutionTable[5][3] = 0.2f;
                RestitutionTable[4][4] = 0.6f;
                RestitutionTable[4][5] = RestitutionTable[5][4] = 0.2f;
                RestitutionTable[5][5] = 0.1f;  //.....same as ice

                // .................................................... Density Table
                DensityTable[0] = 1.2f;
                DensityTable[1] = 2.5f;
                DensityTable[2] = 0.7f;
                DensityTable[3] = 4.8f;
                DensityTable[4] = 2.9f;
                DensityTable[5] = 2.2f;


                // .................................................... Engine Table
                EngineTable[Engine.EngineType.X1plus] = 5000000.0f;
                EngineTable[Engine.EngineType.X2plus] = 10000000.0f;
                EngineTable[Engine.EngineType.X3plus] = 30000000.0f;
                EngineTable[Engine.EngineType.X1min] = -5000000.0f;
                EngineTable[Engine.EngineType.X2min] = -10000000.0f;
                EngineTable[Engine.EngineType.X3min] = -30000000.0f;

            }
            public static float GetRestitution(RigidBody rg1, RigidBody rg2)
            {
                return RestitutionTable[rg1.Material][rg2.Material];
            }
            public static float GetFriction(RigidBody rg1, RigidBody rg2)
            {
                return FrictionTable[rg1.Material][rg2.Material];
            }
            public static float GetDensity(RigidBody rg)
            {
                return DensityTable[rg.Material];
            }
            //ground stuff
            public static float GetRestitution(RigidBody rg)
            {
                //Rubber = 0
                //Wood = 1
                //Glass = 2
                //Metal = 3
                //Concrete = 4
                //Ice = 5
                switch (rg.Material)
                {
                    case 0:
                        return 0.85f;
                    case 1:
                        return 0.3f;
                    case 2:
                        return 0.6f;
                    case 3:
                        return 0.4f;
                    case 4:
                        return 0.2f;
                    case 5:
                        return 0.1f;
                    default:
                        return 0.3f;
                }
            }
            public static float GetFriction(RigidBody rg)
            {
                //Rubber = 0
                //Wood = 1
                //Glass = 2
                //Metal = 3
                //Concrete = 4
                //Ice = 5
                switch (rg.Material)
                {
                    case 0:
                        return 1.3f;
                    case 1:
                        return 0.7f;
                    case 2:
                        return 0.4f;
                    case 3:
                        return 0.5f;
                    case 4:
                        return 0.9f;
                    case 5:
                        return 0.8f;
                    default:
                        return 
                            0.9f;
                }
            }
        }
        [Serializable]
        public class Vector
        {
            #region Values
            public double X { set; get; }
            public double Y { set; get; }
            private const double Tolerance = 0.0001d;
            #endregion
            #region Constructers
            public Vector()
            {
                X = 0;
                Y = 0;
            }
            public Vector(double x, double y)
            {
                X = x;
                Y = y;
            }
            public Vector(double orientation, double magnitude, int eliasconstructer)
            {
                X = magnitude * Math.Cos(orientation);
                Y = magnitude * Math.Sin(orientation);
            }
            #endregion
            #region Normal-functions
            public void Clear()
            {
                this.X = 0;
                this.Y = 0;
            }
            public void Invert()
            {
                this.X = -this.X;
                this.Y = -this.Y;
            }
            public double Magnitude()
            {
                return Math.Sqrt(X * X + Y * Y);
            }
            public double SquareMagnitude()
            {
                return X * X + Y * Y;
            }
            public void Normalize()
            {
                double l = Magnitude();
                if (l > 0)
                {
                    this.X = this.X * (1.0d / l);
                    this.Y = this.Y * (1.0d / l);
                }
            }
            public void AddScaledVector(Vector v, double s)
            {
                this.X += v.X * s;
                this.Y += v.Y * s;
            }
            public Vector ComponentProduct(Vector v)
            {
                return new Vector(this.X * v.X, this.X * v.Y);
            }
            public void ComponentProductUpdate(Vector v)
            {
                this.X *= v.X;
                this.Y *= v.Y;

            }
            public double ScalarProduct(Vector v)
            {
                return this.X * v.X + this.Y * v.Y;
            }
            public static double TwoDimantionVectorProduct(Vector v, Vector u)
            {
                return (v.X * u.Y) - (v.Y * u.X);
            }
            public static Vector ZonlyVectorProduct(Vector v, double z)
            {
                return -1 * ZonlyVectorProduct(z, v);
            }
            public static Vector ZonlyVectorProduct(double z, Vector v)
            {
                return new Vector(-v.Y * z, v.X * z);
            }
            #endregion
            #region Operators-Overloads
            //multi by a scalar
            public static Vector operator *(Vector v, double s)
            {
                return new Vector(v.X * s, v.Y * s);
            }
            public static Vector operator *(double s, Vector v)
            {
                return v * s;
            }
            //multi by a vector (Scalar Product)
            public static double operator *(Vector v, Vector u)
            {
                return ((v.X * u.X) + (v.Y * u.Y));
            }
            //addition
            public static Vector operator +(Vector v, Vector u)
            {
                return new Vector(v.X + u.X, v.Y + u.Y);
            }
            //subtraction
            public static Vector operator -(Vector v, Vector u)
            {
                return new Vector(v.X - u.X, v.Y - u.Y);
            }
            //vector product
            public static double operator %(Vector v, Vector u)
            {
                return (v.X * u.Y) - (v.Y * u.X);
            }

            #endregion
        }
        [Serializable]
        static class Matrix2
        {
            public static Vector3 Mul_Matrix_vector(Matrix a, Vector3 b)
            {
                return new Vector3(0, 0, a.M33 * b.Z);
            }
            public static Vector3 M_V(Vector3 x, float angle)
            {
                Vector3 temp = new Vector3(0, 0, 0);

                temp.X = (float)Math.Cos(angle) * x.X - (float)Math.Sin(angle) * x.Y;
                temp.Y = (float)Math.Sin(angle) * x.X + (float)Math.Cos(angle) * x.Y;
                temp.Z = 0;

                return temp;
            }
            public static Vector3 transform(Matrix a, Vector3 v)
            {
                return new Vector3(a.M11 * v.X + a.M12 * v.Y, a.M21 * v.X + a.M22 * v.Y, a.M33 * v.Z);
            }
            public static Vector3 transformTranspose(Matrix aa, Vector3 v)
            {
                Matrix a = Matrix.Transpose(aa);
                return new Vector3(a.M11 * v.X + a.M12 * v.Y, a.M21 * v.X + a.M22 * v.Y, a.M33 * v.Z);
            }
            public static Matrix setSkewSymmetric(Vector3 v)
            {
                return new Matrix(0, -v.Z, v.Y, 0, v.Z, 0, -v.X, 0, -v.Y, v.X, 0, 0, 0, 0, 0, 1);
            }
        }
    }
}
