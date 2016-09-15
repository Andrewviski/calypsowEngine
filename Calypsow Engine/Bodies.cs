using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Calypsow_Engine.Rigid.World;
using Calypsow_Engine.Utility;
using Calypsow_Engine.Rigid.CollisionResolution;
using Calypsow_Engine.Rigid.CollisionDetection_Fine;
using Calypsow_Engine.Rigid.Forces;

namespace Calypsow_Engine
{
    namespace Particles
    {
        [Serializable]
        public class Particle
        {
            public Vector3 Position;
            public Vector3 Velocity;
            public Vector3 Acceleration;

            public Vector3 ForceAccumlator;
            public readonly float Damping = 0.7f;
            public float InversedMass;
            public Particle()
            {
                this.InversedMass = 0;
                this.Position = new Vector3();
                this.Velocity = new Vector3();
                this.Acceleration = new Vector3();
                this.ForceAccumlator = new Vector3();
            }
            public Particle(float m, Vector3 p, Vector3 v, Vector3 a)
            {
                this.InversedMass = 1.0f / m;
                this.Position = p;
                this.Velocity = v;
                this.Acceleration = a;
                this.ForceAccumlator = new Vector3();
            }

            public float GetMass()
            {
                return (float)1.0f / InversedMass;
            }
            public float GetInversedMass()
            {
                return InversedMass;
            }
            public Vector3 GetAcceleration()
            {
                return Acceleration;
            }
            public Vector3 GetVelocity()
            {
                return this.Velocity;
            }
            public Vector3 GetPosition()
            {
                return Position;
            }
            public void ClearForceAccumlator()
            {
                ForceAccumlator = Vector3.Zero;
            }
            public void Integrate(float dt)
            {
                //update linear position
                this.Position = this.Velocity * dt;
                //workout the acceleration from the forces               
                this.Acceleration = this.ForceAccumlator * this.InversedMass;
                //update linear velocity for acceleration
                this.Velocity = this.Acceleration * dt;
                //impose drag
                //Velocity *= Damping;
                //clear the accumelator
                ClearForceAccumlator();
            }
            public void AddForceToAccumlator(Vector3 f)
            {
                ForceAccumlator += f;
            }
        }
    }
    namespace Rigid
    {
        public interface iExplosive
        {
            void Explode();
        }
        [Serializable]
        public abstract class RigidBody
        {
            //linear
            protected float InversedMass;
            public Vector3 Position;
            public Vector3 Velocity;
            public Vector3 Acceleration;
            public Vector3 ForceAccumlator;
            public readonly float LinearDamping = 0.95f;
            //rotational
            public float BaseMomentOfInertia;
            public Matrix InverseInertiaTensorWorld;
            public float Orientation = 0;
            public float TorqueAccumlator = 0;
            public float AngularVelocity = 0;
            public float AngularAcceleration = 0;
            public readonly float AngularDamping = 0.80f;
            //extra
            public Vector3 LastFrameAcceleration;
            public Vector3 LastFrameVelocity;
            public Vector3 xAxis = new Vector3(1, 0, 0);
            public Vector3 yAxis = new Vector3(0, 1, 0);

            public int Material;
            //for friction stuff
            public bool IsAwake;
            public bool CanSleep;
            public double Motion;
            public float SleepEpsilon = 3.9f;
            public bool canDelete = false;
            public bool hitGround = false;

            //jointedwith
            public RigidBody[] jointedWith;

            public RigidBody(Vector3 p, Vector3 v, Vector3 a)
            {
                this.Acceleration = a;
                this.Velocity = v;
                this.Position = p;
                ForceAccumlator = new Vector3();
                TorqueAccumlator = 0;
                IsAwake = true;
                CanSleep = false;
                this.Motion = this.SleepEpsilon * 10.0f;
            }
            public Vector3 GetLastFrameVelocity()
            {
                return LastFrameVelocity;
            }
            public Vector3 GetLastFrameAcceleration()
            {
                return LastFrameAcceleration;
            }
            public void SetLastFrameAccelaration(Vector3 a)
            {
                LastFrameAcceleration = a;
            }
            public bool GetisAwake()
            {
                return this.IsAwake;
            }
            public bool GetcanSleep()
            {
                return this.CanSleep;
            }
            public Vector3 GetPosition()
            {
                return this.Position;
            }
            public Vector3 GetVelocity()
            {
                return this.Velocity;
            }
            public Vector3 GetAcceleration()
            {
                return this.Acceleration;
            }
            public void SetPosition(Vector3 pos)
            {
                this.Position = pos;
            }
            public void SetVelocity(Vector3 vel)
            {
                this.Velocity = vel;
            }
            public void SetAcceleration(Vector3 acc)
            {
                this.Acceleration = acc;
            }
            public abstract float GetInverseMass();

            public abstract float GetMass();

            public void SetInverseMass(float m)
            {
                this.InversedMass = m;
            }
            public float GetRotation()
            {
                return this.AngularVelocity;
            }
            public void SetRotation(float angR)
            {
                this.AngularVelocity = angR;
            }
            public Matrix GetInverseInertiaTensorWorld()
            {
                return InverseInertiaTensorWorld;
            }
            public float GetOrientation()
            {
                return Orientation;
            }
            public void SetOrientation(float o)
            {
                Orientation = o;
                xAxis = Matrix2.M_V(new Vector3(1, 0, 0), Orientation);
                yAxis = Matrix2.M_V(new Vector3(0, 1, 0), Orientation);
                xAxis.Normalize();
                yAxis.Normalize();
            }
            public void SetCanSleep(bool canSleepe)
            {
                this.CanSleep = canSleepe;

                if (!this.CanSleep && !this.IsAwake) SetAwake(true);
            }
            public void SetAwake(bool awake)
            {
                if (awake)
                {
                    this.IsAwake = true;
                    this.Motion = this.SleepEpsilon * 2.0f;
                }
                else
                {
                    this.IsAwake = false;
                    this.Velocity = Vector3.Zero;
                    this.AngularVelocity = 0;
                }
            }
            public void AddForce(Vector3 force)
            {
                ForceAccumlator += force;
                IsAwake = true;
                CanSleep = false;
            }
            public void AddTorque(Vector3 force, Vector3 begin)
            {
                Vector3 temp = Vector3.Cross(force, begin);
                TorqueAccumlator += temp.Z;
                IsAwake = true;
                CanSleep = false;
            }
            public abstract bool Collide(RigidBody other, ref CollisionData data);
            public abstract void Update(float duration);
            public abstract Vector3 getRandomPoint();
            public abstract bool IsIn(int x, int y);
            public abstract bool canJoint();
            public abstract void removeJoint(int index);
            public float GetDiameter()
            {
                if (this is Box)
                {
                    return (this as Box).HalfSizeWidth;
                }
                else
                {
                    return (this as Circle).Radius;
                }
            }
            public bool isBox()
            {
                return (this is Box);
            }
            public bool isCircle()
            {
                return (this is Circle);
            }
        }
        [Serializable]
        public class Box : RigidBody
        {
            [Serializable]
            public static class Direction
            {
                public static int RIGHT = 0;
                public static int DOWN = 1;
                public static int LEFT = 2;
                public static int UP = 3;
            }

            public ContainableItem ContainedBody;
            public CollisionBox collisionbox;
            public float HalfSizeWidth;
            public float HalfSizeheight;

            //joints
            public bool[] joints = new bool[4];
            public List<Circle> JointedCircles = new List<Circle>(4);
            public Box(Vector3 p, Vector3 v, Vector3 a, float hwidth, float hheight, int mat, RigidBody containedbody = null)
                : base(p, v, a)
            {
                this.jointedWith = new RigidBody[4];
                this.Material = mat;
                this.HalfSizeWidth = hwidth;
                this.HalfSizeheight = hheight;
                float mass = (HalfSizeWidth * 2 * HalfSizeheight * 2 * 4 * Utility.Tables.GetDensity(this))*10 ;
                this.InversedMass = 1 / mass;
                this.BaseMomentOfInertia = mass * Math.Max(HalfSizeheight, HalfSizeWidth) * Math.Max(HalfSizeheight, HalfSizeWidth) * 4 / 12f;
                this.InverseInertiaTensorWorld = new Matrix();
                InverseInertiaTensorWorld.M11 = InverseInertiaTensorWorld.M22 = InverseInertiaTensorWorld.M33 = (float)(1.0f / BaseMomentOfInertia);
                InverseInertiaTensorWorld.M44 = 1;
                ContainedBody = null;
                collisionbox = new CollisionBox(new Vector3(HalfSizeWidth, HalfSizeheight, 0));
                collisionbox.body = this;
            }
            public override void removeJoint(int index)
            {
                this.joints[index] = false;
            }
            public override bool canJoint()
            {
                return (!joints[0] || !joints[1] || !joints[2] || !joints[3]);
            }
            public float GethWidth()
            {
                return HalfSizeWidth;
            }
            public float GethHeight()
            {
                return HalfSizeheight;
            }
            public void SethWidth(float w)
            {
                HalfSizeWidth = w;
            }
            public void SethHeight(float h)
            {
                HalfSizeheight = h;
            }
            public bool Ispinned() 
            {
                return (ContainedBody is Pin);
            }
            public override void Update(float duration)
            {
                this.hitGround = false;
                if (!this.Empty())
                {
                    if (ContainedBody is Engine)
                    {
                        if (((Engine)ContainedBody).On)
                        {
                            foreach (RigidBody item in JointedCircles)
                            {
                                if (item != null)
                                {
                                    item.TorqueAccumlator += ((Engine)ContainedBody).Torque;
                                }
                            }
                        }

                    }
                    UpdateContainedBody();
                }
                if (!this.IsAwake)
                {
                    ForceAccumlator = Vector3.Zero;
                    TorqueAccumlator = 0f;
                    return;
                }

                LastFrameAcceleration = Acceleration;
                LastFrameAcceleration += ForceAccumlator * GetInverseMass();

                AngularAcceleration = TorqueAccumlator / BaseMomentOfInertia;
                LastFrameVelocity = Velocity;
                Velocity += LastFrameAcceleration * duration;
                AngularVelocity += AngularAcceleration * duration;

                Velocity *= (float)Math.Pow(LinearDamping, duration); //dumping
                AngularVelocity *= (float)Math.Pow(AngularDamping, duration);

                Position += Velocity * duration;
                Orientation += AngularVelocity * duration;

                xAxis = Matrix2.M_V(new Vector3(1, 0, 0), Orientation);
                yAxis = Matrix2.M_V(new Vector3(0, 1, 0), Orientation);
                xAxis.Normalize();
                yAxis.Normalize();

                if (this.CanSleep)
                {
                    float currentMotion = Vector3.Dot(Velocity, Velocity) + 0.1f * AngularVelocity * AngularVelocity;

                    double bias = Math.Pow((double)0.5, (double)duration);
                    Motion = bias * Motion + (1 - bias) * currentMotion;

                    if (Motion < SleepEpsilon) SetAwake(false);
                    else if (Motion > 10 * SleepEpsilon) Motion = 10 * SleepEpsilon;
                }
                this.SleepEpsilon = 3.0f;
                ForceAccumlator = Vector3.Zero;
                TorqueAccumlator = 0f;
            }
            public override bool Collide(RigidBody other, ref CollisionData data)
            {
                if (other is Circle)
                {
                    Circle temp = other as Circle;
                    if (CollisionDetector.BoxAndCircle(this.collisionbox, temp.collisioncircle, ref data))
                    {
                        return true;
                    }
                }
                else if (other is Box)
                {
                    Box temp = other as Box;
                    //if (temp.hitGround || this.hitGround)
                    //{
                    //    temp.SleepEpsilon = 5.0f;
                    //    this.SleepEpsilon = 5.0f;
                    //    temp.hitGround = true;
                    //    this.hitGround = true;
                    //}
                    if (CollisionDetector.BoxAndBox(this.collisionbox, temp.collisionbox, ref data))
                    {
                        return true;
                    }
                }
                return false;
            }
            public override Vector3 getRandomPoint()
            {
                Random rnd = new Random();
                double rnd1 = rnd.NextDouble();
                double rnd2 = rnd.NextDouble();
                double x;
                double y;

                //x
                if (rnd1 > 0.5)
                {
                    x = rnd1 * HalfSizeWidth;
                }
                else
                {
                    x = (1 - rnd1) * HalfSizeWidth;
                }

                //y
                if (rnd2 > 0.5)
                {
                    y = rnd2 * HalfSizeheight;
                }
                else
                {
                    y = (1 - rnd2) * HalfSizeheight;
                }

                return new Vector3((float)x, (float)y, 0.0f);
            }
            public override bool IsIn(int x, int y)
            {
                Vector3[] points = new Vector3[]{ 
                     Position + Vector3.Multiply(this.xAxis,this.HalfSizeWidth) + Vector3.Multiply(this.yAxis,this.HalfSizeheight) ,
                     Position + Vector3.Multiply(this.xAxis,-this.HalfSizeWidth) + Vector3.Multiply(this.yAxis,this.HalfSizeheight) ,
                     Position + Vector3.Multiply(this.xAxis,this.HalfSizeWidth) + Vector3.Multiply(this.yAxis,-this.HalfSizeheight) ,
                     Position + Vector3.Multiply(this.xAxis,-this.HalfSizeWidth) + Vector3.Multiply(this.yAxis,-this.HalfSizeheight) 
                    };
                if (((x > points[0].X && x < points[1].X) || (x > points[1].X && x < points[0].X)) && ((y > points[0].Y && y < points[3].Y) || (y > points[3].Y && y < points[0].Y)))
                {
                    return true;
                }
                return false;
            }
            //contained body
            public bool Empty()
            {
                return ContainedBody == null;
            }
            public void ConatainBody(ContainableItem rg)
            {
                ContainedBody = rg;
            }
            public void UpdateContainedBody()
            {
                if (!Empty())
                {
                    ContainedBody.SetPosition(this.Position.X, this.Position.Y);
                }
            }
            public override float GetMass()
            {
                return (Empty()) ? 1.0f / this.InversedMass : 1.0f / this.InversedMass + 1.0f / ContainedBody.InversedMass;
            }
            public override float GetInverseMass()
            {
                return (Empty()) ? this.InversedMass : 1.0f / (1.0f / this.InversedMass + 1.0f / ContainedBody.InversedMass);
            }
        }
        [Serializable]
        public class Circle : RigidBody
        {
            public bool joint = false;
            public float InitTorque;
            public float InitRotation;
            public float Radius;
            public CollisionCircle collisioncircle;
            public Circle(Vector3 p, Vector3 v, Vector3 a, int mat, float r)
                : base(p, v, a)
            {
                this.jointedWith = new RigidBody[1];
                this.Material = mat;
                InitTorque = 0;
                InitRotation = 0;
                this.Radius = r;
                float mass = 3.14f * 4 / 3 * Radius * Radius * Radius * Utility.Tables.GetDensity(this);
                this.InversedMass = 1 / mass;
                this.BaseMomentOfInertia = 0.5f * mass * Radius * Radius;
                InverseInertiaTensorWorld = new Matrix();
                InverseInertiaTensorWorld.M11 = InverseInertiaTensorWorld.M22 = InverseInertiaTensorWorld.M33 = (float)1 / BaseMomentOfInertia;
                InverseInertiaTensorWorld.M44 = 1;
                collisioncircle = new CollisionCircle(this.Radius);
                collisioncircle.body = this;
            }
            public override void removeJoint(int index)
            {
                this.joint = false;
            }
            public override bool canJoint()
            {
                return !joint;
            }
            public void SetRadius(float d)
            {
                this.Radius = d;
            }
            public float GetRadius()
            {
                return this.Radius;
            }
            public void setInitTorque(float t)
            {
                InitTorque = t;
            }
            public void setInitRotation(float r)
            {
                AngularVelocity = InitRotation = r;
            }
            public bool intersect(Circle C)
            {
                if (Vector3.Distance(this.Position, C.Position) < (this.Radius + C.Radius))
                {
                    return false;
                }
                return true;
            }
            public override void Update(float duration)
            {

                if (!this.IsAwake)
                {
                    ForceAccumlator = Vector3.Zero;
                    TorqueAccumlator = 0f;
                    return;
                }


                LastFrameAcceleration = Acceleration;
                LastFrameAcceleration += ForceAccumlator * GetInverseMass();
                AngularAcceleration = TorqueAccumlator / BaseMomentOfInertia;
                LastFrameVelocity = Velocity;
                Velocity += LastFrameAcceleration * duration;
                AngularVelocity += AngularAcceleration * duration;

                Velocity *= (float)Math.Pow(LinearDamping, duration); //dumping
                AngularVelocity *= (float)Math.Pow(AngularDamping, duration);

                Position += Velocity * duration;
                Orientation += AngularVelocity * duration;
                if (this.hitGround)
                {
                    Vector3 dx = Velocity * duration;
                    float distance = dx.X;
                    distance /= this.GetDiameter();
                    this.Orientation += distance;
                }
                xAxis = Matrix2.M_V(new Vector3(1, 0, 0), Orientation);
                yAxis = Matrix2.M_V(new Vector3(0, 1, 0), Orientation);
                xAxis.Normalize();
                yAxis.Normalize();

                if (this.CanSleep)
                {
                    float currentMotion = Vector3.Dot(Velocity, Velocity) + 0.1f * AngularVelocity * AngularVelocity;
                    double bias = Math.Pow((double)0.5, (double)duration);
                    Motion = bias * Motion + (1 - bias) * currentMotion;

                    if (Motion < SleepEpsilon) SetAwake(false);
                    else if (Motion > 10 * SleepEpsilon) Motion = 10 * SleepEpsilon;
                }

                ForceAccumlator = Vector3.Zero;
                if ((Math.Abs((double)AngularVelocity - InitRotation) > 1))
                    TorqueAccumlator = InitTorque;
                else
                    TorqueAccumlator = 0;
                this.hitGround = false;
            }
            public override bool Collide(RigidBody other, ref CollisionData data)
            {
                if (other is Circle)
                {
                    Circle temp = other as Circle;
                    if (CollisionDetector.CircleAndCircle(this.collisioncircle, temp.collisioncircle, ref data))
                    {
                        return true;
                    }
                }
                else if (other is Box)
                {
                    Box temp = other as Box;
                    if (CollisionDetector.BoxAndCircle(temp.collisionbox, this.collisioncircle, ref data))
                    {
                        return true;
                    }
                }
                return false;
            }
            public override Vector3 getRandomPoint()
            {
                Random rnd = new Random();
                double rnd1 = rnd.NextDouble();
                double rnd2 = rnd.NextDouble();
                double x;
                double y;

                //x
                if (rnd1 > 0.5)
                {
                    x = rnd1 * Radius;
                }
                else
                {
                    x = (1 - rnd1) * Radius;
                }

                //y
                if (rnd2 > 0.5)
                {
                    y = rnd2 * Radius;
                }
                else
                {
                    y = (1 - rnd2) * Radius;
                }

                return new Vector3((float)x, (float)y, 0.0f);
            }
            public override bool IsIn(int x, int y)
            {
                return (Vector3.Distance(new Vector3(x, y, 0), Position) > Radius) ? false : true;
            }
            public override float GetMass()
            {
                return 1.0f / InversedMass;
            }
            public override float GetInverseMass()
            {
                return this.InversedMass;
            }
        }
        [Serializable]
        public class ExplosiveBox : Box, iExplosive
        {
            //world
            public World.World explosionWorld;
            //forces
            public float implosionForce;
            public float waveForce;
            public float timepassed;
            public double waveSpeed;
            public double waveAcceleration;
            double implosionMin = 50;
            double implosionMax = 700;
            double implosionDuration = 1.25;
            double explosionDuration = 25;
            double waveWidth = 50;

            //exploded
            public bool exploded;
            public float explodetimer = -1;

            public override void Update(float duration)
            {
                base.Update(duration);
                if (exploded)
                {
                    timepassed += duration;
                    if (timepassed > this.implosionDuration)
                    {
                        canDelete = true;
                    }
                }

                if (explodetimer > 0)
                {
                    explodetimer -= duration;
                    if (explodetimer <= 0)
                    {
                        this.Explode();
                    }
                }


            }
            public ExplosiveBox(Vector3 p, Vector3 v, Vector3 a, float hwidth, float hheight, int mat, float ImplosionForce, float WaveForce, double wavespeed, double waveacceleration, World.World myworld, double implosionMin = 50, double implosionMax = 700, double implosionDuration = 1.25, double explosionDuration = 25, double waveWidth = 50)
                : base(p, v, a, hwidth, hheight, mat, null)
            {
                this.explosionWorld = myworld;
                this.implosionForce = ImplosionForce;
                this.waveForce = WaveForce;
                this.waveSpeed = wavespeed;
                this.waveAcceleration = waveacceleration;
                this.implosionMin = implosionMin;
                this.implosionMax = implosionMax;
                this.implosionDuration = implosionDuration;
                this.explosionDuration = explosionDuration;
                this.waveWidth = waveWidth;
                timepassed = 0;
                exploded = false;

            }
            public void Explode()
            {
                if (!exploded)
                {
                    this.explosionWorld.removeJointOfBody(this);
                    exploded = true;
                    foreach (RigidBody item in this.explosionWorld.RigidRegistry)
                    {
                        RigidForceGenerator r = new RigidExplosion(this.Position, this.implosionMin, this.implosionMax, this.implosionDuration, this.implosionForce, this.waveSpeed, this.explosionDuration, this.waveWidth, this.waveForce, this.waveAcceleration, item);
                        this.explosionWorld.AddForce(item, r);
                    }
                }
            }
        }
        [Serializable]
        public class ExplosiveCircle : Circle, iExplosive
        {
            //world
            public World.World explosionWorld;
            //forces
            public float implosionForce;
            public float waveForce;
            public float timepassed;
            public double waveSpeed;
            public double waveAcceleration;
            double implosionMin = 50;
            double implosionMax = 700;
            double implosionDuration = 1.25;
            double explosionDuration = 25;
            double waveWidth = 50;

            //exploded
            public bool exploded;
            public float explodetimer = -1;

            public override void Update(float duration)
            {
                base.Update(duration);
                if (exploded)
                {
                    timepassed += duration;
                    if (timepassed > this.implosionDuration)
                    {
                        canDelete = true;
                    }
                }
                if (explodetimer > 0)
                {
                    explodetimer -= duration;
                    if (explodetimer <= 0)
                    {
                        this.Explode();
                    }
                }

            }
            public ExplosiveCircle(Vector3 p, Vector3 v, Vector3 a, float radius, int mat, float ImplosionForce, float WaveForce, double wavespeed, double waveacceleration, World.World myworld, double implosionMin = 50, double implosionMax = 700, double implosionDuration = 1.25, double explosionDuration = 25, double waveWidth = 50)
                : base(p, v, a, mat, radius)
            {
                this.explosionWorld = myworld;
                this.implosionForce = ImplosionForce;
                this.waveForce = WaveForce;
                this.waveSpeed = wavespeed;
                this.waveAcceleration = waveacceleration;
                this.implosionMin = implosionMin;
                this.implosionMax = implosionMax;
                this.implosionDuration = implosionDuration;
                this.explosionDuration = explosionDuration;
                this.waveWidth = waveWidth;
                timepassed = 0;
                exploded = false;

            }

            public void Explode()
            {
                if (!exploded)
                {
                    exploded = true;
                    foreach (RigidBody item in this.explosionWorld.RigidRegistry)
                    {
                        this.explosionWorld.removeJointOfBody(this);
                        RigidForceGenerator r = new RigidExplosion(this.Position, this.implosionMin, this.implosionMax, this.implosionDuration, this.implosionForce, this.waveSpeed, this.explosionDuration, this.waveWidth, this.waveForce, this.waveAcceleration, item);
                        this.explosionWorld.AddForce(item, r);
                    }
                }
            }
        }
        [Serializable]
        public abstract class ContainableItem
        {
            public float InversedMass;
            public Vector3 Position;

            public float GetMass()
            {
                return 1.0f / this.InversedMass;
            }
            public float GetInversedMass()
            {
                return this.InversedMass;
            }
            public Vector3 GetPosition()
            {
                return this.Position;
            }
            public void SetPosition(float x, float y)
            {
                this.Position.X = x;
                this.Position.Y = y;
            }
            public bool IsEngine()
            {
                return (this is Engine);
            }
            public bool IsPig()
            {
                return (this is Pig);
            }
        }
        [Serializable]
        public class Engine : ContainableItem
        {
            [Serializable]
            public static class EngineType
            {
                public static int X1plus = 0;
                public static int X2plus = 1;
                public static int X3plus = 2;
                public static int X1min = 3;
                public static int X2min = 4;
                public static int X3min = 5;
            }
            public int type;
            public float Torque;
            public bool On;
            public Engine(int t)
            {
                this.type = t;
                On = true;
                this.Position = new Vector3(0, 0, 0);
                this.Torque = Utility.Tables.EngineTable[type];
                if (type < 3)
                {
                    this.InversedMass = 1.0f / (type * 5.0f * 1000.0f);
                }
                else
                {
                    this.InversedMass = 1.0f / (type * 5.0f * 1000.0f);
                }
            }
        }
        [Serializable]
        public class Pig : ContainableItem
        {
            public Pig()
            {
                this.InversedMass = 1.0f / 100000.0f;
                Position = new Vector3();
            }
        }
        [Serializable]
        public class Pin : ContainableItem
        {
            public Pin()
            {
                this.InversedMass = 0;
                Position = new Vector3();
            }
        }
    }
}
