using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Calypsow_Engine.Particles.Forces;
using Calypsow_Engine.Particles.CollisionResolution;
using Calypsow_Engine.Rigid.Forces;
using Calypsow_Engine.Rigid.CollisionResolution;
using Calypsow_Engine.Rigid.CollisionDetection_Fine;
using Microsoft.Xna.Framework;
namespace Calypsow_Engine
{
    namespace Particles
    {
        namespace World
        {
            public class ParticleWorld // not finished !!!
            {
                bool CalculateIterations;
                //particles list
                List<Particle> ParticleRegistry;
                //forces list
                ParticleForceRegistry ForceRegistry;
                //contactgenerator list
                List<ParticleContactGenerator> ContactRegistration;
                //contacts list
                List<ParticleContact> Contacts;
                //contact resolver
                ParticleContactResolver Resolver;
                //number of maximum contacts in the frame
                int MaxContacts;

                //explode threshold
                public static double explodeThreshold;
                public ParticleWorld(int maxcontacts, int iterations = 0)
                {
                    this.MaxContacts = maxcontacts;
                    Resolver = new ParticleContactResolver(iterations);
                    Contacts = new List<ParticleContact>();
                    ContactRegistration = new List<ParticleContactGenerator>();
                    ForceRegistry = new ParticleForceRegistry();
                    ParticleRegistry = new List<Particle>();
                    if (iterations == 0)
                        CalculateIterations = true;
                }
                public void StartFrame()
                {
                    foreach (Particle item in ParticleRegistry)
                    {
                        item.ClearForceAccumlator();
                    }
                }
                public int GenerateContacts()
                {
                    int i = 0, j = 0;
                    int limit = MaxContacts;
                    ParticleContact nextcontact = Contacts[j];
                    ParticleContactGenerator gen = ContactRegistration[i];
                    while (i < ContactRegistration.Count)
                    {
                        int used = gen.AddContact(nextcontact, limit);
                        limit -= used;
                        j += used;
                        nextcontact = Contacts[j];
                        if (limit <= 0)
                            break;
                        i++;
                    }
                    return 0;
                }
                public void Integrate(float duration)
                {
                    foreach (Particle item in ParticleRegistry)
                    {
                        item.Integrate(duration);
                    }
                }
                void RunPhysics(float duration)
                {
                    ForceRegistry.UpdateForces(duration);
                    Integrate(duration);
                    int usedcontacts = GenerateContacts();
                    if (CalculateIterations)
                        Resolver.SetIterations(usedcontacts * 2);
                    Resolver.ResolveContacts(Contacts, usedcontacts, duration);
                }
                #region MyMethods
                public void AddParticle(Particle p)
                {
                    ParticleRegistry.Add(p);
                }
                public void AddForce(Particle p, ParticleForceGenerator pfg)
                {
                    ForceRegistry.Add(p, pfg);
                }
                public void AddContact(ParticleContact pc)
                {
                    Contacts.Add(pc);
                }
                #endregion
            }
        }
    }
    namespace Rigid 
    {
        namespace World
        {
            [Serializable]
            public class plane
            {
                public Vector3 point;
                public float tangent;
                public Vector3 tangentVector;
                public plane(Vector3 p, Vector3 t)
                {
                    //point
                    this.point = p;

                    //tangent vector
                    this.tangentVector = t;

                    //calculate the tangent
                    float destx = p.X + t.X;
                    float desty = p.Y + t.Y;

                    this.tangent = (p.Y - desty) / (p.X - destx);
                }
                public CollisionPlane getCollisionPlane()
                {
                    return new CollisionPlane(this.point, this.tangent,this.tangentVector);
                }
            }
            [Serializable]
            public class World
            {
                //number of maximum contacts in the frame
                public int MaxContacts;

                //Objects Lists
                public List<RigidBody> RigidRegistry;
                public List<Joint> JointRegistry;
                public ForceRegistry ForcesRegistry;

                //map shit
                public List<Vector2> MapBezierPoints;
                public plane[] Planes;
                public CollisionPlane[] Map=null;
                public int maplength=0;

                //Contact 
                public CollisionData ColliData;

                //explode threshold
                public static double explodeThreshold = 1000000;

                //Objects to delete
                public List<RigidBody> ToDeleteRegistry;

                public World(int maxcontacts,List<Vector2> MapBezierPoints=null)
                {
                    this.MapBezierPoints = MapBezierPoints;
                    this.MaxContacts = maxcontacts;
                    JointRegistry = new List<Joint>();
                    RigidRegistry = new List<RigidBody>();
                    ForcesRegistry = new ForceRegistry();
                    ToDeleteRegistry = new List<RigidBody>();
                    
                    #region mapBuilder
                    if (MapBezierPoints!=null)
                    {
                        //building map
                        this.maplength = (int)MapBezierPoints[MapBezierPoints.Count - 1].X;

                        Planes = new plane[maplength+1];
                        Map = new CollisionPlane[maplength+1];

                        //jumping
                        int start = 0;
                        int end = 0;

                        for (int i = 0; i <= maplength; )
                        {
                            end = start + 3;
                            if (i >= MapBezierPoints[end].X)
                            {
                                start += 3;
                                continue;
                            }

                            float step = 0;
                            //float dstep =1.0f/ (MapBezierPoints[end].X - MapBezierPoints[start].X);
                            for (i = (int)MapBezierPoints[start].X; i <= MapBezierPoints[end].X; i++)
                            {
                                step = GetT(MapBezierPoints[start].X, MapBezierPoints[start + 1].X, MapBezierPoints[start + 2].X, MapBezierPoints[start + 3].X, (float)i);
                                Planes[i] = new plane(
                                    Bezier(MapBezierPoints[start], MapBezierPoints[start + 1], MapBezierPoints[start + 2], MapBezierPoints[start + 3], step),
                                    BezierTangent(MapBezierPoints[start], MapBezierPoints[start + 1], MapBezierPoints[start + 2], MapBezierPoints[start + 3], step));
                                //step += dstep;
                            }
                        }

                        //making collisionPlanes
                        for (int i = 0; i <= maplength; i++)
                        {
                            Map[i] = Planes[i].getCollisionPlane();
                        }
                    }
                    #endregion
                }
                public void RunPhysics(float duration)
                {
                    ForcesRegistry.UpdateForces(duration);
                    Integrate(duration);
                    ColliData = GenerateContacts();
                    GenerateJointData();
                    ContactResolver.ResolveContacts(ColliData.Contacts, ColliData.ContactUsed , duration/2.0f);
                }
                public void wakeAll()
                {
                    for (int i = 0; i < this.RigidRegistry.Count; i++)
                    {
                        this.RigidRegistry[i].IsAwake = true;
                        this.RigidRegistry[i].Motion = this.RigidRegistry[i].SleepEpsilon * 2.0f;
                    }
                }
                public CollisionData GenerateContacts()
                {
                    //bascically without BVH
                    CollisionData temp = new CollisionData(MaxContacts);
                    for (int i = 0; i < this.RigidRegistry.Count; i++)
                    {
                        RigidBody item = RigidRegistry[i];

                        #region check collision with ground
                        //check collision with planes from -halfsize to +halfsize
                        if (Map != null)
                        {
                            int left = (int)item.GetPosition().X;
                            int right = left;
                            if (item is Box)
                            {
                                Box tempbox = item as Box;

                                left -= (int)(1.4f * tempbox.HalfSizeWidth);
                                right += (int)(1.4f * tempbox.HalfSizeWidth);
                                if (right > this.maplength) right = maplength;
                                if (left < 0) left = 0;
                                for (int navigator = left; navigator <= right; navigator++)
                                {
                                    double rnd = new Random().NextDouble();
                                    rnd = rnd < 0.5 ? rnd += 0.5 : rnd;
                                    if (CollisionDetector.BoxAndHalfSpace(tempbox.collisionbox, Map[navigator], ref temp) && (item is iExplosive) && (item.Velocity.Length() * item.GetMass() * rnd > explodeThreshold))
                                    {
                                        iExplosive Explosive = (iExplosive)item;
                                        Explosive.Explode();
                                    }
                                    if (item.Velocity.Length() * item.GetMass() * rnd > explodeThreshold * 100)
                                    {
                                        removeJointOfBody(item);
                                    } 
                                }
                            }
                            else if (item is Circle)
                            {
                                Circle tempcircle = item as Circle;
                                left -= (int)(tempcircle.Radius);
                                right += (int)(tempcircle.Radius);
                                if (right > this.maplength) right = maplength;
                                if (left < 0) left = 0;
                                for (int navigator = left; navigator <= right; navigator++)
                                {
                                    double rnd = new Random().NextDouble();
                                    rnd = rnd < 0.5 ? rnd += 0.5 : rnd;
                                    if (CollisionDetector.CircleAndHalfSpace(tempcircle.collisioncircle, Map[navigator], ref temp) && (item is iExplosive) && (item.Velocity.Length() * item.GetMass() * rnd > explodeThreshold))
                                    {
                                        iExplosive Explosive = (iExplosive)item;
                                        Explosive.Explode();
                                    }
                                    if (item.Velocity.Length() * item.GetMass() * rnd > explodeThreshold * 100)
                                    {
                                        removeJointOfBody(item);
                                    } 
                                }
                            }
                        }
                        #endregion

                        //check collision with each other body
                        for (int j = i+1; j < this.RigidRegistry.Count; j++)
                        {
                            RigidBody item2 = RigidRegistry[j];

                            #region calculate on collision effects
                            if(item.Collide(item2, ref temp))
                            {
                                double rnd = new Random().NextDouble();
                                rnd = rnd < 0.5 ? rnd += 0.5 : rnd;
                                if (item is iExplosive && (item.Velocity.Length() * item.GetMass() * rnd + item2.Velocity.Length() * item2.GetMass() * rnd > explodeThreshold))
                                {
                                    iExplosive Explosive = (iExplosive)item;
                                    Explosive.Explode();
                                }
                                else if (item2 is iExplosive && (item.Velocity.Length() * item.GetMass() * rnd + item2.Velocity.Length() * item2.GetMass() * rnd > explodeThreshold))
                                {
                                    iExplosive Explosive = (iExplosive)item2;
                                    Explosive.Explode();
                                }

                                if (item2.Velocity.Length() * item2.GetMass() * rnd + item.Velocity.Length() * item.GetMass() * rnd > explodeThreshold*100)
                                {
                                    removeJointOfBody(item);
                                    removeJointOfBody(item2);
                                } 
                            }
                            #endregion
                        }
                    }
                            
                    return temp;
                }
                public void removeJointOfBody(RigidBody rg)
                {
                    List<Joint> todelete = new List<Joint>();
                    for (int i = 0; i < JointRegistry.Count; i++)
                    {
                        if (JointRegistry[i].IsPartOf(rg))
                        {
                            todelete.Add(JointRegistry[i]);
                        }
                    }
                    foreach (Joint item in todelete)
                    {
                        if (!item.breakable)
                            continue;
                        JointRegistry.Remove(item);
                        if (item.bodies[0] is Box && item.bodies[1] is Circle)
                        {
                            Box t = item.bodies[0] as Box;
                            Circle t2 = item.bodies[1] as Circle;
                            t.JointedCircles.Remove(item.bodies[1] as Circle);
                            t2.joint = false;
                            t2.jointedWith[0] = null;
                            for (int i = 0; i < t.jointedWith.Length; i++)
                            {
                                if (t.jointedWith[i] == t2)
                                {
                                    t.jointedWith[i] = null;
                                    t.joints[i] = false;
                                    break;
                                }
                            }
                        }
                        else if (item.bodies[1] is Box && item.bodies[0] is Circle)
                        {
                            Box t = item.bodies[1] as Box;
                            t.JointedCircles.Remove(item.bodies[0] as Circle);
                            Circle t2 = item.bodies[0] as Circle;
                            t2.joint = false;
                            t2.jointedWith[0] = null;
                            for (int i = 0; i < t.jointedWith.Length; i++)
                            {
                                if (t.jointedWith[i] == t2)
                                {
                                    t.jointedWith[i] = null;
                                    t.joints[i] = false;
                                    break;
                                }
                            }
                        }
                        else if (item.bodies[1] is Box && item.bodies[0] is Box)
                        {
                            Box t = item.bodies[0] as Box;
                            Box t2 = item.bodies[1] as Box;

                            for (int i = 0; i < t.jointedWith.Length; i++)
                            {
                                if (t.jointedWith[i] == t2)
                                {
                                    t.jointedWith[i] = null;
                                    t.joints[i] = false;
                                    break;
                                }
                            }
                            for (int i = 0; i < t2.jointedWith.Length; i++)
                            {
                                if (t2.jointedWith[i] == t)
                                {
                                    t2.jointedWith[i] = null;
                                    t2.joints[i] = false;
                                    break;
                                }
                            }
                        }
                    }
                }
                public void GenerateJointData() 
                {
                    for(int i=0;i<JointRegistry.Count;i++) 
                    {
                        Joint item = JointRegistry[i];
                        if (!item.AddContact(ref ColliData)) 
                        {
                            JointRegistry.Remove(item);
                            if (item.bodies[0] is Box && item.bodies[1] is Circle)
                            {
                                Box t = item.bodies[0] as Box;
                                t.JointedCircles.Remove(item.bodies[1] as Circle);
                            }
                            else if (item.bodies[1] is Box && item.bodies[0] is Circle)
                            {
                                Box t = item.bodies[1] as Box;
                                t.JointedCircles.Remove(item.bodies[0] as Circle);
                            }
                        }
                    }
                }
                public void Integrate(float duration)
                {
                    foreach (RigidBody item in RigidRegistry)
                    {
                        item.Update(duration);
                        if (item.canDelete)
                        {
                            this.ToDeleteRegistry.Add(item);
                        }
                    }
                }
                public void AddJoint(RigidBody a,Vector3 a_pos, RigidBody b, Vector3 b_pos,float error) 
                {
                    if (RigidRegistry.Contains(a) && RigidRegistry.Contains(b))
                    {
                        JointRegistry.Add(new Joint(a, a_pos, b, b_pos,error));
                        JointRegistry[JointRegistry.Count - 1].breakable = true;
                    }
                }
                public void AddJoint(Joint j)
                {
                    this.AddJoint(j.bodies[0], j.Position[0], j.bodies[1], j.Position[1], j.Error);
                }
                public void AddBody(RigidBody b)
                {
                    RigidRegistry.Add(b);
                }
                public void AddForce(RigidBody rg,RigidForceGenerator f)
                {
                    if (RigidRegistry.Contains(rg))
                        ForcesRegistry.Add(rg, f);
                }
                public void RemoveJoint(Joint j)
                {
                    if (RigidRegistry.Contains(j.bodies[0]) && RigidRegistry.Contains(j.bodies[1]))
                    {
                        for (int i = 0; i < j.bodies[0].jointedWith.Length; i++)
                        {
                            if (j.bodies[0].jointedWith[i] == j.bodies[1])
                            {
                                j.bodies[0].jointedWith[i] = null;
                                j.bodies[0].removeJoint(i);
                            }
                        }

                        for (int i = 0; i < j.bodies[1].jointedWith.Length; i++)
                        {
                            if (j.bodies[1].jointedWith[i] == j.bodies[0])
                            {
                                j.bodies[1].jointedWith[i] = null;
                                j.bodies[1].removeJoint(i);
                            }
                        }
                        JointRegistry.Remove(j);
                        if (j!=null)
                        {
                            if (j.bodies[0] is Box && j.bodies[1] is Circle)
                            {
                                Box t = j.bodies[0] as Box;
                                t.JointedCircles.Remove(j.bodies[1] as Circle);
                            }
                            else if (j.bodies[1] is Box && j.bodies[0] is Circle)
                            {
                                Box t = j.bodies[1] as Box;
                                t.JointedCircles.Remove(j.bodies[0] as Circle);
                            }
                        }
                    }
                }
                public void RemoveBody(RigidBody b)
                {
                    if (RigidRegistry.Contains(b))
                        RigidRegistry.Remove(b);
                }
                public void RemoveForce(RigidBody rg, RigidForceGenerator f)
                {
                    if (RigidRegistry.Contains(rg))
                        ForcesRegistry.Remove(rg, f);
                }
                //map!
                public float Bezier(float p1, float p2, float p3, float p4, float t)
                {
                    return p1 * (1 - t) * (1 - t) * (1 - t) +
                           p2 * 3 * t * (1 - t) * (1 - t) +
                           p3 * 3 * t * t * (1 - t) +
                           p4 * t * t * t;
                }
                public Vector3 Bezier(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, float t)
                {
                    Vector3 point = new Vector3();
                    point.X = Bezier(p1.X, p2.X, p3.X, p4.X, t);
                    point.Y = Bezier(p1.Y, p2.Y, p3.Y, p4.Y, t);
                    point.Z = 0;
                    return point;
                }
                public float BezierTangent(float p1, float p2, float p3, float p4, float t)
                { 
                    //div(b(t)) = T(t) = 3(1-t)*(1-t) (P1-P0) + 6(1-t)t(p2-p1) + 3t*t(p3-p2)
                    return 3 * (1 - t) * (1 - t) * (p2 - p1) +
                           6 * (1 - t) * t * (p3 - p2) +
                           3 * t * t * (p4 - p3);
                }
                public Vector3 BezierTangent(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, float t)
                {
                    Vector3 Tangent = new Vector3();
                    Tangent.X = BezierTangent(p1.X, p2.X, p3.X, p4.X, t);
                    Tangent.Y = BezierTangent(p1.Y, p2.Y, p3.Y, p4.Y, t);
                    Tangent.Z = 0;
                    Tangent.Normalize();
                    return Tangent;
                }
                public float GetT(float x1, float x2, float x3, float x4, float x)
                {
                    float xbezier;
                    float start = 0.0f;
                    float end = 1.0f;
                    float mid;
                    do
                    {
                        mid = (start + end) / 2;
                        xbezier = Bezier(x1, x2, x3, x4, mid);
                        if (xbezier > x)
                        {
                            end=mid;
                        }
                        else if (xbezier < x)
                        {
                            start = mid;
                        }
                        else
                        {
                            return mid;
                        }
                    } while (Math.Abs(xbezier - x) > 0.001);
                    return mid;
                }
            }
            public class RodWorld
            {
                //number of maximum contacts in the frame
                public int MaxContacts;

                //Objects Lists
                public List<RigidBody> RigidRegistry;
                public List<RigidRod> JointRegistry;
                public ForceRegistry ForcesRegistry;

                //map shit
                public List<Vector2> MapBezierPoints;
                public plane[] Planes;
                public CollisionPlane[] Map = null;
                public int maplength = 0;

                //Contact 
                public CollisionData ColliData;

                //explode threshold
                public static double explodeThreshold;

                //Objects to delete
                public List<RigidBody> ToDeleteRegistry;

                public RodWorld(int maxcontacts, List<Vector2> MapBezierPoints = null)
                {
                    this.MapBezierPoints = MapBezierPoints;
                    this.MaxContacts = maxcontacts;
                    JointRegistry = new List<RigidRod>();
                    RigidRegistry = new List<RigidBody>();
                    ForcesRegistry = new ForceRegistry();
                    ToDeleteRegistry = new List<RigidBody>();
                    explodeThreshold = 1000000;
                    #region mapBuilder
                    if (MapBezierPoints != null)
                    {
                        //building map
                        this.maplength = (int)MapBezierPoints[MapBezierPoints.Count - 1].X;

                        Planes = new plane[maplength + 1];
                        Map = new CollisionPlane[maplength + 1];

                        //jumping
                        int start = 0;
                        int end = 0;

                        for (int i = 0; i <= maplength; )
                        {
                            end = start + 3;
                            if (i >= MapBezierPoints[end].X)
                            {
                                start += 3;
                                continue;
                            }

                            float step = 0;
                            //float dstep =1.0f/ (MapBezierPoints[end].X - MapBezierPoints[start].X);
                            for (i = (int)MapBezierPoints[start].X; i <= MapBezierPoints[end].X; i++)
                            {
                                step = GetT(MapBezierPoints[start].X, MapBezierPoints[start + 1].X, MapBezierPoints[start + 2].X, MapBezierPoints[start + 3].X, (float)i);
                                Planes[i] = new plane(
                                    Bezier(MapBezierPoints[start], MapBezierPoints[start + 1], MapBezierPoints[start + 2], MapBezierPoints[start + 3], step),
                                    BezierTangent(MapBezierPoints[start], MapBezierPoints[start + 1], MapBezierPoints[start + 2], MapBezierPoints[start + 3], step));
                                //step += dstep;
                            }
                        }

                        //making collisionPlanes
                        for (int i = 0; i <= maplength; i++)
                        {
                            Map[i] = Planes[i].getCollisionPlane();
                        }
                    }
                    #endregion
                }
                public void RunPhysics(float duration)
                {
                    GenerateJointData(duration);
                    ForcesRegistry.UpdateForces(duration);
                    Integrate(duration);
                    ColliData = GenerateContacts();
                    ContactResolver.ResolveContacts(ColliData.Contacts, ColliData.ContactUsed, duration / 2.0f);
                }
                public void wakeAll()
                {
                    for (int i = 0; i < this.RigidRegistry.Count; i++)
                    {
                        this.RigidRegistry[i].IsAwake = true;
                        this.RigidRegistry[i].Motion = this.RigidRegistry[i].SleepEpsilon * 2.0f;
                    }
                }
                public CollisionData GenerateContacts()
                {
                    //bascically without BVH
                    CollisionData temp = new CollisionData(MaxContacts);
                    for (int i = 0; i < this.RigidRegistry.Count; i++)
                    {
                        RigidBody item = RigidRegistry[i];

                        #region check collision with ground
                        //check collision with planes from -halfsize to +halfsize
                        if (Map != null)
                        {
                            int left = (int)item.GetPosition().X;
                            int right = left;
                            if (item is Box)
                            {
                                Box tempbox = item as Box;

                                left -= (int)(1.4f * tempbox.HalfSizeWidth);
                                right += (int)(1.4f * tempbox.HalfSizeWidth);
                                if (right > this.maplength) right = maplength;
                                if (left < 0) left = 0;
                                for (int navigator = left; navigator <= right; navigator++)
                                {
                                    double rnd = new Random().NextDouble();
                                    rnd = rnd < 0.5 ? rnd += 0.5 : rnd;
                                    bool detectground = CollisionDetector.BoxAndHalfSpace(tempbox.collisionbox, Map[navigator], ref temp);
                                    if (detectground && (item is iExplosive) && (item.Velocity.Length() * item.GetMass() * rnd > explodeThreshold))
                                    {
                                        iExplosive Explosive = (iExplosive)item;
                                        Explosive.Explode();
                                    }
                                    if (detectground &&item.Velocity.Length() * item.GetMass() * rnd > explodeThreshold * 100)
                                    {
                                        removeJointOfBody(item);
                                    }
                                }
                            }
                            else if (item is Circle)
                            {
                                Circle tempcircle = item as Circle;
                                left -= (int)(tempcircle.Radius);
                                right += (int)(tempcircle.Radius);
                                if (right > this.maplength) right = maplength;
                                if (left < 0) left = 0;
                                for (int navigator = left; navigator <= right; navigator++)
                                {
                                    double rnd = new Random().NextDouble();
                                    rnd = rnd < 0.5 ? rnd += 0.5 : rnd;
                                    bool detectground = CollisionDetector.CircleAndHalfSpace(tempcircle.collisioncircle, Map[navigator], ref temp);
                                    if (detectground && (item is iExplosive) && (item.Velocity.Length() * item.GetMass() * rnd > explodeThreshold))
                                    {
                                        iExplosive Explosive = (iExplosive)item;
                                        Explosive.Explode();
                                    }
                                    if (detectground && item.Velocity.Length() * item.GetMass() * rnd > explodeThreshold * 100)
                                    {
                                        removeJointOfBody(item);
                                    }
                                }
                            }
                        }
                        #endregion

                        //check collision with each other body
                        for (int j = i + 1; j < this.RigidRegistry.Count; j++)
                        {
                            RigidBody item2 = RigidRegistry[j];

                            #region calculate on collision effects
                            if (item.Collide(item2, ref temp))
                            {
                                double rnd = new Random().NextDouble();
                                rnd = rnd < 0.5 ? rnd += 0.5 : rnd;
                                if (item is iExplosive && (item.Velocity.Length() * item.GetMass() * rnd + item2.Velocity.Length() * item2.GetMass() * rnd > explodeThreshold / 10))
                                {
                                    iExplosive Explosive = (iExplosive)item;
                                    Explosive.Explode();
                                }
                                else if (item2 is iExplosive && (item.Velocity.Length() * item.GetMass() * rnd + item2.Velocity.Length() * item2.GetMass() * rnd > explodeThreshold / 10))
                                {
                                    iExplosive Explosive = (iExplosive)item2;
                                    Explosive.Explode();
                                }

                                if (item2.Velocity.Length() * item2.GetMass() * rnd + item.Velocity.Length() * item.GetMass() * rnd > explodeThreshold * 100)
                                {
                                    removeJointOfBody(item);
                                    removeJointOfBody(item2);
                                }
                            }
                            #endregion
                        }
                    }

                    return temp;
                }
                public void removeJointOfBody(RigidBody rg)
                {
                    List<RigidRod> todelete = new List<RigidRod>();
                    for (int i = 0; i < JointRegistry.Count; i++)
                    {
                        if (JointRegistry[i].IsPartOf(rg))
                        {
                            todelete.Add(JointRegistry[i]);
                        }
                    }
                    foreach (RigidRod item in todelete)
                    {
                        JointRegistry.Remove(item);

                    }
                }
                public void GenerateJointData(float duration)
                {
                    for (int i = 0; i < JointRegistry.Count; i++)
                    {
                        RigidRod item = JointRegistry[i];
                        item.set(item.bodies[0], item.bodies[1]);
                        item.Adjust(duration);
                    }
                }
                public void Integrate(float duration)
                {
                    foreach (RigidBody item in RigidRegistry)
                    {
                        item.Update(duration);
                        if (item.canDelete)
                        {
                            this.ToDeleteRegistry.Add(item);
                        }
                    }
                }
                public void AddJoint(RigidBody a, Vector3 a_pos, RigidBody b, Vector3 b_pos, float error)
                {
                    if (RigidRegistry.Contains(a) && RigidRegistry.Contains(b))
                    {
                        JointRegistry.Add(new RigidRod(a,b));
                    }
                }
                public void AddJoint(RigidRod j)
                {
                    this.AddJoint(j.bodies[0], j.bodies[0].Position, j.bodies[1], j.bodies[1].Position,3);
                }
                public void AddBody(RigidBody b)
                {
                    RigidRegistry.Add(b);
                }
                public void AddForce(RigidBody rg, RigidForceGenerator f)
                {
                    if (RigidRegistry.Contains(rg))
                        ForcesRegistry.Add(rg, f);
                }
                public void RemoveJoint(RigidRod j)
                {
                    if (RigidRegistry.Contains(j.bodies[0]) && RigidRegistry.Contains(j.bodies[1]))
                    {
                        for (int i = 0; i < j.bodies[0].jointedWith.Length; i++)
                        {
                            if (j.bodies[0].jointedWith[i] == j.bodies[1])
                            {
                                j.bodies[0].jointedWith[i] = null;
                                j.bodies[0].removeJoint(i);
                            }
                        }

                        for (int i = 0; i < j.bodies[1].jointedWith.Length; i++)
                        {
                            if (j.bodies[1].jointedWith[i] == j.bodies[0])
                            {
                                j.bodies[1].jointedWith[i] = null;
                                j.bodies[1].removeJoint(i);
                            }
                        }
                        JointRegistry.Remove(j);
                    }
                }
                public void RemoveBody(RigidBody b)
                {
                    if (RigidRegistry.Contains(b))
                        RigidRegistry.Remove(b);
                }
                public void RemoveForce(RigidBody rg, RigidForceGenerator f)
                {
                    if (RigidRegistry.Contains(rg))
                        ForcesRegistry.Remove(rg, f);
                }
                //map!
                public float Bezier(float p1, float p2, float p3, float p4, float t)
                {
                    return p1 * (1 - t) * (1 - t) * (1 - t) +
                           p2 * 3 * t * (1 - t) * (1 - t) +
                           p3 * 3 * t * t * (1 - t) +
                           p4 * t * t * t;
                }
                public Vector3 Bezier(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, float t)
                {
                    Vector3 point = new Vector3();
                    point.X = Bezier(p1.X, p2.X, p3.X, p4.X, t);
                    point.Y = Bezier(p1.Y, p2.Y, p3.Y, p4.Y, t);
                    point.Z = 0;
                    return point;
                }
                public float BezierTangent(float p1, float p2, float p3, float p4, float t)
                {
                    //div(b(t)) = T(t) = 3(1-t)*(1-t) (P1-P0) + 6(1-t)t(p2-p1) + 3t*t(p3-p2)
                    return 3 * (1 - t) * (1 - t) * (p2 - p1) +
                           6 * (1 - t) * t * (p3 - p2) +
                           3 * t * t * (p4 - p3);
                }
                public Vector3 BezierTangent(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, float t)
                {
                    Vector3 Tangent = new Vector3();
                    Tangent.X = BezierTangent(p1.X, p2.X, p3.X, p4.X, t);
                    Tangent.Y = BezierTangent(p1.Y, p2.Y, p3.Y, p4.Y, t);
                    Tangent.Z = 0;
                    Tangent.Normalize();
                    return Tangent;
                }
                public float GetT(float x1, float x2, float x3, float x4, float x)
                {
                    float xbezier;
                    float start = 0.0f;
                    float end = 1.0f;
                    float mid;
                    do
                    {
                        mid = (start + end) / 2;
                        xbezier = Bezier(x1, x2, x3, x4, mid);
                        if (xbezier > x)
                        {
                            end = mid;
                        }
                        else if (xbezier < x)
                        {
                            start = mid;
                        }
                        else
                        {
                            return mid;
                        }
                    } while (Math.Abs(xbezier - x) > 0.001);
                    return mid;
                }
            }
        }
    }

}
