using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Globalization;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Text.RegularExpressions;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

// ! maybe post in the discord
// ! everything from here up to the start of logging in Main() is around 200 LoC

namespace IngameScript
{
    partial class Program : MyGridProgram
    {

        public IMyShipController controller = null;
        public IMyMotorStator pitch, roll;
        public IMyThrust thruster;
        public IMyLandingGear landingGear;
        public bool applyGravWhileLocked;
        public double gearAccel;
        public double stoppingTime;
        public Vector3D testOppVec;
        private List<LCD> lcds = new List<LCD>();
        public PID pitchPID, rollPID, thrustPID;
        public int counter;
        public static readonly float radToDeg = (float) (180 / Math.PI);
        public static readonly float piOver2 = (float) (Math.PI / 2);
        public enum WriteAngle {
            RAD,
            DEG,
            BOTH
        }

        
        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Once | UpdateFrequency.Update1;
            Init();
        }

        public void Init() {

            counter = 0;
            testOppVec = new Vector3D(0, 0, 0);

            // get ship controller
            List<IMyShipController> controllers = new List<IMyShipController>();
            GridTerminalSystem.GetBlocksOfType<IMyShipController>(controllers);
            controller = controllers.Pop();

            // get a landing gear
            landingGear = GridTerminalSystem.GetBlockWithName("Lander") as IMyLandingGear;

            // get screens
            List<IMyTextPanel> lcdBlocks = new List<IMyTextPanel>();
            GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(lcdBlocks);

            // this ignores screens configured for a different script i made
            // i think it would probably be better to use GetBlockGroupWithName
            int idx = lcdBlocks.FindIndex(e => e.CustomName == "Rain");
            while(idx > -1) {
                lcdBlocks.RemoveAt(idx);
                idx = lcdBlocks.FindIndex(e => e.CustomName == "Rain");
            }
            lcdBlocks.ForEach(e => {
                lcds.Add(new LCD(e));
            });

            // get thruster
            thruster = GridTerminalSystem.GetBlockWithName("Thruster") as IMyThrust;

            // get rotors
            pitch = GridTerminalSystem.GetBlockWithName("RotorPitch") as IMyMotorStator;
            roll = GridTerminalSystem.GetBlockWithName("RotorRoll") as IMyMotorStator;

            // create PIDs for pitch and roll rotors
            pitchPID = new PID(1.35, 0, 0, 1.0 / 60.0);
            rollPID = new PID(1.35, 0, 0, 1.0 / 60.0);

            // thruster PID experiment, didn't go so well, but maybe a tuning error
            // thrustPID = new PID(controller.CalculateShipMass().PhysicalMass * 3 / 5, 0, 0, 1.0 / 60.0);

            // set acceleration/"gear" in Newtons (grams/m^s2)
            gearAccel = 3000;
            // set stopping time for dampening vector
            stoppingTime = 0.6;

            // set whether to apply gravity while parked/locked
            applyGravWhileLocked = false;
        }

        public void Save()
        {
            // Called when the program needs to save its state. Use
            // this method to save your state to the Storage field
            // or some other means. 
            // 
            // This method is optional and can be removed if not
            // needed.
        }

        public void Main(string argument, UpdateType updateSource)
        {
            Echo(string.Format("Counter: {0}", counter));
            counter++;

            // argument handling; PID tuning and some other utility things that shouldn't affect flight
            {
                if (argument != "" ) {
                    if (argument == "gravWhileLocked") applyGravWhileLocked = !applyGravWhileLocked;
                    if (argument == "regen") {
                        Random r = new Random();
                        testOppVec = new Vector3D((double) r.Next(0, 201) / 100 - 1, 
                                            (double) r.Next(0, 101) / 100 - 1, // only downward y
                                            (double) r.Next(0, 201) / 100 - 1);
                        SafeNormalize(testOppVec);
                    }
                    if (argument == "reset") { 
                        testOppVec = new Vector3D(0, 0, 0);
                        pitchPID.Kp = rollPID.Kp = 1.35;
                        pitchPID.Ki = rollPID.Ki = 0;
                        pitchPID.Kd = rollPID.Kd = 0;
                    }
                    if (argument[0] == 'p') {
                        if (argument[1] == '+') {
                            pitchPID.Kp += 0.05d;
                            rollPID.Kp += 0.05d;
                        } else {
                            pitchPID.Kp -= 0.05d;
                            rollPID.Kp -= 0.05d;
                        }
                    }
                    if (argument[0] == 'i') {
                        if (argument[1] == '+') {
                            pitchPID.Ki += 0.05d;
                            rollPID.Ki += 0.05d;
                        } else {
                            pitchPID.Ki -= 0.05d;
                            rollPID.Ki -= 0.05d;
                        }
                    }
                    if (argument[0] == 'd') {
                        if (argument[1] == '+') {
                            pitchPID.Kd += 0.02d;
                            rollPID.Kd += 0.02d;
                        } else {
                            pitchPID.Kd -= 0.02d;
                            rollPID.Kd -= 0.02d;
                        }
                    }
                }
            }

            // math links
            // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula#Statement
            // https://archive.lib.msu.edu/crcmath/math/math/s/s571.htm

            // world matrix <-> local vector calculations from 
            // https://github.com/malware-dev/MDK-SE/wiki/Vector-Transformations-with-World-Matrices
            MatrixD worldMatrix = controller.WorldMatrix;
            Matrix invWorldRot = MatrixD.Transpose(controller.WorldMatrix);

            Vector3D localVelocity = Vector3D.Rotate(controller.GetShipVelocities().LinearVelocity, invWorldRot);
            Vector3D moveInd = controller.MoveIndicator;

            // I'd like to set this as a constant that updates whenever the ship becomes unparked.
            // That way, when docked to a station with other subgrids docked on it,
            // the ship doesn't think it has 1 billion mass.
            float mass = controller.CalculateShipMass().PhysicalMass; 

            // gravity-related calculations
            Vector3D totalGravityVec = controller.GetTotalGravity();
            Vector3D localGravVec = Vector3D.Rotate(totalGravityVec, invWorldRot);
            Vector3D shipWeightVec = mass * localGravVec;
            double gravAccel = localGravVec.Length();
            double gForce = gravAccel / 9.81;
            Vector3D localGravDirection = SafeNormalize(localGravVec);

            // this is just for tracking/readout/utility, doesn't get used in antigravity/dampening calculations
            Vector3D rotatedThrusterVec = CurrentThrusterDirectionVector();

            // Final result will be a force, vector pair (Force in Newtons, vector as a unit direction vector) defined by:
            // F_final * V_final = -1 * (F_gravity * V_gravity + F_dampening * V_dampening - F_direction * V_direction)
            // F_grav is gravity applied to the ship's weight, V_grav is the unit local gravity direction vector
            // F_dampening is the acceleration approximation to cancel ship velocity, V_dampening is the counter-direction of ship velocity
            // F_directional is the acceleration we want for directional input, V_directional is the unit directional input
            
            // Dampening implementation
            // I referenced Whip's vector thrust script for this, but I think further changes might be needed
            // hopefully my comments describe the math correctly, i put them there to help my understanding
            Vector3D V_damp = Vector3D.Zero;
            // are we making a directional input?
            if (!moveInd.IsZero()) { 
                if (Vector3D.Dot(moveInd, localVelocity) < 0) { 
                    // if we're moving "against" velocity (at all beyond perpendicular to velocty), 
                    // that dampens our movement more than we would like,
                    // so let's add some extra strength to our V_final to cancel that out.
                    // (remember that V_final is the inverse of the gravity and dampening vectors, 
                    // so addition here becomes subtraction later)
                    V_damp += Projection(localVelocity, moveInd.Normalized());
                }
                // in general if we're moving, add extra strength to V_final so our own movement doesn't get dampened
                V_damp += Vector3D.Reject(localVelocity, moveInd.Normalized());
            } else {
                // if we aren't moving, just counter the existing velocity
                V_damp += localVelocity;
            }

            // ! not sure if this calculation needs adjusting,
            // ! or if my rotors are introducing lift, or both
            // ! dampStrengthPercentage is a thing I saw in whip's script
            // ! but I wasn't sure how to effectively use it
            double F_damp = mass * V_damp.Length() / stoppingTime;
            double dampStrengthPercentage = 1;
            V_damp.Normalize();

            // * would like to eventually maybe test the PID velocity dampener again,
            // * if not only to see if it's better for performance
            // * not sure how to best initialize values or tune it though
            // double F_damp = thrustPID.Control(thrustErr);
            
            double F_final, F_grav, F_directional;
            Vector3D V_final, V_grav, V_directional;
            F_grav = mass * gravAccel; // f=ma
            F_directional = gearAccel; // gearAccel is a fixed value in N for now, might add cruise control later
            V_grav = localGravDirection;
            if (landingGear.IsLocked && !applyGravWhileLocked) F_grav = 0;
            V_directional = moveInd;
            V_final = (F_grav * V_grav) - (F_directional * V_directional) + (V_damp * F_damp * dampStrengthPercentage);
            
            F_final = !V_final.IsZero() ? V_final.Length() : 0;

            Vector3D requiredVec = -1 * V_final;

            MyTuple<double, double> requiredAngles = GetAnglesFromVector(requiredVec);
            double requiredPitchAngle = requiredAngles.Item1;
            double requiredRollAngle = requiredAngles.Item2;

            double pitchErr = requiredPitchAngle - pitch.Angle;
            double rollErr = requiredRollAngle - roll.Angle;

            // * apply pitch and roll PIDs to rotor velocity
            if ((updateSource & UpdateType.Update1) != 0) {

                // * avoid setting via ThrustOverride - it has some issues with atmospheric multiplier?
                // * do this instead
                thruster.ThrustOverridePercentage = (float) F_final / thruster.MaxEffectiveThrust;
                // * source: whiplash on discord said so lol
                // https://canary.discord.com/channels/125011928711036928/216219467959500800/1307221752153243699
                
                // thruster PID experiment to try and counter leftover velocity after handling gravity, didn't go too well
                // maybe just a tuning issue though
                // if (!moveInd.IsZero()) thruster.ThrustOverridePercentage = (float) F_final / thruster.MaxEffectiveThrust;
                // else {
                //     thruster.ThrustOverridePercentage = (float) (F_final + F_damp) / thruster.MaxEffectiveThrust;
                // }

                // do PID for pitch/roll
                float pitchTgtVel = (float) pitchPID.Control(pitchErr);
                float rollTgtVel = (float) rollPID.Control(rollErr);

                // clamp to some rpm value
                float velCapRPM = 54;
                float velCapRad = velCapRPM * (float) Math.PI / 30;
                // per minute to per second: rpm * 60
                // revolutions to radians: rps * pi/180
                // total conversion: rpm * pi / 30

                // TODO
                // TODO sometimes the thrusters put too much force
                // TODO and start turning the rotors
                // TODO might be able to increment the kI and kD values
                // TODO for PIDs dynamically - as the needed thrust increases

                // ? not sure if i need to clamp velocities
                // ? some testing during a previous (probably more buggy) state
                // ? introduced some weird instability/feedback loops
                // ? during angle rotation
                if (pitchTgtVel > velCapRad) pitchTgtVel = velCapRad;
                else if (pitchTgtVel < -velCapRad) pitchTgtVel = -velCapRad;
                if (rollTgtVel > velCapRad) rollTgtVel = velCapRad;
                else if (rollTgtVel < -velCapRad) rollTgtVel = -velCapRad;

                pitch.TargetVelocityRad = pitchTgtVel;
                roll.TargetVelocityRad = rollTgtVel;
            }

            // * echo important stuff
            {
            Echo($"F_grav: {F_grav}, \nF_directional: {F_directional}, \nF_final: {F_final}\n");
            Echo($"V_grav: {V_grav}, \nV_directional: {V_directional}, \nV_final: {V_final}\n");
            Echo($"\nThrustOverride: {thruster.ThrustOverride}\n");
            Echo($"\nMax effective thrust: {thruster.MaxEffectiveThrust}");
            }

            // * write to screens (lcds index hardcoded)
            // TODO dynamic lcd assignment, maybe position based
            {
                lcds[0].addLine($"Kp: {pitchPID.Kp} Ki: {pitchPID.Ki} Kd: {pitchPID.Kd}");
                lcds[0].addLineFormat("Ship mass: {0} kg, ", new string[]{mass.ToString()});
                lcds[0].addLine($"G-forces: {gForce:0.##} g");
                lcds[0].addLine(new string('\u2550', 36));
                lcds[0].addLine("Local gravity direction vector:");
                lcds[0].addLineVector3D(localGravDirection);
                lcds[0].addLineFormat("Local velocity direction vector: \n", new string[]{});
                lcds[0].addLineVector3D(SafeNormalize(localVelocity));
                lcds[0].addLine($"Stopping force @{stoppingTime:0.##}s: {F_damp:0.##} N");

                lcds[0].addLine("Ship weight vector:");
                lcds[0].addLineVector3D(shipWeightVec);
                lcds[0].addLine("Dampening direction vector:");
                lcds[0].addLineVector3D(SafeNormalize(V_damp));
                lcds[0].addLine($"Required velocity to dampen: {F_damp:0.##} m/s");
                lcds[0].addLineFormat("Local movement vector: ", new string[]{});
                lcds[0].addLineVector3D(moveInd, 3, false);
                lcds[0].addLine($"Desired movement force: {gearAccel:0.##} N");
                lcds[0].addLine(new string('\u2550', 36));
                lcds[0].addLine("Current thrust vector:");
                lcds[0].addLineVector3D(rotatedThrusterVec);
                lcds[0].addLine("Current target vector (dampening + movement):");
                lcds[0].addLineVector3D(SafeNormalize(requiredVec));
                lcds[0].addLine($"Current target force: {F_final:0.##} N");
                lcds[0].addLine($"Current force: {thruster.CurrentThrust:0.##} N");
                lcds[0].addLine(new string('\u2550', 36));
                
                lcds[0].addLineFormat("Current pitch: ", new string[]{});
                lcds[0].addLineAngle(pitch.Angle, WriteAngle.BOTH);
                lcds[0].addLineFormat("Required pitch: ", new string[]{});
                lcds[0].addLineAngle(requiredPitchAngle, WriteAngle.BOTH);
                lcds[0].addLineFormat("Pitch error: ", new string[]{});
                lcds[0].addLineAngle(pitchErr, WriteAngle.BOTH);
                lcds[0].addLineFormat("Pitch velocity: {0}rad | {1}rpm\n", new string[]{
                    pitch.TargetVelocityRad.ToString("0.##"), pitch.TargetVelocityRPM.ToString("0.##")
                });

                lcds[0].addLineFormat("Current roll: ", new string[]{});
                lcds[0].addLineAngle(roll.Angle, WriteAngle.BOTH);
                lcds[0].addLineFormat("Required roll: ", new string[]{});
                lcds[0].addLineAngle(requiredRollAngle, WriteAngle.BOTH);
                lcds[0].addLineFormat("Roll error: ", new string[]{});
                lcds[0].addLineAngle(rollErr, WriteAngle.BOTH);
                lcds[0].addLineFormat("Roll velocity: {0}rad | {1}rpm\n", new string[]{
                    roll.TargetVelocityRad.ToString("0.##"), roll.TargetVelocityRPM.ToString("0.##")
                });

                // lcds[0].addLine($"Current velocity: {localVelocity.Length():0.##}, error: {thrustErr:0.##}");
            }

            // write screens
            lcds.ForEach(e => {
                e.writeStringBuilder();
                e.sb.Clear();
            });

        }

        // not used, but i referenced this a bit 
        // source is the ingame logic for autopilot RCs (i think)
        public Vector3D ComputeThrust(Vector3D direction) {
            // https://github.com/KeenSoftwareHouse/SpaceEngineers/blob/master/Sources/Sandbox.Game/Game/EntityComponents/MyEntityThrustComponent.cs#L1361
            
            direction = SafeNormalize(direction);
            
            Matrix invWorldRot = MatrixD.Transpose(controller.WorldMatrix);

            // split direction vector into positive and negative components
            Vector3D positiveControl = Vector3D.Clamp(direction, Vector3D.Zero, Vector3D.One);
            Vector3D negativeControl = Vector3D.Clamp(direction, -Vector3D.One, Vector3D.Zero);

            // get positve and negative gravity applied to ship
            Vector3D gravity = controller.GetTotalGravity();
            float mass = controller.CalculateShipMass().PhysicalMass;
            Vector3D positiveGravity = Vector3D.Clamp(-Vector3D.Rotate(gravity, invWorldRot) * mass, Vector3D.Zero, Vector3D.PositiveInfinity);
            Vector3D negativeGravity = Vector3D.Clamp(-Vector3D.Rotate(gravity, invWorldRot) * mass, Vector3D.NegativeInfinity, Vector3D.Zero);

            // max desired thrust in positive and negative directions
            // "speed limit"
            Vector3D maxPositiveThrust = gearAccel * Vector3D.One.Normalized();
            Vector3D maxNegativeThrust = -1 * gearAccel * Vector3D.One.Normalized();

            Vector3D maxPositiveThrustWithGravity = Vector3D.Clamp(maxPositiveThrust - positiveGravity, Vector3D.Zero, Vector3D.PositiveInfinity);
            Vector3D maxNegativeThrustWithGravity = Vector3D.Clamp(maxNegativeThrust + negativeGravity, Vector3D.Zero, Vector3D.PositiveInfinity);

            Vector3D maxPositiveControl = maxPositiveThrustWithGravity * positiveControl;
            Vector3D maxNegativeControl = maxNegativeThrustWithGravity * -negativeControl;

            float max = (float) Math.Max(maxPositiveControl.Max(), maxNegativeControl.Max());

            Vector3D thrust = Vector3D.Zero;
            if (max > 0.001f)
            {
                Vector3D optimalPositive = positiveControl * maxPositiveControl;
                Vector3D optimalNegative = -negativeControl * maxNegativeControl;

                Vector3D optimalPositiveRatio = maxPositiveThrustWithGravity / optimalPositive;
                Vector3D optimalNegativeRatio = maxNegativeThrustWithGravity / optimalNegative;
                
                if (double.IsInfinity(optimalPositiveRatio.X) || double.IsNaN(optimalPositiveRatio.X))
                    optimalPositiveRatio.X = 1;
                if (double.IsInfinity(optimalPositiveRatio.Y) || double.IsNaN(optimalPositiveRatio.Y))
                    optimalPositiveRatio.Y = 1;
                if (double.IsInfinity(optimalPositiveRatio.Z) || double.IsNaN(optimalPositiveRatio.Z))
                    optimalPositiveRatio.Z = 1;

                if (double.IsInfinity(optimalNegativeRatio.X) || double.IsNaN(optimalNegativeRatio.X))
                    optimalNegativeRatio.X = 1;
                if (double.IsInfinity(optimalNegativeRatio.Y) || double.IsNaN(optimalNegativeRatio.Y))
                    optimalNegativeRatio.Y = 1;
                if (double.IsInfinity(optimalNegativeRatio.Z) || double.IsNaN(optimalNegativeRatio.Z))
                    optimalNegativeRatio.Z = 1;

                thrust = -optimalNegative * optimalNegativeRatio + optimalPositive * optimalPositiveRatio;
                thrust += positiveGravity + negativeGravity;
                thrust = Vector3D.Clamp(thrust, -maxNegativeThrust, maxPositiveThrust);
            }

            float STOPPING_TIME = 0.5f;
            Vector3D localVelocity = Vector3D.Rotate(controller.GetShipVelocities().LinearVelocity + gravity / 2.0f, invWorldRot);

            Vector3D velocityToCancel;
            if (!Vector3D.IsZero(direction))
            {
                Vector3D normalizedDir = Vector3D.Normalize(direction);
                velocityToCancel = Vector3D.Reject(localVelocity, normalizedDir);
            }
            else
            {
                velocityToCancel = localVelocity;
            }

            var slowdownAcceleration = -velocityToCancel / STOPPING_TIME;
            var slowdownThrust = slowdownAcceleration * mass;
            thrust = Vector3D.Clamp(thrust + slowdownThrust, -maxNegativeThrust, maxPositiveThrust);
            return thrust;
        }

        /// <summary> Derives the current thruster's vector based on the position of the ship's rotors. </summary>
        /// <param name="v">The vector to be converted.</param>
        /// <returns> A Vector3D containing the thruster's direction.</returns>
        public Vector3D CurrentThrusterDirectionVector() {
            double x, y, z;
            x = Math.Sin(roll.Angle);
            y = Math.Cos(pitch.Angle) * Math.Cos(roll.Angle);
            z = Math.Sin(pitch.Angle) * Math.Cos(roll.Angle);
            return new Vector3D(x, y, z);
        }

        /// <summary> Converts a vector to a pair of angles (pitch, roll). </summary>
        /// <param name="v">The vector to be converted.</param>
        /// <returns> A MyTuple containing two doubles. item1 = pitch angle, item2 = roll angle.</returns>
        public MyTuple<double, double> GetAnglesFromVector(VRageMath.Vector3 v){
            // inverse trig requires unit vector
            if (!Vector3D.IsZero(v)) v.Normalize();
            double x = v.X;
            double y = v.Y;
            double z = v.Z;
            float pitch, roll;
            roll = (float) Math.Asin(x);
            if (y == 0) {
                // fully pitched in some direction, but need to avoid dividing by 0
                pitch = Math.Sign(z) * piOver2;
            } else if (y > 0) {
                pitch = (float) Math.Atan(z / y);
            } else {
                if (z < 0) pitch = (float) (Math.Atan(z / y) - Math.PI);
                else pitch = (float) (Math.Atan(z / y) +  Math.PI);
            }
            return new MyTuple<double, double>(pitch, roll);
        }

        /// <summary> Tries to normalize a vector. </summary>
        /// <param name="v">The vector to be normalized.</param>
        /// <returns> The normalized form of the provided vector, or a zero vector if the provided vector is a zero vector.</returns>
        public Vector3D SafeNormalize(Vector3D v) {
            if (v.IsZero()) return v;
            else return v.Normalized();
        }

        /// <summary> Projects vector a onto vector b. </summary>
        /// <param name="a">The vector to be projected.</param>
        /// <param name="b">The vector to project onto.</param>
        /// <returns> The projection vector of a onto b.</returns>
        public static Vector3D Projection(Vector3D a, Vector3D b)
            {
                if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
                    return Vector3D.Zero;

                if (Vector3D.IsUnit(ref b))
                    return a.Dot(b) * b;

                return a.Dot(b) / b.LengthSquared() * b;
            }


        // just a class to help with logging to ingame text displays
        class LCD {
            // Monospace, font size 1.38 yields 37 characters wide by 10 characters tall
            // Left alignment, 2% text padding
            // Primary text color (255,204,0) background color (2,3,2)
            public IMyTextPanel block;
            public StringBuilder sb;

            public LCD(){}
            public LCD(IMyTextPanel block) {
                this.block = block;
                sb = new StringBuilder();
            }

            public void addLine(String line) {
                sb.Append(line + "\n");
            }

            public void addLineFormat(String format, String[] args) {
                sb.AppendFormat(format, args);
            }
            public void addLineVector3D(Vector3D v) {
                addLineVector3D(v, 3, true);
            }
            public void addLineVector3D(Vector3D v, int amtDecimalPlaces) {
                addLineVector3D(v, amtDecimalPlaces, true);
            }

            public void addLineVector3D(Vector3D v, int amtDecimalPlaces, bool trailZero) {
                string decFormat = "0." + new string(trailZero ? '0' : '#', Math.Max(1, amtDecimalPlaces));
                sb.AppendFormat("{0}x {1}y {2}z\n", v.X.ToString(decFormat), v.Y.ToString(decFormat), v.Z.ToString(decFormat));
            }
            
            public void addLineAngle(double a, WriteAngle wa) {
                switch (wa) {
                    case WriteAngle.RAD:
                        sb.AppendFormat("{0} rad\n", a.ToString("0.###"));
                        break;
                    case WriteAngle.DEG:
                        sb.AppendFormat("{0}\u00B0\n", (a * radToDeg).ToString("0.##"));
                        break;
                    case WriteAngle.BOTH:
                        sb.AppendFormat("{0} rad | {1}\u00B0\n", a.ToString("0.###"), (a * radToDeg).ToString("0.##"));
                        break;
                    default:
                        break;
                }

            }
            public void writeStringBuilder() {
                block.WriteText(sb.ToString());
            }
        }
        
        // PID code taken from whip's tutorial on MDK1 wiki
        // https://github.com/malware-dev/MDK-SE/wiki/PID-Controllers
        public class PID {
            public double Kp { get; set; } = 0;
            public double Ki { get; set; } = 0;
            public double Kd { get; set; } = 0;
            public double Value { get; private set; }

            double _timeStep = 0;
            double _inverseTimeStep = 0;
            double _errorSum = 0;
            double _lastError = 0;
            bool _firstRun = true;

            public PID(double kp, double ki, double kd, double timeStep) {
                Kp = kp;
                Ki = ki;
                Kd = kd;
                _timeStep = timeStep;
                _inverseTimeStep = 1 / _timeStep;
            }

            protected virtual double GetIntegral(double currentError, double errorSum, double timeStep) {
                return errorSum + currentError * timeStep;
            }

            public double Control(double error) {
                //Compute derivative term
                double errorDerivative = (error - _lastError) * _inverseTimeStep;

                if (_firstRun) {
                    errorDerivative = 0;
                    _firstRun = false;
                }

                //Get error sum
                _errorSum = GetIntegral(error, _errorSum, _timeStep);

                //Store this error as last error
                _lastError = error;

                //Construct output
                Value = Kp * error + Ki * _errorSum + Kd * errorDerivative;
                return Value;
            }

            public double Control(double error, double timeStep) {
                if (timeStep != _timeStep) {
                    _timeStep = timeStep;
                    _inverseTimeStep = 1 / _timeStep;
                }
                return Control(error);
            }

            public void Reset() {
                _errorSum = 0;
                _lastError = 0;
                _firstRun = true;
            }
        }

    }

}
