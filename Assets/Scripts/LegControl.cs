using System;
using System.Collections.Generic;
using Unity.MLAgents.Actuators;
using UnityEditor;
using UnityEngine;

namespace Assets.Scripts
{
    public class LegControl
    {
        // All the different joints
        private ArticulationBody frShoulder, frKnee, brShoulder, brKnee, flShoulder, flKnee, blShoulder, blKnee;

        private static System.Random r = new System.Random();

        /**
         * Constructor with all the different legs as parameters
         */
        public LegControl(ArticulationBody frShoulder,
                          ArticulationBody frKnee,
                          ArticulationBody brShoulder,
                          ArticulationBody brKnee,
                          ArticulationBody flShoulder,
                          ArticulationBody flKnee,
                          ArticulationBody blShoulder,
                          ArticulationBody blKnee)
        {
            this.frShoulder = frShoulder;
            this.frKnee = frKnee;
            this.brShoulder = brShoulder;
            this.brKnee = brKnee;
            this.flShoulder = flShoulder;
            this.flKnee = flKnee;
            this.blShoulder = blShoulder;
            this.blKnee = blKnee;
        }
        private void MoveLeg(ArticulationBody body, float force)
        {
            ArticulationDrive drive = body.xDrive;
            drive.target = Mathf.Clamp(drive.target + force, drive.lowerLimit, drive.upperLimit);
            body.xDrive = drive;
        }

        /**
         * Reset the positions of all the legs to a new random position
         */
        public void ResetLegs()
        {
            foreach(ArticulationBody body in new List<ArticulationBody> { this.frShoulder, this.frKnee, this.brShoulder, this.brKnee, this.flShoulder, this.flKnee, this.blShoulder, this.blKnee })
            {
                ArticulationDrive drive = body.xDrive;
                drive.target = r.Next((int)drive.lowerLimit, (int)drive.upperLimit);
                body.xDrive = drive;
            }
        }

        public void MoveFrontRightShoulder(float force)
        {
            this.MoveLeg(this.frShoulder, force);
        }
        public void MoveFrontRightKnee(float force)
        {
            this.MoveLeg(this.frKnee, force);
        }
        public void MoveBackRightShoulder(float force)
        {
            this.MoveLeg(this.brShoulder, force);
        }
        public void MoveBackRightKnee(float force)
        {
            this.MoveLeg(this.brKnee, force);
        }
        public void MoveFrontLeftShoulder(float force)
        {
            this.MoveLeg(this.flShoulder, -force);
        }
        public void MoveFrontLeftKnee(float force)
        {
            this.MoveLeg(this.flKnee, -force);
        }
        public void MoveBackLeftShoulder(float force)
        {
            this.MoveLeg(this.blShoulder, -force);
        }
        public void MoveBackLeftKnee(float force)
        {
            this.MoveLeg(this.blKnee, -force);
        }
    }
}