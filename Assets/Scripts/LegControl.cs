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
        private void MoveLeg(ArticulationBody body, float target)
        {
            ArticulationDrive drive = body.xDrive;
            drive.target = Mathf.Clamp(target, drive.lowerLimit, drive.upperLimit);
            body.xDrive = drive;
        }

        /**
         * Reset the positions of all the legs to a new random position
         */
        public void ResetLegs(bool random=false)
        {
            foreach(ArticulationBody body in new List<ArticulationBody> { this.frShoulder, this.frKnee, this.brShoulder, this.brKnee, this.flShoulder, this.flKnee, this.blShoulder, this.blKnee })
            {
                ArticulationDrive drive = body.xDrive;
                drive.target = random? r.Next((int)drive.lowerLimit, (int)drive.upperLimit) : 0;
                body.xDrive = drive;
            }
        }

        public void MoveFrontRightShoulder(float target)
        {
            this.MoveLeg(this.frShoulder, target);
        }
        public void MoveFrontRightKnee(float target)
        {
            this.MoveLeg(this.frKnee, target);
        }
        public void MoveBackRightShoulder(float target)
        {
            this.MoveLeg(this.brShoulder, target);
        }
        public void MoveBackRightKnee(float target)
        {
            this.MoveLeg(this.brKnee, target);
        }
        public void MoveFrontLeftShoulder(float target)
        {
            this.MoveLeg(this.flShoulder, -target);
        }
        public void MoveFrontLeftKnee(float target)
        {
            this.MoveLeg(this.flKnee, -target);
        }
        public void MoveBackLeftShoulder(float target)
        {
            this.MoveLeg(this.blShoulder, -target);
        }
        public void MoveBackLeftKnee(float target)
        {
            this.MoveLeg(this.blKnee, -target);
        }
    }
}