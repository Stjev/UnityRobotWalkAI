using Assets.Scripts;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class Player : Agent, IPlayer
{
    private static readonly int LEVEL = 1;
    private Color hideColor;
    private Renderer bodyRenderer;

    /*
     * Get the target position
     */
    [SerializeField] private Transform Target;

    private Vector3 originalTargetPos;

    /*
     * Set the spawn point 
     */
    [SerializeField] private Transform Spawnpoint;

    /**
     * Get the different joints
     */
    [SerializeField] private GameObject frontRightLeg;
    [SerializeField] private GameObject frontLeftLeg;
    [SerializeField] private GameObject backRightLeg;
    [SerializeField] private GameObject backLeftLeg;

    [SerializeField] private RaySensors raySensor;

    private ArticulationBody[] allABs;
    private GameObject[] allLimbs;

    [Header("Force to be applied to all joints")]
    [SerializeField] private float force;

    private Rigidbody rBody;
    private ArticulationBody aBody;
    private LegControl LegControl;

    private Vector3 lastPos;
    private long timeSinceLastPos;

    // Start is called before the first frame update
    void Start()
    {
        // Execute level specific code
        this.OnStartLevelControl();

        // Get the initial color of the body
        this.bodyRenderer = GetComponent<Renderer>();
        this.hideColor = this.bodyRenderer.material.GetColor("_Color");

        // Save the target location
        this.originalTargetPos = Target.position;

        // The rigidbody of the body
        this.rBody = GetComponent<Rigidbody>();
        this.aBody = GetComponent<ArticulationBody>();

        this.allABs = new ArticulationBody[] { frontRightLeg.GetComponentsInChildren<ArticulationBody>()[0],
                                               frontRightLeg.GetComponentsInChildren<ArticulationBody>()[1],
                                               frontLeftLeg.GetComponentsInChildren<ArticulationBody>()[0],
                                               frontLeftLeg.GetComponentsInChildren<ArticulationBody>()[1],
                                               backRightLeg.GetComponentsInChildren<ArticulationBody>()[0],
                                               backRightLeg.GetComponentsInChildren<ArticulationBody>()[1],
                                               backLeftLeg.GetComponentsInChildren<ArticulationBody>()[0],
                                               backLeftLeg.GetComponentsInChildren<ArticulationBody>()[1]};

        this.LegControl = new LegControl(this.allABs[0], this.allABs[1],
                                         this.allABs[2], this.allABs[3],
                                         this.allABs[4], this.allABs[5],
                                         this.allABs[6], this.allABs[7]);

        List<GameObject> tempLimbs = new List<GameObject>();

        foreach (GameObject leg in new GameObject[] { frontRightLeg, frontLeftLeg, backLeftLeg, backRightLeg })
        {
            foreach (Rigidbody legRBody in leg.GetComponentsInChildren<Rigidbody>())
            {
                tempLimbs.Add(legRBody.gameObject);
            }
        }

        this.allLimbs = tempLimbs.ToArray();
    }

    /**
     * This is fired at the start of a new training example.
     * 
     * A new training example starts when the agent reached the goal or 
     * fell of the field or got stuck.
     */
    public override void OnEpisodeBegin()
    {
        this.OnEpisodeBeginLevelControl();

        // Reset the legs of the Agent
        this.LegControl.ResetLegs();

        // Zero its momentum (in case of falling)
        this.rBody.angularVelocity = Vector3.zero;
        this.rBody.velocity = Vector3.zero;

        this.aBody.TeleportRoot(Spawnpoint.position, Quaternion.Euler(90, 0, 0));

        // Reset the lastPos and the timeSinceLastPos
        this.lastPos = this.transform.position;
        this.timeSinceLastPos = 0;
    }

    /**
     * The observations the Agent will get
     * 
     * There are 3 + 3 + 3 + 3 + 12 + 4 + 8 + 8 + 15 = 59 sensors
     */
    public override void CollectObservations(VectorSensor sensor)
    {
        // Target position relative to the Agents position
        sensor.AddObservation(this.Target.position - this.transform.position);    // 3

        // Agent rotation
        sensor.AddObservation(this.transform.rotation); // 3
        // Agent Velocity and Angular Velocity
        sensor.AddObservation(this.rBody.velocity); // 3
        sensor.AddObservation(this.rBody.angularVelocity); // 3

        // Add all relevant observations for each limb
        foreach (GameObject limb in this.allLimbs)
        {
            sensor.AddObservation(limb.transform.localPosition); // 12
            sensor.AddObservation(limb.transform.localRotation.y); // 4
        }

        foreach (ArticulationBody joint in this.allABs)
        {
            sensor.AddObservation(joint.velocity); // 8
            sensor.AddObservation(joint.angularVelocity); // 8
        }

        // Add the distances from every ray
        sensor.AddObservation(raySensor.GetRayDistances()); // 15
    }

    private static readonly int MOVEMENTTIME = 25000;
    private static readonly float MIN_MOVEDISTANCE = 0.05f;


    /**
     * This method checks if the agent has moved for a certain amount of time
     */
    public void CheckIfStuck()
    {
        this.timeSinceLastPos += 1;

        float distanceMoved = Vector3.Distance(this.lastPos, this.transform.position);

        // The Agent should move to accomplish its goal, this will make sure the Agent resets when
        // no movement was detected for a while
        if (distanceMoved < MIN_MOVEDISTANCE && this.timeSinceLastPos > MOVEMENTTIME)
        {
            Die();
        }
        else if (distanceMoved > MIN_MOVEDISTANCE)
        {
            this.lastPos = this.transform.position;
            this.timeSinceLastPos = 0;
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Actions, size = 8, one for each joint
        this.LegControl.MoveFrontRightShoulder(actions.ContinuousActions[0] * 100);
        this.LegControl.MoveFrontRightKnee(actions.ContinuousActions[1] * 100);
        this.LegControl.MoveBackRightShoulder(actions.ContinuousActions[2] * 100);
        this.LegControl.MoveBackRightKnee(actions.ContinuousActions[3] * 100);
        this.LegControl.MoveFrontLeftShoulder(actions.ContinuousActions[4] * 100);
        this.LegControl.MoveFrontLeftKnee(actions.ContinuousActions[5] * 100);
        this.LegControl.MoveBackLeftShoulder(actions.ContinuousActions[6] * 100);
        this.LegControl.MoveBackLeftKnee(actions.ContinuousActions[7] * 100);

        // Check if the agent got stuck (or just hasn't moved for quite some time)
        this.CheckIfStuck();

        // If the Agent reached the target
        if (Vector3.Distance(this.transform.position, this.Target.position) < 1.0f)
        {
            // The discount factor is the angle the player looked away from the Target 
            // squared. We square because we don't care if the Agent almost looked at the Target
            // but rather if the agent looked far from the target
            float discountFactor = (float) (Math.Pow(this.GetAngleTargetAgent(), 2) / Math.Pow(180, 2));

            // The calculation of the reward:
            //  1 + (angle^2/180^2) * 0.9
            // Changing the last constant multiplication value changes
            // the weight of how much we want the agent to look at the goal
            // (higher = We wint him to look at the goal more)
            SetReward(1.0f - discountFactor * 0.9f);

            this.OnGoalReachedLevelControl();
        }
        // The Agent fell off the edge or fell on the ground
        else if (this.transform.localPosition.y < -0.2f)
        {
            Die();
        }
    }

    /**
     * Calculates the angle between where the player is looking and where
     * the target is
     */
    public float GetAngleTargetAgent()
    {
        // Calculate the angle between the Agent and the Target
        Vector3 targetDir = Target.position - transform.position;
        return Vector3.Angle(new Vector3(targetDir.x, 0, targetDir.z), new Vector3(transform.up.x, 0, transform.up.z));
    }

    /**
     * Kill the player (if he fell)
     */
    public void Die()
    {
        EndEpisode();
    }

    /**
     * This is some (very limited) manual control over the agent. 
     * This will just apply the force to all joints at the same time
     */
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Let all actions just be the force
        var continuousActionsOut = actionsOut.ContinuousActions;
        for (int i = 0; i < 8; i += 1) continuousActionsOut[i] = force;
    }

    #region LevelControl
    private void OnStartLevelControl()
    {
        switch (LEVEL)
        {
            case 1:
                LevelControl.Level1.OnStart(Target, Spawnpoint);
                break;
            case 2:
                LevelControl.Level2.OnStart();
                break;
        }
    }

    private void OnEpisodeBeginLevelControl()
    {
        switch (LEVEL)
        {
            case 1:
                LevelControl.Level1.OnEpisodeBegin(this.bodyRenderer);
                break;
            case 2:
                LevelControl.Level2.OnEpisodeBegin(Target, this.originalTargetPos);
                break;
        }
    }

    private void OnGoalReachedLevelControl()
    {
        switch (LEVEL)
        {
            case 1:
                LevelControl.Level1.OnGoalReached(Target, Spawnpoint, this.bodyRenderer, this.hideColor);
                break;
            case 2:
                LevelControl.Level2.OnGoalReached(Target, Spawnpoint, this.originalTargetPos);
                break;
        }
    }

    #endregion
}
