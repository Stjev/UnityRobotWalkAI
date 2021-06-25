using Assets.Scripts;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class Player : Agent
{
    /*
     * Get the target position
     */
    [SerializeField] private Transform Target;

    /*
     * Set the spawn point 
     */
    [SerializeField] private Transform Spawnpoint;

    /**
     * Get the different joints
     */
    [Header("Front Right Leg")]
    [SerializeField] private ArticulationBody frShoulder;
    [SerializeField] private ArticulationBody frKnee;

    [Header("Back Right Leg")]
    [SerializeField] private ArticulationBody brShoulder;
    [SerializeField] private ArticulationBody brKnee;

    [Header("Front Left Leg")]
    [SerializeField] private ArticulationBody flShoulder;
    [SerializeField] private ArticulationBody flKnee;

    [Header("Back Left Leg")]
    [SerializeField] private ArticulationBody blShoulder;
    [SerializeField] private ArticulationBody blKnee;

    [Header("Force to be applied to all joints")]
    [SerializeField] private float force;

    private Rigidbody rBody;
    private ArticulationBody aBody;
    private LegControl LegControl;

    private Vector3 lastPos;
    private long timeSinceLastPos;

    private int lastDistance;

    private float distanceStartGoal;

    // Start is called before the first frame update
    void Start()
    {
        // Get the distance from the goal to the target
        this.distanceStartGoal = Vector3.Distance(this.transform.position, this.Target.position);
        // The rigidbody of the body
        this.rBody = GetComponent<Rigidbody>();
        this.aBody = GetComponent<ArticulationBody>();
        this.LegControl = new LegControl(frShoulder, frKnee, brShoulder, brKnee, flShoulder, flKnee, blShoulder, blKnee);
    }

    /**
     * This is fired at the start of a new training example.
     * 
     * A new training example starts when the agent reached the goal or 
     * fell of the field or got stuck.
     */
    public override void OnEpisodeBegin()
    {
        // The last distance will be the spawn point
        this.lastDistance = (int)Math.Floor(this.distanceStartGoal);

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
     * There are 17 sensors
     */
    public override void CollectObservations(VectorSensor sensor)
    {
        // Target and Agent position
        sensor.AddObservation(this.transform.position); // 3
        sensor.AddObservation(this.Target.position);    // 3

        // Agent rotation
        sensor.AddObservation(this.transform.localRotation); // 3

        // The position of all the legs of the Agent
        sensor.AddObservation(this.frShoulder.xDrive.target); // 1
        sensor.AddObservation(this.frKnee.xDrive.target);     // 1
        sensor.AddObservation(this.brShoulder.xDrive.target); // 1
        sensor.AddObservation(this.brKnee.xDrive.target);     // 1
        sensor.AddObservation(this.flShoulder.xDrive.target); // 1
        sensor.AddObservation(this.flKnee.xDrive.target);     // 1 
        sensor.AddObservation(this.blShoulder.xDrive.target); // 1
        sensor.AddObservation(this.blKnee.xDrive.target);     // 1
    }

    private float GetReward()
    {
        // The best distance is currently the maximum value
        int currentDistance = (int)Math.Floor(Vector3.Distance(this.transform.position, this.Target.position));

        // The docs advise to not give out too many rewards, so only give a 
        // reward every 2 units the agents has moved
        if (currentDistance % 2 == 0)
        {
            return -100;
        }

        if (currentDistance < this.lastDistance)
        { // The best distance has improved, give a reward
            this.lastDistance = currentDistance;
            return 0.1f;
        } else if (currentDistance > this.lastDistance)
        { // The current distance is worse than before, return a negative reward
            this.lastDistance = currentDistance;
            return -0.1f;
        }

        return -100; 
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        this.timeSinceLastPos += 1;

        // Actions, size = 8, one for each joint
        this.LegControl.MoveFrontRightShoulder(actions.ContinuousActions[0]);
        this.LegControl.MoveFrontRightKnee(actions.ContinuousActions[1]);
        this.LegControl.MoveBackRightShoulder(actions.ContinuousActions[2]);
        this.LegControl.MoveBackRightKnee(actions.ContinuousActions[3]);
        this.LegControl.MoveFrontLeftShoulder(actions.ContinuousActions[4]);
        this.LegControl.MoveFrontLeftKnee(actions.ContinuousActions[5]);
        this.LegControl.MoveBackLeftShoulder(actions.ContinuousActions[6]);
        this.LegControl.MoveBackLeftKnee(actions.ContinuousActions[7]);

        // Rewards 
        float distanceToTarget = Vector3.Distance(this.transform.position, this.Target.position);

        float reward = this.GetReward();

        if (reward > -50)
        {
            Debug.Log(reward);
            SetReward(reward);
        }

        // The Agent should move to accomplish its goal, this will make sure the Agent resets when
        // no movement was detected for a while
        if (Vector3.Distance(this.lastPos, this.transform.position) < 0.1 && this.timeSinceLastPos > 25000)
        {
            EndEpisode();
        }
        else if (Vector3.Distance(this.lastPos, this.transform.position) > 0.5)
        {
            this.lastPos = this.transform.position;
            this.timeSinceLastPos = 0;
        }

        // If the Agent reached the target
        if (distanceToTarget < 1.4f)
        {
            SetReward(1);
            EndEpisode();
        }
        // The Agent fell off the edge
        else if (this.transform.position.y < -1)
        {
            EndEpisode();
        }
    }

    /**
     * This is some (very limited) manual control over the agent. 
     * This will just apply the force to all joints at the same time
     */
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Let all actions just be the force
        var continuousActionsOut = actionsOut.ContinuousActions;
        for(int i = 0; i < 8; i += 1) continuousActionsOut[i] = force;
    }
}
