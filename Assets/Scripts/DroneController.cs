using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.MLAgents;
using Unity.MLAgents.Actuators; // This helps to agents to take actions.
using Unity.MLAgents.Sensors; // This helps agent to sense environment.
using UnityEngine;

using Random = UnityEngine.Random;

// Define a class DroneController, inherting from Unity's Agent (part of ML-Agents)
public class DroneController : Agent
{
    public float upForce;  // force applied to lift drone upwards
    private float MovementForwardSpeed = 500f; // speed for forward movement
    private float tilteAmountForward = 0; // amount to tilt dron forward
    private float tilteVelocityForward; // helps smooth out tilt changes over time

    // this variables help in yaw rotation of drone, i copied it from github/chatgpt need help to understand better
    private float wantedYRotation;  // desired rotatoion angle for drone
    private float currentYRotation; // current rotation angle for drone
    private float rotationAmount = 2.5f; // amount to rotate per input
    private float rotationYVelocity; // to smooth out the rotaton

    Rigidbody ourDrone; // stores the rigid body component for physics-based movement

    [SerializeField] private Transform targetTransform; //reference to the target the drone should reach

    // Called at the start of each training episode to reset the environment
    public override void OnEpisodeBegin()
    {
        // Randomly place the drone within a specific area to vary the start position each episode
        transform.localPosition = new Vector3(Random.Range(-90f, 80f), 0f, Random.Range(-120f,0f)); 
        // Randomly place the target within a different specific area, giving the agent new challenges
        targetTransform.localPosition = new Vector3(Random.Range(-75f, 50f), 0f, Random.Range(-80f, -5f)); 

    }


    // Collects information about the environment to give to the agent's neural network
    public override void CollectObservations(VectorSensor sensor)
    {

        // Agent's current position in the environment
        sensor.AddObservation(transform.localPosition.x); // x-coordinate of the drone
        sensor.AddObservation(transform.localPosition.y); // y-coordinate of the drone
        sensor.AddObservation(transform.localPosition.z); // z-coordinate of the drone

        // Target's position, which is needed to find and move towards it
        sensor.AddObservation(targetTransform.localPosition.x); // x-coordinate of the target
        sensor.AddObservation(targetTransform.localPosition.y); // y-coordinate of the target
        sensor.AddObservation(targetTransform.localPosition.z); // z-coordinate of the target

        // Distance between the drone and the target (useful to know how close or far it is)
        sensor.AddObservation(Vector3.Distance(targetTransform.localPosition, transform.localPosition));

    }


    // Executes actions based on the ML model's output or user input (for testing)
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Extract movement actions from the ML model's output
        float moveX = actions.ContinuousActions[0] * 10; // Move left/right
        float moveY = actions.ContinuousActions[1] * 10; // Move up/down
        float moveZ = actions.ContinuousActions[2] * 10; // Move forward/backward

        // doesn't work for now
        //float upForce_up = upForce + actions.ContinuousActions[0]; ;
        //float upForce_down = upForce + actions.ContinuousActions[1]; ; 

        //ourDrone.AddRelativeForce(Vector3.up * upForce_up);

       // ourDrone.rotation = Quaternion.Euler(new Vector3(tilteAmountForward, currentYRotation, ourDrone.rotation.z));

        // Control the speed of movement for smoother adjustments
        float moveSpeed = 10f;

        // Update the drone's position based on the actions, with Time.deltaTime for frame consistency
        transform.localPosition += new Vector3(moveX, moveY, moveZ) * Time.deltaTime * moveSpeed;

        // Apply a small penalty for each action to encourage reaching the target quickly
        AddReward(-0.01f);
    }

    // Provides manual control for testing with keyboard input when training is disabled
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Define a segment for continuous actions (movements)
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;

        // Set left-right movement based on "Horizontal" input (like A/D or Left/Right arrows)
        continuousActions[0] = Input.GetAxisRaw("Horizontal");

        // Set forward-backward movement based on "Vertical" input (like W/S or Up/Down arrows)
        continuousActions[2] = Input.GetAxisRaw("Vertical");
    }

    // Detects collisions to check if the drone hits a target or obstacle
    private void OnTriggerEnter(Collider other)
    {
        // If the drone hits a wall, it’s penalized and the episode restarts
        if (other.TryGetComponent<Wall>(out Wall wall))
        {
            SetReward(-1f); // Large negative reward for hitting an obstacle
            EndEpisode(); // End the episode
        }

        // If the drone reaches the goal, it’s rewarded and the episode restarts
        if (other.TryGetComponent<Goal>(out Goal goal))
        {
            SetReward(1f); // Positive reward for reaching the goal
            EndEpisode(); // End the episode
        }
    }

   

    // Placeholder for forward movement function, used for manual controls (not in use here)
    void MovementForward()
    {
        if (Input.GetAxis("Vertical") != 0) // Check for user input
        {
            // Apply force in the forward direction for manual control
            ourDrone.AddRelativeForce(Vector3.forward * Input.GetAxis("Vertical") * MovementForwardSpeed);
        }
    }

    // Placeholder for rotation function, used for manual controls (not in use here)
    void Rotation()
    {
        if (Input.GetKey(KeyCode.L)) // Check if 'L' key is pressed
        {
            wantedYRotation += rotationAmount; // Rotate right
        }
        if (Input.GetKey(KeyCode.J)) // Check if 'J' key is pressed
        {
            wantedYRotation -= rotationAmount; // Rotate left
        }

        // Smoothly adjust current rotation to the desired rotation for a gradual turning effect
        currentYRotation = Mathf.SmoothDamp(currentYRotation, wantedYRotation, ref rotationYVelocity, 0.25f);
    }
}
