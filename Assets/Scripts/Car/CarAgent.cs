using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

namespace UnityStandardAssets.Vehicles.Car{

    public class CarAgent : Agent {


        private Rigidbody rb;
        public float envRadiusX = 3f;
        public float envRadiusZ = 10f;
        private CarController carController;
        EnvironmentParameters defaultParameters;
        private int steps = 0;
        private bool inTarget = false;
        public float inTargetMultiplier = 1.5f;
        public GameObject target;
        private RayPerceptionSensorComponent3D RayPerceptionSensorComponent;
        private Vector3 startPosition;
        private Quaternion startRotation;
        private Vector3 lastPosition;
        private bool isLookingForSpot=false;

        private void Reset(){
            Vector3 spawnPosition = new Vector3(startPosition.x, startPosition.y, startPosition.z);

            rb.transform.position = spawnPosition;
            rb.transform.rotation = startRotation;
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            steps = 0;
        }

        public override void Initialize(){
            carController = GetComponent<CarController>();
            rb = GetComponent<Rigidbody>();

            RayPerceptionSensorComponent = GetComponent<RayPerceptionSensorComponent3D>();

            defaultParameters = Academy.Instance.EnvironmentParameters;
            
            startPosition = transform.position;
            startRotation = transform.rotation;
            
            lastPosition = startPosition;

            Reset();
        }


        public override void OnEpisodeBegin(){
            Reset();
        }

        public override void CollectObservations(VectorSensor sensor){
            sensor.AddObservation(carController.CurrentSpeed);
        }

            private float CalculateReward(){
            
            // Compare the difference of the previous distance to target to the current one
            // If the agent got closer, reward it. Else penalize it.
            float reward = 0f;

            float totDirectionChangeReward = 0f;
            float totAngleChangeReward = 0f;
            float totDistanceReward = 0f;

            if(lastPosition != Vector3.zero){
                float distanceToTargetX = Mathf.Abs(transform.position.x - target.transform.position.x);
                float distanceToTargetZ = Mathf.Abs(transform.position.z - target.transform.position.z);

                float lastDistanceToTargetX = Mathf.Abs(lastPosition.x - target.transform.position.x);
                float lastDistanceToTargetZ = Mathf.Abs(lastPosition.z - target.transform.position.z);

                float directionChangeX = lastDistanceToTargetX - distanceToTargetX;
                float directionChangeZ = lastDistanceToTargetZ - distanceToTargetZ;

                totDirectionChangeReward = (directionChangeX + directionChangeZ) * 10f;
                totDirectionChangeReward = Mathf.Clamp(totDirectionChangeReward, -0.5f, 0.5f);

                float distanceRewardX = (1f - distanceToTargetX/envRadiusX);
                float distanceRewardZ = (1f - distanceToTargetZ/envRadiusZ);

                totDistanceReward = (distanceRewardX + distanceRewardZ) / 20f;

                reward += totDirectionChangeReward + totDistanceReward;
            }

            if(inTarget){
                float angleToTarget = Vector3.Angle(transform.forward, target.transform.forward);
                // When driving in the spot backwards, the angle to target is 180 degrees
                if(angleToTarget > 90f){
                    angleToTarget = 180f - angleToTarget;
                }

                angleToTarget = Mathf.Clamp(angleToTarget, 0f, 90f);
                float angleReward = (-(1f/45f) * angleToTarget) + 1f;

                totAngleChangeReward = angleReward + 1f;

                // Reward for minimising the angle to the target
                reward += totAngleChangeReward;

                float distanceToTarget = Vector3.Distance(transform.position, target.transform.position);

                // Check if car was able to park and reward it accordingly
                if(angleToTarget < 2.5f && distanceToTarget < 1f && Mathf.Abs(carController.CurrentSpeed) < 2f){
                    Debug.Log("Car parked!");
                    reward += 100f;
                    EndEpisode();
                }

            }

            lastPosition = transform.position;
            return reward;            
        }

        public override void OnActionReceived(ActionBuffers actions){
            float steering = actions.ContinuousActions[0];
            float accel = actions.ContinuousActions[1];
            float reverse = actions.ContinuousActions[2];

            // Input is from -1 to 1, map values accordingly
            accel = (accel + 1) / 2;
            reverse = (reverse + 1) / 2;

            accel = accel - reverse;
            
            if(!isLookingForSpot){
                carController.Move(steering, accel, 0f, 0f);
            }
            
            steps++;

            float reward = CalculateReward();
            AddReward(reward);
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;

            float steering = Input.GetAxis("Horizontal"); //-1 to 1
            float accel = Input.GetAxis("Accelerate");  //0 to 1
            float reverse = Input.GetAxis("Reverse");   //0 to 1

            // Input from network is between -1 to 1, map values accordingly
            accel = accel * 2 - 1;
            reverse = reverse * 2 - 1;

            continuousActionsOut[0] = steering;
            continuousActionsOut[1] = accel;
            continuousActionsOut[2] = reverse;
        }

        void OnTriggerEnter(Collider other)
        {
            if(other.gameObject.tag == "Finish"){
                inTarget = true;
            }
        }

        void OnTriggerExit(Collider other)
        {
            if(other.gameObject.tag == "Finish"){
                inTarget = false;
            }
        }

        void OnCollisionEnter(Collision collision)
        {
            print(collision.gameObject.tag);
            if (collision.gameObject.tag == "Wall")
            {
                AddReward(-10f);
                EndEpisode();
            }
        }

        void OnCollisionStay(Collision collision)
        {
            if (collision.gameObject.tag == "Kerb")
            {
                AddReward(-2f);
            }
            else if(collision.gameObject.tag == "Car")
            {
                
                float reward = -Mathf.Abs(carController.CurrentSpeed) * 50f - 5f;
                AddReward(reward);
                EndEpisode();
            }
        }




    }

}